#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>

#include "sim_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include "sim_cartesian_velocity_control/cartesian_dynamics_controller.h"

namespace controller_interface
{
namespace
{

/** \brief Convert a urdf inertial block to a KDL rigid body inertia
 *
 * The same conversion kdl_parser does internally: the URDF gives the
 * rotational inertia about the centre of mass in the inertial frame, KDL wants
 * it in the link frame, and the only way to rotate a RotationalInertia with
 * KDL's own operators is through a RigidBodyInertia of zero mass.
 */
KDL::RigidBodyInertia toKdlInertia(const urdf::InertialSharedPtr &i)
{
  const KDL::Frame origin(
      KDL::Rotation::Quaternion(i->origin.rotation.x, i->origin.rotation.y,
                                i->origin.rotation.z, i->origin.rotation.w),
      KDL::Vector(i->origin.position.x, i->origin.position.y,
                  i->origin.position.z));
  const KDL::RotationalInertia urdf_inertia(i->ixx, i->iyy, i->izz,
                                            i->ixy, i->ixz, i->iyz);
  const KDL::RigidBodyInertia rotated =
      origin.M * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), urdf_inertia);
  return KDL::RigidBodyInertia(i->mass, origin.p, rotated.getRotationalInertia());
}

KDL::Frame jointOrigin(const urdf::Joint &j)
{
  return KDL::Frame(
      KDL::Rotation::Quaternion(j.parent_to_joint_origin_transform.rotation.x,
                                j.parent_to_joint_origin_transform.rotation.y,
                                j.parent_to_joint_origin_transform.rotation.z,
                                j.parent_to_joint_origin_transform.rotation.w),
      KDL::Vector(j.parent_to_joint_origin_transform.position.x,
                  j.parent_to_joint_origin_transform.position.y,
                  j.parent_to_joint_origin_transform.position.z));
}

/** \brief Sum the rigid body inertia of everything attached beyond a link,
 * expressed in that link's frame
 *
 * KDL builds its chain along one path, so a chain that ends at ee_link carries
 * none of the hardware bolted to ee_link. On this model that is the Robotiq
 * coupler, the gripper and the camera: 0.767 kg, hanging at the far end of a
 * 17 kg arm. Leaving it out is not a rounding error. Measured at the SRDF
 * 'init' pose, folding it in changes the gravity torque at shoulder_lift from
 * 14.645 Nm to 18.848 Nm (+29 %), the apparent mass at the end effector from
 * 3.40/4.14/7.76 kg to 4.16/4.91/8.52 kg, and the gravity wrench p from 36.9 N
 * to 44.4 N. It also takes the condition number of J M^-1 J^T from about
 * 59000 down to about 1000, because without it wrist_3 is left carrying the
 * 0.0001 kg placeholder inertia the gripper links are given.
 *
 * The walk treats every distal joint as fixed at zero. On this model the only
 * movable ones are the gripper knuckles, whose links weigh 0.0001 kg each, so
 * the error is below anything else here. A heavier moving tool would want its
 * joints read.
 */
KDL::RigidBodyInertia accumulateDistalInertia(const urdf::Link &link, double *mass)
{
  KDL::RigidBodyInertia total = KDL::RigidBodyInertia::Zero();
  for (std::size_t k = 0; k < link.child_links.size(); k++)
  {
    const urdf::Link &child = *link.child_links[k];
    KDL::RigidBodyInertia child_inertia = KDL::RigidBodyInertia::Zero();
    if (child.inertial)
    {
      child_inertia = toKdlInertia(child.inertial);
      *mass += child.inertial->mass;
    }
    child_inertia = child_inertia + accumulateDistalInertia(child, mass);
    total = total + jointOrigin(*child.parent_joint) * child_inertia;
  }
  return total;
}

} // anonymous namespace

bool CartesianDynamicsController::init(hardware_interface::JointStateInterface *robot,
                                       ros::NodeHandle &n)
{
  if (!KinematicChainControllerBase<hardware_interface::JointStateInterface>::init(robot, n))
    return false;

  if (!n.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }
  n.param("sigma_min", sigma_min_, 0.05);
  n.param("pinv_rcond", pinv_rcond_, 1e-6);
  n.param("include_tip_payload", include_tip_payload_, true);

  // Gravity in the root frame. On this model every joint from world to
  // base_link has rpy 0 and the only rotation in between is r_joint about z,
  // which leaves gravity along -z whatever the gantry does. A different
  // mounting needs this set.
  std::vector<double> gravity_param;
  KDL::Vector gravity(0.0, 0.0, -9.81);
  if (n.getParam("gravity", gravity_param))
  {
    if (gravity_param.size() != 3)
    {
      ROS_ERROR("Parameter 'gravity' must have three elements, got %lu",
                gravity_param.size());
      return false;
    }
    gravity = KDL::Vector(gravity_param[0], gravity_param[1], gravity_param[2]);
  }

  dyn_chain_ = this->kdl_chain_;

  if (include_tip_payload_)
  {
    std::string robot_description, xml;
    if (!ros::param::search(this->nh_.getNamespace(), "robot_description", robot_description) ||
        !this->nh_.getParam(robot_description, xml))
    {
      ROS_ERROR("Could not re-read the robot description to find the tip payload");
      return false;
    }
    urdf::Model model;
    if (!model.initString(xml))
    {
      ROS_ERROR("Could not parse the robot description");
      return false;
    }
    std::string tip_name;
    this->nh_.getParam(this->nh_.getNamespace() + "/tip_name", tip_name);
    urdf::LinkConstSharedPtr tip = model.getLink(tip_name);
    if (!tip)
    {
      ROS_ERROR("Tip link '%s' is not in the robot description", tip_name.c_str());
      return false;
    }

    double walked_mass = 0.0;
    const KDL::RigidBodyInertia payload = accumulateDistalInertia(*tip, &walked_mass);
    if (payload.getMass() > 0.0)
    {
      const KDL::Segment &old_tip = dyn_chain_.getSegment(dyn_chain_.getNrOfSegments()-1);
      KDL::Chain extended;
      for (unsigned int s = 0; s + 1 < dyn_chain_.getNrOfSegments(); s++)
        extended.addSegment(dyn_chain_.getSegment(s));
      extended.addSegment(KDL::Segment(old_tip.getName(), old_tip.getJoint(),
                                       old_tip.getFrameToTip(),
                                       old_tip.getInertia() + payload));
      dyn_chain_ = extended;
      ROS_INFO("Folded %.4f kg attached beyond %s into the dynamics model, "
               "centre of mass (%.4f %.4f %.4f) in that frame",
               payload.getMass(), tip_name.c_str(),
               payload.getCOG().x(), payload.getCOG().y(), payload.getCOG().z());
    }
    else
    {
      ROS_INFO("Nothing with mass is attached beyond %s", tip_name.c_str());
    }
  }

  dyn_param_.reset(new KDL::ChainDynParam(dyn_chain_, gravity));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(dyn_chain_));
  jac_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(dyn_chain_));
  // Reference frame base, reference point end effector - the same convention
  // ChainJntToJacSolver uses, so J and Jdot agree. This is the solver's
  // default, and it is set through a local copy rather than with
  // setHybridRepresentation() because KDL declares HYBRID as a static const
  // member without defining it anywhere in the library: taking a reference to
  // it, which setRepresentation(const int&) does, leaves this whole plugin
  // library with an undefined symbol and nothing in it can be loaded.
  const int hybrid = KDL::ChainJntToJacDotSolver::HYBRID;
  jac_dot_solver_->setRepresentation(hybrid);

  const unsigned int n_joints = dyn_chain_.getNrOfJoints();
  mass_.resize(n_joints);
  gravity_torque_.resize(n_joints);
  coriolis_torque_.resize(n_joints);
  jacobian_.resize(n_joints);

  mass_inv_ = Eigen::MatrixXd::Zero(n_joints, n_joints);
  j_mass_inv_ = Eigen::MatrixXd::Zero(6, n_joints);
  inertia_inv_ = Eigen::MatrixXd::Zero(6, 6);
  inertia_ = Eigen::MatrixXd::Zero(6, 6);
  coriolis_ = Eigen::VectorXd::Zero(n_joints);
  gravity_ = Eigen::VectorXd::Zero(n_joints);
  jac_dot_q_dot_ = Eigen::VectorXd::Zero(6);
  svd_jacobian_ = Eigen::JacobiSVD<Eigen::MatrixXd>(6, n_joints);
  svd_inertia_inv_ = Eigen::JacobiSVD<Eigen::MatrixXd>(
      6, 6, Eigen::ComputeFullU | Eigen::ComputeFullV);

  realtime_pub_.reset(new realtime_tools::RealtimePublisher<
      sim_cartesian_velocity_control::OperationalSpaceDynamics>(n, "ee_dynamics", 4));

  // The parts of the message that never change
  sim_cartesian_velocity_control::OperationalSpaceDynamics &msg = realtime_pub_->msg_;
  this->nh_.getParam(this->nh_.getNamespace() + "/root_name", msg.root_name);
  this->nh_.getParam(this->nh_.getNamespace() + "/tip_name", msg.tip_name);
  msg.joint_names.clear();
  for (unsigned int s = 0; s < dyn_chain_.getNrOfSegments(); s++)
    if (dyn_chain_.getSegment(s).getJoint().getType() != KDL::Joint::None)
      msg.joint_names.push_back(dyn_chain_.getSegment(s).getJoint().getName());
  msg.position.resize(n_joints);
  msg.velocity.resize(n_joints);

  ROS_INFO("Cartesian dynamics controller: %u joints, gravity (%.2f %.2f %.2f) "
           "in %s, sigma_min %.4f, pinv rcond %.1e",
           n_joints, gravity.x(), gravity.y(), gravity.z(),
           msg.root_name.c_str(), sigma_min_, pinv_rcond_);
  return true;
}

void CartesianDynamicsController::starting(const ros::Time& time)
{
  this->resetJointStateEstimate();
  last_publish_time_ = time;
}

void CartesianDynamicsController::update(const ros::Time& time, const ros::Duration& period)
{
  // The velocity estimate is a filter, so it has to be stepped every cycle
  // even on the cycles that do not publish.
  this->readJointStates(period);

  if (publish_rate_ <= 0.0)
    return;
  if (last_publish_time_ + ros::Duration(1.0/publish_rate_) >= time)
    return;
  if (!realtime_pub_->trylock())
    return;

  last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
  computeDynamics(time);
  realtime_pub_->unlockAndPublish();
}

void CartesianDynamicsController::computeDynamics(const ros::Time& time)
{
  sim_cartesian_velocity_control::OperationalSpaceDynamics &msg = realtime_pub_->msg_;
  const unsigned int n = dyn_chain_.getNrOfJoints();

  dyn_param_->JntToMass(this->joint_msr_.q, mass_);
  dyn_param_->JntToGravity(this->joint_msr_.q, gravity_torque_);
  dyn_param_->JntToCoriolis(this->joint_msr_.q, this->joint_msr_.qdot, coriolis_torque_);
  jac_solver_->JntToJac(this->joint_msr_.q, jacobian_);

  KDL::Twist jac_dot_q_dot;
  jac_dot_solver_->JntToJacDot(this->joint_msr_, jac_dot_q_dot);
  for (int i = 0; i < 3; i++)
  {
    jac_dot_q_dot_(i)     = jac_dot_q_dot.vel(i);
    jac_dot_q_dot_(i + 3) = jac_dot_q_dot.rot(i);
  }
  for (unsigned int i = 0; i < n; i++)
  {
    gravity_(i)  = gravity_torque_(i);
    coriolis_(i) = coriolis_torque_(i);
  }

  // M is symmetric positive definite for any configuration of a serial chain,
  // so this inverse is safe where the operational-space one is not.
  mass_inv_ = mass_.data.inverse();
  j_mass_inv_ = jacobian_.data * mass_inv_;
  inertia_inv_ = j_mass_inv_ * jacobian_.data.transpose();

  svd_jacobian_.compute(jacobian_.data);
  const double sigma_min = svd_jacobian_.singularValues()(
      svd_jacobian_.singularValues().size() - 1);

  // Pseudo-inverse of J M^-1 J^T with a relative tolerance. Relative because
  // that matrix mixes linear and angular units: even at a well conditioned
  // pose its singular values here span about three decades, so an absolute
  // threshold would either keep nothing or keep everything.
  svd_inertia_inv_.compute(inertia_inv_);
  const Eigen::VectorXd &sv = svd_inertia_inv_.singularValues();
  const double cut = pinv_rcond_ * sv(0);
  Eigen::VectorXd sv_inv = Eigen::VectorXd::Zero(sv.size());
  unsigned int rank = 0;
  for (int i = 0; i < sv.size(); i++)
    if (sv(i) > cut) { sv_inv(i) = 1.0/sv(i); rank++; }
  inertia_ = svd_inertia_inv_.matrixV() * sv_inv.asDiagonal() *
             svd_inertia_inv_.matrixU().transpose();

  const Eigen::VectorXd p  = inertia_ * (j_mass_inv_ * gravity_);
  const Eigen::VectorXd mu = inertia_ * (j_mass_inv_ * coriolis_) -
                             inertia_ * jac_dot_q_dot_;

  msg.header.stamp = time;
  for (unsigned int i = 0; i < n; i++)
  {
    msg.position[i] = this->joint_msr_.q(i);
    msg.velocity[i] = this->joint_msr_.qdot(i);
  }
  for (int r = 0; r < 6; r++)
  {
    for (int c = 0; c < 6; c++)
    {
      msg.inertia_inv[6*r + c] = inertia_inv_(r, c);
      msg.inertia[6*r + c]     = inertia_(r, c);
    }
    msg.coriolis[r] = mu(r);
    msg.gravity[r]  = p(r);
  }
  msg.jacobian_sigma_min = sigma_min;
  msg.inertia_rank = static_cast<unsigned char>(rank);
  msg.valid = (sigma_min > sigma_min_) && (rank == 6);

  if (!msg.valid)
    ROS_WARN_THROTTLE(10.0, "Cartesian dynamics controller: singular "
                            "configuration (smallest singular value of J "
                            "%.5f, rank of the operational-space inertia %u), "
                            "publishing it marked invalid", sigma_min, rank);
}

} // namespace controller_interface

PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianDynamicsController,
                       controller_interface::ControllerBase)
