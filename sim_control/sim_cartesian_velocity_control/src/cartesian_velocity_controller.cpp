#include <pluginlib/class_list_macros.h>
#include "sim_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include "sim_cartesian_velocity_control/cartesian_velocity_controller.h"
#include "sim_cartesian_velocity_control/distal_inertia.h"
#include "kdl_conversions/kdl_msg.h"
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace controller_interface
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 * The inverse velocity kinematics is solved by damped least squares
 * (ChainIkSolverVel_wdls), not by the undamped pseudo-inverse this used to
 * use (ChainIkSolverVel_pinv_givens). Undamped, a command given near a
 * singularity is inverted through a vanishing singular value and comes back as
 * an arbitrarily large joint velocity.
 *
 * That is not a corner case here: the arm always comes up fully extended, which
 * is the elbow singularity, and nothing in the bringup can place it anywhere
 * else - the spawner's -J arguments and /gazebo/set_model_configuration are
 * both overwritten by the hardware interface (see the README's known issues).
 * Measured on this chain at that pose, for a 0.02 m/s command along x, the
 * Jacobian's singular values are
 *   2.105  1.565  0.644  0.532  0.0725  0.0000
 * and the undamped solver returns a joint velocity of norm 1.84 rad/s - a gain
 * of 92 per second from task space to joint space. In Gazebo that lands on a
 * position interface as roughly a 12 mm jump per cycle, whose reaction flung
 * the unactuated gantry (y_joint 1.82 m and r_joint 15.6 rad over a five second
 * command, with ee_state reporting speeds up to 105 m/s) and dragged the end
 * effector 0.76 m in the direction opposite to the one commanded.
 *
 * KDL's wdls damps the smallest singular values, and its 'eps' constructor
 * argument - ik_sigma_min here - is the singular value at which it starts.
 * KDL's own default of 1e-5 is far below anything this chain reaches in
 * practice, so damping would never engage; the value has to be chosen for the
 * robot. Measured, same command, norm of the returned joint velocity, and the
 * twist it actually achieves:
 *
 *   pose                     sigma_min   undamped   sigma_min=0.05   achieved
 *   fully extended (spawn)     0.0000     1.84        0.27           0.0200 (exact)
 *   near-singular elbow 0.05   0.0083     0.068       0.035          0.0200 (exact)
 *   near-singular elbow 0.20   0.0329     0.067       0.035          0.0198
 *   srdf 'init' pose           0.2140     0.0666      0.0666         0.0200 (exact)
 *
 * So 0.05 sits above everything the singular poses produce and below the
 * smallest singular value of a well conditioned pose: it removes the
 * seven-fold blow-up at the spawn pose without changing the solution at a
 * working pose at all. Raising it to 0.3 starts costing tracking everywhere
 * (0.0200 -> 0.0180 at the 'init' pose), which is why it is not higher.
 *
 * ik_lambda is the damping applied once below that threshold. It only has a
 * visible effect when the threshold is set high enough to be active at a well
 * conditioned pose, so with the default ik_sigma_min it mostly does not
 * matter; it is a parameter so that the trade-off can be made from a launch
 * file rather than from this source file.
 *
 * max_joint_velocity is a separate, unconditional backstop: whatever the
 * solver returns is scaled down uniformly if its norm exceeds it. It is the
 * arm's own limit from the robot description, 3.14 rad/s, because a backstop
 * set below the hardware's limit stops being a backstop and becomes the
 * controller's bandwidth.
 *
 * It was 1.0, and that was measured doing two different things. At a
 * singularity it does nothing: from the spawn pose the solver asks for 0.35
 * rad/s at its worst, a third of even that cap, and a command five times
 * harsher (0.10 m/s) still only gets 6 % through, because the damping
 * saturates the solution first - identically at 1.0 and at 3.14, with the free
 * gantry moving 0.02 mm either way. Tracking a base disturbance it was the
 * binding constraint: holding the end effector still against a moving base
 * costs about 4 rad/s of joint velocity per m/s at the 'init' pose, so at 3 Hz
 * the correction asked for 1.46 rad/s, was scaled by 0.69, and the clipping
 * cost 14.6 points of disturbance rejection (55.0 % to 69.6 %) and ten times
 * the residual's waveform distortion. At 5 Hz it asked for 2.71 rad/s and the
 * clipping cost 33.5 points.
 *
 * Verified in Gazebo, same 0.02 m/s command along x. From the spawn pose the
 * end effector now tracks at 0.0200 m/s while the gantry joints stay under
 * 1 mm and 0.001 rad, until the arm runs out of reach and the solution
 * degrades smoothly. From the SRDF 'init' pose, reached by driving the arm
 * there with arm_joint_controller first, damped and undamped are the same to
 * the measurement: both 0.0200 m/s with dy and dz of 0.0000 m.
 */
template<typename T>
bool CartesianVelocityControllerBase<T>::init(T *robot, ros::NodeHandle &n) 
{
  // KDL
  if (!KinematicChainControllerBase<T>::init(robot, n))
    return false;

  n.param("ik_sigma_min", ik_sigma_min_, 0.05);
  n.param("ik_lambda", ik_lambda_, 0.2);
  n.param("max_joint_velocity", max_joint_velocity_, 3.14);
  n.param("max_command_lead", max_command_lead_, 0.1);
  n.param("ff_gravity", ff_gravity_, 1.0);
  n.param("ff_coriolis", ff_coriolis_, 1.0);

  // 'eps' of the wdls solver is the singular value threshold, ik_sigma_min.
  KDL::ChainIkSolverVel_wdls *wdls =
      new KDL::ChainIkSolverVel_wdls(this->kdl_chain_, ik_sigma_min_);
  wdls->setLambda(ik_lambda_);
  ik_vel_solver_.reset(wdls);
  ROS_INFO("Cartesian velocity controller: damped IK, sigma_min %.4f, lambda "
           "%.4f, joint velocity capped at %.3f rad/s, command lead limited to "
           "%.3f rad",
           ik_sigma_min_, ik_lambda_, max_joint_velocity_, max_command_lead_);
  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));

  // get publishing period
  if (!n.getParam("publish_rate", publish_rate_))
  {
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
  }
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(n, "ee_state", 4));

  setupFeedforward(n);

  // Topics
  sub_command_ = n.subscribe("command_cart_vel", 5, &CartesianVelocityControllerBase<T>::command_cart_vel,
                             this,ros::TransportHints().reliable().tcpNoDelay());

  // Variable init (joint_msr_ is sized by the base class)
  q_dt_cmd_.resize(this->kdl_chain_.getNrOfJoints());
  q_cmd_.resize(this->kdl_chain_.getNrOfJoints());
  x_dt_des_ = KDL::Twist::Zero();
  x_.p.Zero();
  x_.M.Identity();
  x_dot_.p.Zero();
  x_dot_.M.Identity();

  return true;
}

/** \brief Build the dynamics model and find the joint stiffness it needs
 *
 * See computeFeedforwardOffset() for what this is for. The stiffness is the
 * only part that is not in the robot description: it is a property of whatever
 * closes the joint loop. Under Gazebo that is gazebo_ros_control, so the
 * default is to read its own gains - the same numbers the plant is using, which
 * is exactly what makes the compensation match. Set joint_stiffness explicitly
 * for anything else.
 *
 * With no stiffness found, the feedforward switches itself off rather than
 * guessing. That is the right answer for the kinematic plant: with no
 * /gazebo_ros_control/pid_gains the joint goes exactly where it is told and
 * there is no deflection to compensate.
 */
template<typename T>
void CartesianVelocityControllerBase<T>::setupFeedforward(ros::NodeHandle &n)
{
  const unsigned int n_joints = this->kdl_chain_.getNrOfJoints();
  gravity_torque_.resize(n_joints);
  coriolis_torque_.resize(n_joints);
  dq_ff_.resize(n_joints);
  KDL::SetToZero(dq_ff_);
  ff_enabled_ = false;
  dyn_chain_ = this->kdl_chain_;

  std::string tip_name;
  this->nh_.getParam(this->nh_.getNamespace() + "/tip_name", tip_name);

  std::string robot_description, xml;
  urdf::Model model;
  if (ros::param::search(this->nh_.getNamespace(), "robot_description", robot_description) &&
      this->nh_.getParam(robot_description, xml) && model.initString(xml))
  {
    double payload_mass = 0.0;
    KDL::Vector payload_com;
    dyn_chain_ = chainWithTipPayload(this->kdl_chain_, model, tip_name,
                                     &payload_mass, &payload_com);
    if (payload_mass > 0.0)
      ROS_INFO("Cartesian velocity controller: %.4f kg attached beyond %s is in "
               "the feedforward model, centre of mass (%.4f %.4f %.4f)",
               payload_mass, tip_name.c_str(),
               payload_com.x(), payload_com.y(), payload_com.z());
  }
  else
  {
    ROS_WARN("Cartesian velocity controller: could not re-read the robot "
             "description, the feedforward model will not include anything "
             "attached beyond %s", tip_name.c_str());
  }

  // Gravity in the root frame. See the dynamics controller for why -z is right
  // on this model.
  std::vector<double> gravity_param;
  KDL::Vector gravity(0.0, 0.0, -9.81);
  if (n.getParam("gravity", gravity_param) && gravity_param.size() == 3)
    gravity = KDL::Vector(gravity_param[0], gravity_param[1], gravity_param[2]);
  dyn_param_.reset(new KDL::ChainDynParam(dyn_chain_, gravity));

  // Joint stiffness: explicit, else gazebo_ros_control's own p gains.
  joint_stiffness_.assign(n_joints, 0.0);
  std::vector<double> explicit_stiffness;
  if (n.getParam("joint_stiffness", explicit_stiffness))
  {
    if (explicit_stiffness.size() != n_joints)
    {
      ROS_ERROR("joint_stiffness has %lu entries, the chain has %u joints; "
                "the model feedforward stays off",
                explicit_stiffness.size(), n_joints);
      return;
    }
    joint_stiffness_ = explicit_stiffness;
  }
  else
  {
    for (std::size_t i = 0; i < this->joint_handles_.size(); i++)
    {
      const std::string key = "/gazebo_ros_control/pid_gains/" +
                              this->joint_handles_[i].getName() + "/p";
      double p = 0.0;
      if (ros::param::get(key, p))
        joint_stiffness_[i] = p;
    }
  }

  std::size_t have = 0;
  for (std::size_t i = 0; i < joint_stiffness_.size(); i++)
    if (joint_stiffness_[i] > 0.0) have++;

  if (have == 0)
  {
    ROS_INFO("Cartesian velocity controller: no joint stiffness found, so no "
             "model feedforward. That is correct when the joints are positioned "
             "kinematically - there is no deflection to compensate.");
    return;
  }
  if (have != n_joints)
    ROS_WARN("Cartesian velocity controller: a joint stiffness is missing for "
             "%u of %u joints; those joints get no feedforward",
             n_joints - (unsigned int)have, n_joints);

  ff_enabled_ = (ff_gravity_ != 0.0) || (ff_coriolis_ != 0.0);
  ROS_INFO("Cartesian velocity controller: model position feedforward %s "
           "(gravity weight %.3f, coriolis weight %.3f)",
           ff_enabled_ ? "on" : "off (both weights are zero)",
           ff_gravity_, ff_coriolis_);
}

/** \brief Work out how far ahead of the reference the position command has to
 * sit for the joint servos to land on it
 *
 * The joint loop is a spring: gazebo_ros_control's per-joint PID produces a
 * torque p*(q_cmd - q), so whatever torque the joint has to hold shows up as a
 * position error of that torque over p. With no integral gain - and there is
 * none, deliberately, see gazebo_joint_pid_gains.yaml - that error does not go
 * away. Measured at the SRDF 'init' pose, the arm sits 29 mm below where it was
 * told to be.
 *
 * The dynamics say what that torque is: M qddot + C qdot + G. So adding
 * (G + C qdot)/p to the position command cancels the part of the deflection
 * that gravity and the arm's own motion cause, and the joint lands on the
 * reference instead of below it. The simulation controller handles inertia
 * separately when a stamped base acceleration is available.
 *
 * The sign and the size were checked against measurements taken before this
 * existed. At the SRDF 'init' pose the model gives G of -18.848 Nm at
 * shoulder_lift and -18.815 Nm at elbow; with p of 2925 and 940 that predicts
 * deflections of -6.4 mrad and -20.0 mrad, and the arm was measured sitting
 * 6.5 mrad and 20.0 mrad past its command at those two joints.
 *
 * This is a position offset, so it only means anything on a position
 * interface. It is applied in CartesianVelocityControllerSim and ignored by the
 * velocity-interface instantiation, where the joint loop is not ours to
 * compensate.
 *
 * Note what it does not do: it carries no information about an external force,
 * so it does not stiffen the arm against being pushed. It cancels the
 * deflection from loads the model knows about, which is gravity above all.
 */
template<typename T>
void CartesianVelocityControllerBase<T>::computeFeedforwardOffset()
{
  if (!ff_enabled_)
    return;

  dyn_param_->JntToGravity(this->joint_msr_.q, gravity_torque_);
  dyn_param_->JntToCoriolis(this->joint_msr_.q, this->joint_msr_.qdot,
                            coriolis_torque_);

  for (std::size_t i = 0; i < this->joint_handles_.size(); i++)
  {
    if (joint_stiffness_[i] <= 0.0)
    {
      dq_ff_(i) = 0.0;
      continue;
    }
    const double torque = ff_gravity_ * gravity_torque_(i) +
                          ff_coriolis_ * coriolis_torque_(i);
    dq_ff_(i) = torque / joint_stiffness_[i];
  }
}

/** \brief This is called from within the realtime thread just before the
 * first call to \ref update
 *
 * \param time The current time
 */
template<typename T>
void CartesianVelocityControllerBase<T>::starting(const ros::Time& time)
{
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    q_dt_cmd_(i) = 0.0;
    // Start the commanded position at the current one, so that switching this
    // controller in does not move the arm.
    q_cmd_(i) = this->joint_handles_[i].getPosition();
  }
  // The joints may have moved since this controller last ran, so do not
  // differentiate across the gap.
  this->resetJointStateEstimate();
  x_dt_des_ = KDL::Twist::Zero();
  last_publish_time_ = time;
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
template<typename T>
void CartesianVelocityControllerBase<T>::update(const ros::Time& time, const ros::Duration& period) 
{
  // Get joint positions, and the velocity differentiated from them. The
  // velocity the hardware interface reports is not usable under Gazebo - see
  // readJointStates() in kinematic_chain_controller_base.h.
  this->readJointStates(period);

  // Compute inverse kinematics velocity solver. A positive return value is
  // E_CONVERGE_PINV_SINGULAR: the solution is usable but damped, i.e. the arm
  // is at or near a singularity. That used to happen silently.
  int ik_ret = ik_vel_solver_->CartToJnt(this->joint_msr_.q, x_dt_des_, q_dt_cmd_);
  if (ik_ret < 0)
  {
    ROS_WARN_THROTTLE(5.0, "Cartesian velocity controller: IK solver failed "
                           "(%d), commanding zero joint velocity", ik_ret);
    KDL::SetToZero(q_dt_cmd_);
  }
  else if (ik_ret > 0)
  {
    ROS_WARN_THROTTLE(10.0, "Cartesian velocity controller: at or near a "
                            "singularity, the IK solution is damped and the "
                            "end effector will not follow the command exactly");
  }

  const bool velocity_limited = limitJointVelocities();
  // The acceleration reference assumes the arm can execute the tracking motion.
  reference_unconstrained_ = ik_ret == 0 && !velocity_limited;
  computeFeedforwardOffset();
  writeVelocityCommands(time, period);

  // Forward kinematics
  fk_vel_solver_->JntToCart(this->joint_msr_, x_dot_);
  fk_pos_solver_->JntToCart(this->joint_msr_.q, x_);

  // Limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time) 
  {
    // try to publish
    if (realtime_pub_->trylock()) 
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

      // populate message
      realtime_pub_->msg_.header.stamp = time;
      tf::poseKDLToMsg(x_, realtime_pub_->msg_.pose.pose);
      tf::twistKDLToMsg(x_dot_.GetTwist(), realtime_pub_->msg_.twist.twist);

      realtime_pub_->unlockAndPublish();
    }
  }
}

/** \brief Scale the commanded joint velocity down uniformly if its norm
 * exceeds max_joint_velocity_
 *
 * Scaling the whole vector keeps the direction of the motion, so the end
 * effector still travels along the commanded path, only slower. Zeroing or
 * clamping single joints instead would send it somewhere else entirely.
 */
template<typename T>
bool CartesianVelocityControllerBase<T>::limitJointVelocities()
{
  if (max_joint_velocity_ <= 0.0)   // disabled
    return false;

  double norm = 0.0;
  for (unsigned int i = 0; i < q_dt_cmd_.rows(); i++)
    norm += q_dt_cmd_(i) * q_dt_cmd_(i);
  norm = std::sqrt(norm);

  if (norm > max_joint_velocity_)
  {
    const double scale = max_joint_velocity_ / norm;
    for (unsigned int i = 0; i < q_dt_cmd_.rows(); i++)
      q_dt_cmd_(i) *= scale;
    // Still enforce the exact cap, but tolerate roundoff and marginal scaling
    // when deciding whether the acceleration reference is usable.
    if (scale >= 0.999)
      return false;
    ROS_WARN_THROTTLE(5.0, "Cartesian velocity controller: joint velocity "
                           "%.3f rad/s over the %.3f rad/s cap, scaled by %.3f",
                      norm, max_joint_velocity_, scale);
    return true;
  }
  return false;
}

/*!
 * \brief Subscriber's callback: copies twist commands
 */
template<typename T>
void CartesianVelocityControllerBase<T>::command_cart_vel(const geometry_msgs::TwistConstPtr &msg) 
{
    x_dt_des_.vel(0) = msg->linear.x;
    x_dt_des_.vel(1) = msg->linear.y;
    x_dt_des_.vel(2) = msg->linear.z;
    x_dt_des_.rot(0) = msg->angular.x;
    x_dt_des_.rot(1) = msg->angular.y;
    x_dt_des_.rot(2) = msg->angular.z;
}


/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

/** \brief write the desired velocity command in the hardware interface input
 * for a VelocityJointInterface
 * \param period The duration of an update cycle
 */
void CartesianVelocityController::writeVelocityCommands(const ros::Time&, const ros::Duration&)
{
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) 
    {
      this->joint_handles_[i].setCommand(q_dt_cmd_(i));
    }
}

bool CartesianVelocityControllerSim::init(hardware_interface::PositionJointInterface* robot,
                                         ros::NodeHandle& n)
{
  if (!CartesianVelocityControllerBase::init(robot, n))
    return false;

  bool enable_inertia;
  n.param("inertia_feedforward", enable_inertia, false);
  if (!enable_inertia)
    return true;

  if (!loadInertiaWeights(n))
    return false;

  std::string topic;
  n.param<std::string>("base_acceleration_topic", topic, "/gantry/base_linear_acceleration");
  if (topic.empty())
  {
    ROS_WARN("Inertia feedforward requires a base_acceleration_topic");
    return true;
  }

  // Match the actual Gazebo servo, even when the static compensation uses
  // a separate stiffness setting. The inverse below is for PD with zero I.
  for (const auto& joint : joint_handles_)
  {
    const std::string gains = "/gazebo_ros_control/pid_gains/" + joint.getName();
    double p = 0.0, d = 0.0, i = 0.0;
    ros::param::get(gains + "/p", p);
    ros::param::get(gains + "/d", d);
    ros::param::get(gains + "/i", i);
    if (!std::isfinite(p) || p <= 0.0 || !std::isfinite(d) || d < 0.0 || i != 0.0)
    {
      ROS_WARN("Inertia feedforward requires Gazebo PD gains with zero I for %s",
               joint.getName().c_str());
      return true;
    }
    joint_proportional_gain_.push_back(p);
    joint_damping_.push_back(d);
  }

  const unsigned int joints = kdl_chain_.getNrOfJoints();
  inertial_offset_.resize(joints);
  reference_acceleration_.resize(joints);
  projected_offset_.resize(joints);
  translation_torque_.resize(joints);
  inertial_torque_.resize(joints);
  arm_inertia_.resize(joints);
  jacobian_dot_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
  inertia_jacobian_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  inertia_jacobian_.resize(joints);
  for (unsigned int axis = 0; axis < unit_acceleration_models_.size(); ++axis)
  {
    KDL::Vector apparent_gravity = KDL::Vector::Zero();
    apparent_gravity(axis) = -1.0;
    unit_acceleration_models_[axis].reset(new KDL::ChainDynParam(dyn_chain_, apparent_gravity));
  }

  n.getParam("root_name", root_name_);
  torque_clip_count_.assign(joints, 0);
  inertia_state_pub_.reset(new realtime_tools::RealtimePublisher<
      sim_cartesian_velocity_control::InertialFeedforwardState>(n, "inertial_feedforward_state", 4));
  auto& state = inertia_state_pub_->msg_;
  state.header.frame_id = root_name_;
  for (const auto& joint : joint_handles_)
    state.joint_names.push_back(joint.getName());
  state.inertial_offset.resize(state.joint_names.size());
  state.inertial_torque.resize(state.joint_names.size());
  state.commanded_torque.resize(state.joint_names.size());
  state.torque_clip_count.resize(state.joint_names.size());

  base_acceleration_subscriber_ = n.subscribe(
      topic, 1, &CartesianVelocityControllerSim::baseAccelerationCallback,
      this, ros::TransportHints().tcpNoDelay());
  n.param("inertia_mapping_experiment", inertia_mapping_experiment_, false);
  if (inertia_mapping_experiment_)
  {
    static_mapping_subscriber_ = n.subscribe(
        "use_static_inertia_mapping", 1, &CartesianVelocityControllerSim::staticInertiaMappingCallback,
        this, ros::TransportHints().tcpNoDelay());
    experiment_state_pub_.reset(new realtime_tools::RealtimePublisher<
        sim_cartesian_velocity_control::InertiaExperimentState>(n, "inertia_experiment_state", 4));
    auto& experiment = experiment_state_pub_->msg_;
    experiment.header.frame_id = root_name_;
    experiment.raw_torque.resize(joints);
    experiment.mapped_torque.resize(joints);
    experiment.requested_torque.resize(joints);
    experiment_subscriber_ = n.subscribe(
        "inertia_experiment_setting", 1, &CartesianVelocityControllerSim::inertiaExperimentCallback,
        this, ros::TransportHints().tcpNoDelay());
  }

  inertia_feedforward_enabled_ = true;
  ROS_INFO("Cartesian velocity controller: translational inertia feedforward listens on %s "
           "(weights %.3g %.3g %.3g, projected on %d%d%d)",
           topic.c_str(), inertia_feedforward_gain_[0], inertia_feedforward_gain_[1],
           inertia_feedforward_gain_[2], inertia_project_translation_[0],
           inertia_project_translation_[1], inertia_project_translation_[2]);
  return true;
}

bool CartesianVelocityControllerSim::loadInertiaWeights(ros::NodeHandle& n)
{
  const auto numeric = [](const XmlRpc::XmlRpcValue& value) {
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ||
           value.getType() == XmlRpc::XmlRpcValue::TypeDouble;
  };
  const auto number = [](XmlRpc::XmlRpcValue& value) {
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? double(int(value)) : double(value);
  };

  // One weight per base_link translation axis, or a single number for all
  // three. The weights are identified in closed loop, one base axis at a time;
  // a zero leaves that direction to the feedback and the kinematic feedforward.
  XmlRpc::XmlRpcValue gain;
  if (n.getParam("inertia_feedforward_gain", gain))
  {
    if (numeric(gain))
      inertia_feedforward_gain_.fill(number(gain));
    else if (gain.getType() == XmlRpc::XmlRpcValue::TypeArray && gain.size() == 3 &&
             numeric(gain[0]) && numeric(gain[1]) && numeric(gain[2]))
    {
      for (int i = 0; i < 3; ++i)
        inertia_feedforward_gain_[i] = number(gain[i]);
    }
    else
    {
      ROS_ERROR("inertia_feedforward_gain takes one number per axis, or one for all three");
      return false;
    }
  }
  if (!std::all_of(inertia_feedforward_gain_.begin(), inertia_feedforward_gain_.end(),
                   [](double value) { return std::isfinite(value); }))
  {
    ROS_ERROR("inertia_feedforward_gain must be finite");
    return false;
  }

  // Where the single-axis domain ends, as the squared cosine between the base
  // acceleration and its largest component: 0.64 is 37 degrees. Below it the
  // correction is off, above it it fades in. Measured at init with the X weight
  // against a Y cross axis, the deployed weight still beat leaving it at zero
  // by 8.6 % of the three-dimensional RMS at 26.6 degrees and cost 5.3 % at
  // 45 degrees, so the crossing interpolates to 37. Zero never shuts the
  // correction off, since the largest component of a vector carries at least a
  // third of its square.
  n.param("inertia_alignment_threshold", inertia_alignment_threshold_, 0.64);
  if (!std::isfinite(inertia_alignment_threshold_) || inertia_alignment_threshold_ < 0.0 ||
      inertia_alignment_threshold_ >= 1.0)
  {
    ROS_ERROR("inertia_alignment_threshold must be in [0, 1)");
    return false;
  }

  // The axes whose correction keeps only the along-disturbance translation,
  // or a bool for all three.
  XmlRpc::XmlRpcValue project;
  if (n.getParam("inertia_project_translation", project))
  {
    if (project.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      inertia_project_translation_.fill(bool(project));
    else if (project.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      inertia_project_translation_.fill(false);
      for (int i = 0; i < project.size(); ++i)
      {
        const std::string axis = project[i].getType() == XmlRpc::XmlRpcValue::TypeString ?
                                 std::string(project[i]) : std::string();
        if (axis.size() != 1 || axis[0] < 'x' || axis[0] > 'z')
        {
          ROS_ERROR("inertia_project_translation takes the axis names x, y and z");
          return false;
        }
        inertia_project_translation_[axis[0] - 'x'] = true;
      }
    }
    else
    {
      ROS_ERROR("inertia_project_translation takes a list of axes, or a bool for all three");
      return false;
    }
  }
  return true;
}

CartesianVelocityControllerSim::InertiaCompensation
CartesianVelocityControllerSim::inertiaCompensation(const KDL::Vector& acceleration) const
{
  const double magnitude_sq = KDL::dot(acceleration, acceleration);
  if (magnitude_sq < 1e-12)
    return {0.0, false};
  unsigned int axis = 0;
  for (unsigned int i = 1; i < 3; ++i)
    if (std::abs(acceleration(i)) > std::abs(acceleration(axis)))
      axis = i;
  const double alignment = acceleration(axis) * acceleration(axis) / magnitude_sq;
  if (alignment <= inertia_alignment_threshold_)
    return {0.0, false};

  // The weights were identified along one base axis at a time, so fade the
  // correction out as the disturbance leaves that domain. A pure single-axis
  // disturbance leaves the factor at one and the weight as configured.
  const double weight = (alignment - inertia_alignment_threshold_) /
                        (1.0 - inertia_alignment_threshold_);
  return {weight * inertia_feedforward_gain_[axis], inertia_project_translation_[axis]};
}

void CartesianVelocityControllerSim::starting(const ros::Time& time)
{
  CartesianVelocityControllerBase::starting(time);
  started_at_ = time;
  if (inertia_feedforward_enabled_)
  {
    KDL::SetToZero(inertial_offset_);
    std::fill(torque_clip_count_.begin(), torque_clip_count_.end(), 0);
    last_inertia_state_time_ = time;
  }
}

void CartesianVelocityControllerSim::baseAccelerationCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  const auto& a = msg->vector;
  if (msg->header.frame_id != root_name_ || msg->header.stamp.isZero() ||
      !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z))
    return;

  BaseAcceleration sample;
  sample.stamp = msg->header.stamp;
  sample.linear = KDL::Vector(a.x, a.y, a.z);
  base_acceleration_.writeFromNonRT(sample);
}

void CartesianVelocityControllerSim::staticInertiaMappingCallback(const std_msgs::BoolConstPtr& msg)
{
  static_mapping_requested_.writeFromNonRT(msg->data);
}

void CartesianVelocityControllerSim::inertiaExperimentCallback(
    const sim_cartesian_velocity_control::InertiaExperimentSettingConstPtr& msg)
{
  if (msg->trial_id != 0 && std::isfinite(msg->gain))
    experiment_requested_.writeFromNonRT(*msg);
}

bool CartesianVelocityControllerSim::computeInertialTorque(const KDL::Vector& base_acceleration,
    bool project_translation, sim_cartesian_velocity_control::InertiaExperimentState* experiment)
{
  // For a world-fixed tip and a translating root, J*qdd + Jdot*qdot = -a_base.
  const KDL::Twist task_acceleration(-base_acceleration, KDL::Vector::Zero());
  KDL::Twist jacobian_bias;
  if (jacobian_dot_->JntToJacDot(joint_msr_, jacobian_bias) != 0 ||
      ik_vel_solver_->CartToJnt(joint_msr_.q, task_acceleration - jacobian_bias,
                              reference_acceleration_) != 0 ||
      dyn_param_->JntToMass(joint_msr_.q, arm_inertia_) != 0)
    return false;

  inertial_torque_.data.noalias() = arm_inertia_.data * reference_acceleration_.data;
  // In a translating root, the same uniform inertial load as -a_base acts
  // on every link. KDL gravity torques are linear in that acceleration.
  for (unsigned int axis = 0; axis < unit_acceleration_models_.size(); ++axis)
  {
    if (base_acceleration(axis) == 0.0)
      continue;
    if (unit_acceleration_models_[axis]->JntToGravity(joint_msr_.q, translation_torque_) != 0)
      return false;
    inertial_torque_.data += base_acceleration(axis) * translation_torque_.data;
  }
  if (experiment)
    std::copy_n(inertial_torque_.data.data(), inertial_torque_.rows(), experiment->raw_torque.begin());

  if (project_translation || experiment)
  {
    const double length = base_acceleration.Norm();
    if (length < 1e-6 || inertia_jacobian_solver_->JntToJac(joint_msr_.q, inertia_jacobian_) != 0)
      return !project_translation && inertial_torque_.data.allFinite();
    const KDL::Vector direction = base_acceleration / length;
    KDL::Vector displacement = KDL::Vector::Zero();
    for (std::size_t i = 0; i < joint_proportional_gain_.size(); ++i)
      displacement += inertia_jacobian_.getColumn(i).vel * (inertial_torque_(i) / joint_proportional_gain_[i]);
    const double axis_offset = KDL::dot(direction, displacement);
    if (experiment)
      experiment->raw_axis_offset = experiment->mapped_axis_offset = axis_offset;
    if (!project_translation)
      return inertial_torque_.data.allFinite();

    // Project the servo's equivalent position offset along the disturbance.
    // Keep the nominal orientation and cross-axis position out of this feedforward.
    const KDL::Twist projected(direction * axis_offset, KDL::Vector::Zero());
    if (ik_vel_solver_->CartToJnt(joint_msr_.q, projected, projected_offset_) != 0)
      return false;
    for (std::size_t i = 0; i < joint_proportional_gain_.size(); ++i)
      inertial_torque_(i) = joint_proportional_gain_[i] * projected_offset_(i);
    if (experiment)
    {
      experiment->mapped_axis_offset = 0.0;
      for (std::size_t i = 0; i < joint_proportional_gain_.size(); ++i)
        experiment->mapped_axis_offset += KDL::dot(direction, inertia_jacobian_.getColumn(i).vel) * projected_offset_(i);
    }
  }
  return inertial_torque_.data.allFinite();
}

/** \brief write the desired velocity command in the hardware interface input
 * for a PosititionJointInterface
 * \param period The duration of an update cycle
 *
 * Gazebo offers no VelocityJointInterface, so a velocity command has to be
 * turned into a position command. The command is the integral of the commanded
 * joint velocity, held as controller state.
 *
 * It used to be joint_msr_.q(i) + q_dt_cmd_(i)*period, i.e. the measured
 * position plus one step. That looks equivalent and is not: it rebuilds the
 * setpoint from the measurement every cycle, so any position error the plant
 * has accumulated - gravity, above all, since nothing in this path compensates
 * it - is adopted as the new setpoint instead of being corrected. The
 * controller then had no position feedback at all, and a zero velocity command
 * meant "keep falling at whatever rate you are falling", not "hold still".
 *
 * Measured in Gazebo with this controller running and a zero command: the arm
 * sagged continuously, shoulder_lift at 0.0195 rad/s, which the end effector
 * felt as a drift of about (-0.017, 0, -0.020) m/s - the same order as the
 * 0.02 m/s commands the Cartesian controllers send, so commanded motion was
 * buried in it. With the controller stopped (the hardware interface then holds
 * the last written position command, a fixed setpoint) every arm joint held to
 * 0.0000 rad over 4.7 s, which is what isolated the cause to this line.
 *
 * Integrating the command keeps the setpoint independent of the measurement, so
 * zero command means hold.
 *
 * Nothing then stops the two from drifting apart, which is what
 * max_command_lead is for. A joint that cannot follow - blocked, saturated
 * against its effort limit, or simply lagging - leaves the integral climbing
 * while the joint stands still, and when the obstruction clears the joint jumps
 * to wherever the integral has got to. Clamping q_cmd_ to within
 * max_command_lead of the measurement is the position-loop equivalent of
 * anti-windup: the command waits for the joint instead of running away from it.
 *
 * This mattered less when gazebo_ros_control had no pid_gains for these joints,
 * because then Gazebo satisfied the position command kinematically and the two
 * could not diverge; with the gains in
 * sim_bringup/config/gazebo_joint_pid_gains.yaml the joint is a real servo with
 * a torque limit, and it can fall behind.
 */
void CartesianVelocityControllerSim::writeVelocityCommands(const ros::Time& time, const ros::Duration& period)
{
  const double dt = period.toSec();
  const auto* input = inertia_feedforward_enabled_ ? base_acceleration_.readFromRT() : nullptr;
  const bool fresh_input = input && !input->stamp.isZero() && input->stamp >= started_at_ &&
                           input->stamp <= time && (time - input->stamp).toSec() <= 0.05;
  // Switch between trials, after the last sample has expired and the offset
  // has decayed. Switching during a disturbance would kick the servo's D term.
  if (inertia_mapping_experiment_ && !fresh_input && inertial_offset_.data.cwiseAbs().maxCoeff() < 1e-5)
  {
    static_inertia_mapping_ = *static_mapping_requested_.readFromRT();
    experiment_setting_ = *experiment_requested_.readFromRT();
  }

  const bool experimental = experiment_setting_.trial_id != 0;
  auto compensation = InertiaCompensation{0.0, false};
  if (fresh_input)
    compensation = experimental ? InertiaCompensation{experiment_setting_.gain, experiment_setting_.project_translation} :
                                  inertiaCompensation(input->linear);

  const bool publish_diagnostics = inertia_state_pub_ && publish_rate_ > 0.0 &&
                                  (time - last_inertia_state_time_).toSec() >= 1.0 / publish_rate_;
  sim_cartesian_velocity_control::InertiaExperimentState* experiment = nullptr;
  if (experimental && publish_diagnostics && experiment_state_pub_->trylock())
  {
    experiment = &experiment_state_pub_->msg_;
    experiment->header.stamp = time;
    experiment->setting = experiment_setting_;
    experiment->sigma_min = ik_vel_solver_->getSigmaMin();
    experiment->raw_axis_offset = experiment->mapped_axis_offset = 0.0;
  }

  const bool model_valid = fresh_input && dt > 0.0 && reference_unconstrained_ &&
                           (compensation.gain != 0.0 || experiment) &&
                           computeInertialTorque(input->linear, compensation.project_translation, experiment);
  const bool compensate_inertia = model_valid && compensation.gain != 0.0;
  if (experiment)
  {
    experiment->model_valid = model_valid;
    if (!model_valid)
      experiment->raw_axis_offset = experiment->mapped_axis_offset = 0.0;
  }

  sim_cartesian_velocity_control::InertialFeedforwardState* state = nullptr;
  if (publish_diagnostics && inertia_state_pub_->trylock())
  {
    state = &inertia_state_pub_->msg_;
    state->header.stamp = time;
    state->active = compensate_inertia;
    state->static_mapping = static_inertia_mapping_;
  }
  if (state || experiment)
    last_inertia_state_time_ = time;
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    q_cmd_(i) += q_dt_cmd_(i)*dt;
    bool lead_limited = false;

    if (max_command_lead_ > 0.0)
    {
      const double measured = this->joint_handles_[i].getPosition();
      if (q_cmd_(i) > measured + max_command_lead_)
      {
        q_cmd_(i) = measured + max_command_lead_;
        lead_limited = true;
        ROS_WARN_THROTTLE(5.0, "Cartesian velocity controller: %s cannot follow, "
                               "holding the command %.4f rad ahead of it",
                          this->joint_handles_[i].getName().c_str(),
                          max_command_lead_);
      }
      else if (q_cmd_(i) < measured - max_command_lead_)
      {
        q_cmd_(i) = measured - max_command_lead_;
        lead_limited = true;
        ROS_WARN_THROTTLE(5.0, "Cartesian velocity controller: %s cannot follow, "
                               "holding the command %.4f rad behind it",
                          this->joint_handles_[i].getName().c_str(),
                          max_command_lead_);
      }
    }

    // The offset is added at the output only, never accumulated into q_cmd_:
    // q_cmd_ is the reference the joint should land on, and the offset is the
    // lead it takes to get there. Keeping them apart is also what lets the
    // anti-windup above compare the reference with the measurement, which is
    // the comparison that means something.
    double command = q_cmd_(i) + dq_ff_(i);
    if (inertia_feedforward_enabled_)
    {
      const double p = joint_proportional_gain_[i];
      const double d = joint_damping_[i];
      // Both mappings bound the position offset to 25 mrad.
      const double torque_limit = p * 0.025;
      const double model_torque = compensate_inertia ? inertial_torque_(i) : 0.0;
      const double requested_torque = lead_limited ? 0.0 : compensation.gain * model_torque;
      const double torque = std::max(-torque_limit, std::min(torque_limit, requested_torque));
      if (torque != requested_torque)
      {
        ++torque_clip_count_[i];
        ROS_WARN_THROTTLE(5.0, "Inertial feedforward torque clipped on %s: requested %.3f N*m, "
                              "bounded to %.3f N*m (limit %.3f N*m)",
                          joint_handles_[i].getName().c_str(), requested_torque, torque, torque_limit);
      }
      double& offset = inertial_offset_(i);
      if (static_inertia_mapping_ && compensate_inertia && !lead_limited)
        offset = torque / p;
      // Backward Euler inverse of P + D*s. Use it for inactive decay in
      // both modes; dropping a nonzero offset would kick the servo's D term.
      else if (dt > 0.0)
        offset += dt * (torque - p * offset) / (d + p * dt);
      command += offset;
      if (state)
      {
        state->inertial_offset[i] = offset;
        state->inertial_torque[i] = model_torque;
        state->commanded_torque[i] = torque;
        state->torque_clip_count[i] = torque_clip_count_[i];
      }
      if (experiment)
      {
        if (!model_valid)
          experiment->raw_torque[i] = 0.0;
        experiment->mapped_torque[i] = model_valid ? inertial_torque_(i) : 0.0;
        experiment->requested_torque[i] = requested_torque;
      }
    }
    this->joint_handles_[i].setCommand(command);
  }
  if (state)
    inertia_state_pub_->unlockAndPublish();
  if (experiment)
    experiment_state_pub_->unlockAndPublish();
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianVelocityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianVelocityControllerSim, controller_interface::ControllerBase)
