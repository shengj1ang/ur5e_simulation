#ifndef CARTESIAN_DYNAMICS_CONTROLLER_H
#define CARTESIAN_DYNAMICS_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>

#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include "sim_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include "sim_cartesian_velocity_control/OperationalSpaceDynamics.h"

namespace controller_interface
{

/** \brief Publishes the operational-space dynamics of the arm
 *
 * This computes, at the current configuration,
 *
 *   Lambda = (J M^-1 J^T)^-1        operational-space inertia
 *   mu     = Lambda (J M^-1 C qdot - Jdot qdot)
 *   p      = Lambda J M^-1 G
 *
 * from the joint-space quantities KDL's ChainDynParam derives from the URDF,
 * and publishes them so that a controller can use them as a feedforward. It
 * only reads the joint states, so it claims no joint for writing and runs
 * alongside whichever controller is actually driving the arm.
 *
 * It does not close any loop itself. Turning these numbers into a command is
 * the job of the node that subscribes.
 */
class CartesianDynamicsController :
  public KinematicChainControllerBase<hardware_interface::JointStateInterface>
{
public:
  CartesianDynamicsController() {}
  ~CartesianDynamicsController() {}

  bool init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

private:
  /** \brief Compute the operational-space quantities and fill the message */
  void computeDynamics(const ros::Time& time);

  // The chain the dynamics are computed on. This is kdl_chain_ with everything
  // rigidly attached beyond the tip folded into the tip segment - see
  // accumulateDistalInertia() in the .cpp for why that is not optional.
  KDL::Chain dyn_chain_;

  boost::shared_ptr<KDL::ChainDynParam> dyn_param_;
  boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  boost::shared_ptr<KDL::ChainJntToJacDotSolver> jac_dot_solver_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<
      sim_cartesian_velocity_control::OperationalSpaceDynamics> > realtime_pub_;

  ros::Time last_publish_time_;
  double publish_rate_;

  // Configuration below which the Jacobian is called singular and the message
  // is marked invalid. Same meaning as ik_sigma_min in the velocity controller.
  double sigma_min_;
  // Relative tolerance of the pseudo-inverse of J M^-1 J^T: a singular value
  // smaller than rcond times the largest one is treated as zero. Relative,
  // because that matrix mixes linear and angular units and its condition
  // number is in the thousands even at a well conditioned pose.
  double pinv_rcond_;
  bool include_tip_payload_;

  // KDL scratch, sized once in init()
  KDL::JntSpaceInertiaMatrix mass_;
  KDL::JntArray gravity_torque_;
  KDL::JntArray coriolis_torque_;
  KDL::Jacobian jacobian_;

  // Eigen scratch, sized once in init()
  Eigen::MatrixXd mass_inv_;
  Eigen::MatrixXd inertia_inv_;
  Eigen::MatrixXd inertia_;
  Eigen::MatrixXd j_mass_inv_;
  Eigen::VectorXd coriolis_;
  Eigen::VectorXd gravity_;
  Eigen::VectorXd jac_dot_q_dot_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_jacobian_;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_inertia_inv_;
};

} // namespace controller_interface

#endif
