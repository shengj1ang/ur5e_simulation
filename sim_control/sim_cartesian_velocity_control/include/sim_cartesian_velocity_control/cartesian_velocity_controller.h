#ifndef CARTESIAN_VELOCITY_CONTROLLER_H
#define CARTESIAN_VELOCITY_CONTROLLER_H

#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "sim_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include <realtime_tools/realtime_publisher.h>

namespace controller_interface
{

/** \brief This class implements a ROS control cartesian velocity
 * controller. Its base class implements the core
 * of the controller.
 */

template<typename T>
class CartesianVelocityControllerBase:
  public KinematicChainControllerBase<T>
{
public:
  CartesianVelocityControllerBase() {}
  ~CartesianVelocityControllerBase() {}

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *s
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(T *robot, ros::NodeHandle &n);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);

  /*!
   * \brief Issues commands to the joint. Called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /*!
   * \brief Subscriber's callback function
   */
  void command_cart_vel(const geometry_msgs::TwistConstPtr &msg);

  /** \brief Write current commands to the hardware interface
   */
  virtual void writeVelocityCommands(const ros::Duration& period) = 0;

  /** \brief Build the dynamics model and find the joint stiffness it needs */
  void setupFeedforward(ros::NodeHandle &n);

  /** \brief Work out how far ahead of the reference the position command has
   * to sit for the joint servos to land on it, given what the model says they
   * are carrying
   */
  void computeFeedforwardOffset();

  /** \brief Scale q_dt_cmd_ down uniformly when its norm exceeds
   * max_joint_velocity_, so that the direction of the motion is preserved.
   */
  void limitJointVelocities();

protected:
  ros::Subscriber sub_command_; // Interface to external commands

  KDL::Twist x_dt_des_;      // Desired end-effector velocity
  KDL::JntArray q_dt_cmd_;      // Desired joint velocity
  // Commanded joint position, integrated from q_dt_cmd_ by the simulated
  // instantiation below. It is deliberately a state of the controller and not
  // re-read from the joint handles every cycle - see the comment on
  // CartesianVelocityControllerSim::writeVelocityCommands.
  KDL::JntArray q_cmd_;

  boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;

  // Damped least squares parameters of the IK solver, plus a hard cap on the
  // joint velocity it returns. See the comment on init() in the .cpp for the
  // measurements the defaults come from.
  double ik_sigma_min_;         // singular value below which damping starts
  double ik_lambda_;            // damping factor at the singularity
  double max_joint_velocity_;   // rad/s, on the norm of q_dt_cmd_; <=0 disables
  // How far the integrated position command may get ahead of the measured
  // position, in rad; <=0 disables the limit. See writeVelocityCommands().
  double max_command_lead_;

  // Model-based position feedforward - see computeFeedforwardOffset()
  KDL::Chain dyn_chain_;                      // kdl_chain_ plus the tip payload
  boost::shared_ptr<KDL::ChainDynParam> dyn_param_;
  KDL::JntArray gravity_torque_;              // G(q)
  KDL::JntArray coriolis_torque_;             // C(q,qdot) qdot
  KDL::JntArray dq_ff_;                       // the offset, in rad
  std::vector<double> joint_stiffness_;       // Nm/rad, per joint
  double ff_gravity_;                         // weight on G(q)
  double ff_coriolis_;                        // weight on C(q,qdot) qdot
  bool ff_enabled_;                           // false without a stiffness

  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > realtime_pub_;

  ros::Time last_publish_time_;
  double publish_rate_;

  KDL::FrameVel x_dot_;
  KDL::Frame x_;
  nav_msgs::Odometry msg_state_;

  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
};

/** \brief Two different instantiations of the CartesianVelocityControllerBase
 * are provided here:
 * - CartesianVelocityController: with a VelocityJointInterface
 * - CartesianVelocityControllerSim: with a PositionJointInterface
 *   (for gazebo)
 */

/** The controller instance for the real hardware
 * */
class CartesianVelocityController :
  public CartesianVelocityControllerBase<hardware_interface::VelocityJointInterface> 
{
    /**
     * Write velocity commands through a velocity interface
     */
    void writeVelocityCommands(const ros::Duration& period);
};

/**
 * The controller instance for simulationd in gazebo. As of now
 * gazebo does not provide a VelocityJointInterface and therefore we simulate
 * it here through a PositionJointInterface.
*/
class CartesianVelocityControllerSim :
  public CartesianVelocityControllerBase<hardware_interface::PositionJointInterface> 
{
  /**
   * Write velocity commands through a position interface
   */
  void writeVelocityCommands(const ros::Duration& period);
};

} // namespace controller_interface


#endif
