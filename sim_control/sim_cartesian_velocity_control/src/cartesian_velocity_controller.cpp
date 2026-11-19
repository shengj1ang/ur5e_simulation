#include <pluginlib/class_list_macros.h>
#include "sim_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include "sim_cartesian_velocity_control/cartesian_velocity_controller.h"
#include "kdl_conversions/kdl_msg.h"

namespace controller_interface
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
template<typename T>
bool CartesianVelocityControllerBase<T>::init(T *robot, ros::NodeHandle &n) 
{
  // KDL
  KinematicChainControllerBase<T>::init(robot, n);
  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));

  // get publishing period
  if (!n.getParam("publish_rate", publish_rate_))
  {
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
  }
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(n, "ee_state", 4));

  // Topics
  sub_command_ = n.subscribe("command_cart_vel", 5, &CartesianVelocityControllerBase<T>::command_cart_vel,
                             this,ros::TransportHints().reliable().tcpNoDelay());

  // Variable init
  this->joint_msr_.resize(this->kdl_chain_.getNrOfJoints());
  q_dt_cmd_.resize(this->kdl_chain_.getNrOfJoints());
  q_cmd_.resize(this->kdl_chain_.getNrOfJoints());
  x_dt_des_ = KDL::Twist::Zero();
  x_.p.Zero();
  x_.M.Identity();
  x_dot_.p.Zero();
  x_dot_.M.Identity();

  return true;
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
  x_dt_des_ = KDL::Twist::Zero();
  last_publish_time_ = time;
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
template<typename T>
void CartesianVelocityControllerBase<T>::update(const ros::Time& time, const ros::Duration& period) 
{
  // Get joint positions
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    this->joint_msr_.q(i)         = this->joint_handles_[i].getPosition();
    this->joint_msr_.qdot(i)      = this->joint_handles_[i].getVelocity();
  }

  // Compute inverse kinematics velocity solver
  ik_vel_solver_->CartToJnt(this->joint_msr_.q, x_dt_des_, q_dt_cmd_);
  writeVelocityCommands(period);

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
void CartesianVelocityController::writeVelocityCommands(const ros::Duration& period) 
{
    for(std::size_t i=0; i < this->joint_handles_.size(); i++) 
    {
      this->joint_handles_[i].setCommand(q_dt_cmd_(i));
    }
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
 * zero command means hold. The position interface in Gazebo tracks a position
 * setpoint exactly, so the two cannot drift apart here; on hardware, or with
 * gazebo_ros_control PID gains configured, a blocked joint would make the
 * commanded position run away from the real one, and that case wants a limit on
 * how far q_cmd_ may lead the measurement.
 */
void CartesianVelocityControllerSim::writeVelocityCommands(const ros::Duration& period)
{
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    q_cmd_(i) += q_dt_cmd_(i)*period.toSec();
    this->joint_handles_[i].setCommand(q_cmd_(i));
  }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianVelocityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianVelocityControllerSim, controller_interface::ControllerBase)
