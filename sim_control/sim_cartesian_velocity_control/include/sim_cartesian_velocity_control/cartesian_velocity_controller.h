#ifndef CARTESIAN_VELOCITY_CONTROLLER_H
#define CARTESIAN_VELOCITY_CONTROLLER_H

#include <array>
#include <cstdint>
#include <memory>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "sim_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include "sim_cartesian_velocity_control/InertialFeedforwardState.h"
#include "sim_cartesian_velocity_control/InertiaExperimentState.h"

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
  virtual void writeVelocityCommands(const ros::Time& time, const ros::Duration& period) = 0;

  /** \brief Build the dynamics model and find the joint stiffness it needs */
  void setupFeedforward(ros::NodeHandle &n);

  /** \brief Work out how far ahead of the reference the position command has
   * to sit for the joint servos to land on it, given what the model says they
   * are carrying
   */
  void computeFeedforwardOffset();

  /** \brief Enforce the velocity cap and report a material reduction in scale.
   */
  bool limitJointVelocities();

protected:
  ros::Subscriber sub_command_; // Interface to external commands

  KDL::Twist x_dt_des_;      // Desired end-effector velocity
  KDL::JntArray q_dt_cmd_;      // Desired joint velocity
  // Commanded joint position, integrated from q_dt_cmd_ by the simulated
  // instantiation below. It is deliberately a state of the controller and not
  // re-read from the joint handles every cycle - see the comment on
  // CartesianVelocityControllerSim::writeVelocityCommands.
  KDL::JntArray q_cmd_;
  bool reference_unconstrained_ = false;

  boost::shared_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;

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
    void writeVelocityCommands(const ros::Time& time, const ros::Duration& period) override;
};

/**
 * The controller instance for simulationd in gazebo. As of now
 * gazebo does not provide a VelocityJointInterface and therefore we simulate
 * it here through a PositionJointInterface.
*/
class CartesianVelocityControllerSim :
  public CartesianVelocityControllerBase<hardware_interface::PositionJointInterface> 
{
public:
  bool init(hardware_interface::PositionJointInterface* robot, ros::NodeHandle& n) override;
  void starting(const ros::Time& time) override;

private:
  struct InertiaCompensation
  {
    double gain;
    bool project_translation;
  };

  struct InertiaCalibration
  {
    std::vector<double> joints;
    std::vector<double> gains;
    std::array<bool, 3> project_translation{{false, false, false}};
  };

  bool loadInertiaCalibration(ros::NodeHandle& n);
  InertiaCompensation inertiaCompensation(const KDL::Vector& acceleration) const;
  void baseAccelerationCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  void staticInertiaMappingCallback(const std_msgs::BoolConstPtr& msg);
  void inertiaExperimentCallback(const sim_cartesian_velocity_control::InertiaExperimentSettingConstPtr& msg);
  bool computeInertialTorque(const KDL::Vector& base_acceleration, bool project_translation,
                            sim_cartesian_velocity_control::InertiaExperimentState* experiment);

  /**
   * Write velocity commands through a position interface
   */
  void writeVelocityCommands(const ros::Time& time, const ros::Duration& period) override;

  struct BaseAcceleration
  {
    ros::Time stamp;
    KDL::Vector linear = KDL::Vector::Zero();
  };
  realtime_tools::RealtimeBuffer<BaseAcceleration> base_acceleration_;
  realtime_tools::RealtimeBuffer<bool> static_mapping_requested_{false};
  realtime_tools::RealtimeBuffer<sim_cartesian_velocity_control::InertiaExperimentSetting> experiment_requested_;
  sim_cartesian_velocity_control::InertiaExperimentSetting experiment_setting_;
  bool inertia_mapping_experiment_ = false;
  bool static_inertia_mapping_ = false;
  ros::Time started_at_;
  std::string root_name_;
  bool inertia_feedforward_enabled_ = false;
  double inertia_feedforward_gain_ = 1.0;
  bool inertia_project_translation_ = false;
  double inertia_calibration_radius_ = 0.2;
  double inertia_calibration_fade_radius_ = 0.3;
  std::vector<InertiaCalibration> inertia_calibrations_;
  std::vector<double> joint_proportional_gain_;
  std::vector<double> joint_damping_;
  KDL::JntArray inertial_offset_;
  KDL::JntArray reference_acceleration_;
  KDL::JntArray projected_offset_;
  KDL::JntArray translation_torque_;
  KDL::JntArray inertial_torque_;
  KDL::JntSpaceInertiaMatrix arm_inertia_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver> jacobian_dot_;
  std::unique_ptr<KDL::ChainJntToJacSolver> inertia_jacobian_solver_;
  KDL::Jacobian inertia_jacobian_;
  std::array<std::unique_ptr<KDL::ChainDynParam>, 3> unit_acceleration_models_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      sim_cartesian_velocity_control::InertialFeedforwardState>> inertia_state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<
      sim_cartesian_velocity_control::InertiaExperimentState>> experiment_state_pub_;
  ros::Time last_inertia_state_time_;
  std::vector<std::uint32_t> torque_clip_count_;
  ros::Subscriber base_acceleration_subscriber_;
  ros::Subscriber static_mapping_subscriber_;
  ros::Subscriber experiment_subscriber_;
};

} // namespace controller_interface


#endif
