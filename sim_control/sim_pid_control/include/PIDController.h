#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include "std_msgs/Float32.h"

#include "std_srvs/Trigger.h"

#include "sim_cartesian_velocity_control/OperationalSpaceDynamics.h"

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 3, 3> Matrix3d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 3, 6> Matrix3_6d;
typedef Matrix<double, 6, 3> Matrix6_3d;

class PIDController
{
protected:
  ////// ROS VARIABLES:
  // A handle to the node in ros
  ros::NodeHandle nh_;
  // Rate of the run loop
  ros::Rate loop_rate_;

  ///// Subscribers:
  // Subscriber for the arm state
  ros::Subscriber sub_arm_state_;
  ros::Subscriber sub_base_state_;
  ros::Subscriber sub_ee_ground_truth_;
  ros::Subscriber sub_now_equilibrium_;
  ros::Subscriber sub_presuppose_;
  ros::Subscriber sub_ee_dynamics_;
  ////// Publishers:
  // Publisher for the twist of arm endeffector
  ros::Publisher pub_arm_cmd_;
  ros::Publisher pub_feedback_;
  /////// ADMITTANCE PARAMETERS:
  // M_a_ -> Desired mass of arm
  // D_a_ -> Desired damping of arm
  // K_ -> Desired Stiffness of the coupling
  Matrix6d K_p_, K_i_, K_d_;
  Matrix6d N_p_, N_d_;
  // Admittances through which the three model wrenches are turned into a
  // velocity contribution, in (m/s)/N and (rad/s)/(N m). Zero disables the
  // term, which is the default - see compute_pid().
  Matrix6d A_inertia_, A_coriolis_, A_gravity_;
  // Accumulated pose error for the integral term, and the anti-windup bound on
  // the term it produces (in m/s and rad/s, i.e. the unit of the output).
  Vector6d error_integral_;
  double integral_limit_;
  // equilibrium position of the coupling spring
  Vector3d equilibrium_position_;
  // equilibrium orientation of the coupling spring
  Quaterniond equilibrium_orientation_;

  // The same equilibrium carried into the world frame at the moment it was
  // received, which is what regulate_in_world_ regulates against. See
  // latch_equilibrium_in_world() and compute_pid().
  bool regulate_in_world_;
  Vector3d equilibrium_world_position_;
  Quaterniond equilibrium_world_orientation_;
  bool equilibrium_world_valid_;
  // True once /base_pose_ground_truth has been heard. Without it there is no
  // base pose to carry anything into world with.
  bool have_base_state_;

  // MANIPULATOR DYNAMICS, from sim_cartesian_dynamics_controller
  Matrix6d op_inertia_;      // Lambda
  Vector6d op_coriolis_;     // mu
  Vector6d op_gravity_;      // p
  bool dynamics_valid_;      // the publisher's own validity flag
  ros::Time last_dynamics_time_;
  bool model_feedforward_enabled_;  // any of the three admittances is nonzero

  // INPUT MODEL
  Vector6d model_affect_ee_twist_;
  Vector6d last_model_effect_ee_twist_;
  Vector6d model_affect_ee_acc_;
  Vector6d model_real_;

  // OUTPUT COMMANDS
  // final arm desired velocity
  Vector6d arm_desired_twist_final_;
  Vector6d last_arm_desired_twist_final_;

  Vector6d last_error_;

  // Vector6d total_error_;
  // Vector6d last_arm_real_twist_;

  // limiting the workspace of the arm
  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_acc_upper_limit_;
  double arm_acc_lower_limit_;
  double frequency;
  double eps_;

  ////// TIMING
  // Every derivative and integral in here uses measured elapsed time, not the
  // nominal loop period: the control loop and the three input streams run at
  // different rates (loop 250 Hz, ee_state 125 Hz, base ground truth 50 Hz), so
  // loop_rate_.expectedCycleTime() is not the age of any of the signals.
  ros::Time last_control_time_;
  ros::Time last_base_state_time_;
  ros::Time last_arm_state_time_;
  ros::Publisher pub_sample_age_;
  // Longest gap still treated as continuous; a larger one means a stalled or
  // restarted publisher, and the derivative is dropped instead of spiking.
  double max_input_gap_;

  ////// STATE VARIABLES:
  // Arm state: position, orientation, and twist (in "ur5_arm_base_link")
  Vector3d arm_real_position_;
  Quaterniond arm_real_orientation_;
  Vector6d arm_real_twist_;
  Vector6d ee_map_real_twist_;

  Vector3d base_real_position_;
  Quaterniond base_real_orientation_;
  Vector6d base_real_twist_;

  ////// LOWPASS_FILTER
  Vector7d x_0, x_1, x_2, x_3, x_4, x_5, x_6;
  Vector7d y_0, y_1, y_2, y_3, y_4, y_5, y_6;
  Vector7d mid_x, mid_y;

  Vector3d cx_0, cx_1, cx_2, cx_3;
  Vector3d cy_0, cy_1, cy_2, cy_3;
  Vector3d cmid_y;

  // Transform from base_link to world
  Matrix6d rotation_base_;
  bool pid_flag_;
  // TF:
  // Listeners
  tf::TransformListener listener_ft_;
  tf::TransformListener listener_control_;
  tf::TransformListener listener_arm_;

  // 判断是否所有坐标系矩阵建立
  bool transformation;
  bool pid_ready_flag_;

  // Initialization
  void wait_for_transformations();

  // Control
  void compute_pid(double dt);

  // Callbacks
  void state_arm_callback(const nav_msgs::OdometryConstPtr msg);
  void base_state_callback(const nav_msgs::OdometryConstPtr msg);
  void ee_ground_truth_state_callback(const nav_msgs::OdometryConstPtr msg);
  void presuppose_callback(const geometry_msgs::TwistConstPtr msg);
  void ee_dynamics_callback(
      const sim_cartesian_velocity_control::OperationalSpaceDynamicsConstPtr msg);
  /** \brief Carry the current equilibrium into the world frame and keep it */
  void latch_equilibrium_in_world();
  /** \brief Whether the last dynamics message can be used this cycle */
  bool dynamics_usable() const;
  void now_equilibrium_callback(const geometry_msgs::PoseStampedConstPtr msg);
  // Util
  bool get_rotation_matrix(Matrix6d &rotation_matrix, tf::TransformListener &listener, std::string from_frame, std::string to_frame);

  void limit_to_workspace();
  void kalman_filter();
  void lowpass_filter();
  void send_commands_to_robot();

public:
  PIDController(ros::NodeHandle &n, double frequency,
                std::string cmd_topic_arm,
                std::string state_topic_arm,
                std::vector<double> K_p,
                std::vector<double> K_i,
                std::vector<double> K_d,
                std::vector<double> N_p,
                std::vector<double> N_d,
                std::vector<double> A_inertia,
                std::vector<double> A_coriolis,
                std::vector<double> A_gravity,
                bool regulate_in_world,
                std::string topic_ee_dynamics,
                std::vector<double> workspace_limits,
                double arm_max_vel,
                double arm_acc_upper_limit,
                double arm_acc_lower_limit,
                double integral_limit);
  void run();
};

#endif // PIDCONTROLLER_H
