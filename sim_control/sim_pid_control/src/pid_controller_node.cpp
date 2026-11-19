#include "ros/ros.h"
#include "PIDController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_controller_node");

  ros::NodeHandle nh;

  // Parameters
  std::string topic_arm_state;
  std::string topic_arm_command;

  std::vector<double> K_p;
  std::vector<double> K_i;
  std::vector<double> K_d;
  std::vector<double> N_p;
  std::vector<double> N_d;
  std::vector<double> A_inertia;
  std::vector<double> A_coriolis;
  std::vector<double> A_gravity;
  std::string topic_ee_dynamics;
  bool regulate_in_world;
  std::vector<double> workspace_limits;

  double arm_max_vel;
  double arm_acc_upper_limit;
  double arm_acc_lower_limit;
  double frequency;
  double integral_limit;

  /// LOADING PARAMETERS FROM THE ROS SERVER
  if (!nh.getParam("topic_arm_state", topic_arm_state))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the arm.");
    return -1;
  }
  if (!nh.getParam("topic_arm_command", topic_arm_command))
  {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
    return -1;
  }

  /// ADMITTANCE PARAMETERS
  if (!nh.getParam("K_p", K_p))
  {
    ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    return -1;
  }
  if (!nh.getParam("K_i", K_i))
  {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    return -1;
  }
  if (!nh.getParam("K_d", K_d))
  {
    ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling.");
    return -1;
  }

  if (!nh.getParam("N_p", N_p))
  {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    return -1;
  }
  if (!nh.getParam("N_d", N_d))
  {
    ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling.");
    return -1;
  }

  /// MODEL FEEDFORWARD (optional, zero means the model is not used, which is
  /// the behaviour of a parameter file written before this existed)
  const std::vector<double> zero36(36, 0.0);
  nh.param<std::vector<double> >("A_inertia", A_inertia, zero36);
  nh.param<std::vector<double> >("A_coriolis", A_coriolis, zero36);
  nh.param<std::vector<double> >("A_gravity", A_gravity, zero36);
  nh.param<std::string>("topic_ee_dynamics", topic_ee_dynamics,
                        "/sim_cartesian_dynamics_controller/ee_dynamics");

  /// WHICH FRAME THE TARGET IS FIXED IN. Optional, false keeps the behaviour
  /// of a parameter file written before this existed.
  nh.param<bool>("regulate_in_world", regulate_in_world, false);
  if (A_inertia.size() != 36 || A_coriolis.size() != 36 || A_gravity.size() != 36)
  {
    ROS_ERROR("A_inertia, A_coriolis and A_gravity must each have 36 elements, "
              "got %lu, %lu, %lu.",
              A_inertia.size(), A_coriolis.size(), A_gravity.size());
    return -1;
  }

  /// SAFETY PARAMETERS
  if (!nh.getParam("workspace_limits", workspace_limits))
  {
    ROS_ERROR("Couldn't retrieve the limits of the workspace.");
    return -1;
  }
  if (!nh.getParam("arm_max_vel", arm_max_vel))
  {
    ROS_ERROR("Couldn't retrieve the max velocity for the arm.");
    return -1;
  }
  if (!nh.getParam("arm_acc_upper_limit", arm_acc_upper_limit))
  {
    ROS_ERROR("Couldn't retrieve the max uppper acceleration for the arm.");
    return -1;
  }
  if (!nh.getParam("arm_acc_lower_limit", arm_acc_lower_limit))
  {
    ROS_ERROR("Couldn't retrieve the max lower acceleration for the arm.");
    return -1;
  }
  if (!nh.getParam("frequency", frequency))
  {
    ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
    return -1;
  }
  // Anti-windup bound on the integral term, in m/s and rad/s. Optional, so that
  // a parameter file written before the integral term was wired up still loads.
  nh.param<double>("integral_limit", integral_limit, 0.05);

  if (frequency <= 0.0)
  {
    ROS_ERROR("frequency must be positive, got %f.", frequency);
    return -1;
  }
  ROS_INFO("pid_controller: loop %.1f Hz, integral limit %.4f", frequency, integral_limit);

  // Constructing the controller
  PIDController pid_controller(
      nh,
      frequency,
      topic_arm_command,
      topic_arm_state,
      K_p, K_i, K_d, N_p, N_d,
      A_inertia, A_coriolis, A_gravity,
      regulate_in_world,
      topic_ee_dynamics,
      workspace_limits,
      arm_max_vel,
      arm_acc_upper_limit,
      arm_acc_lower_limit,
      integral_limit);

  // Running the controller
  pid_controller.run();

  return 0;
}