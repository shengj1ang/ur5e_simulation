#include "PIDController.h"
#include <geometry_msgs/Vector3Stamped.h>

PIDController::PIDController(ros::NodeHandle &n,
                             double frequency,
                             std::string topic_arm_command,
                             std::string topic_arm_state,
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
                             double integral_limit) : nh_(n), loop_rate_(frequency),
                                                           K_p_(K_p.data()), K_i_(K_i.data()), K_d_(K_d.data()), N_p_(N_p.data()), N_d_(N_d.data()),
                                                           A_inertia_(A_inertia.data()), A_coriolis_(A_coriolis.data()), A_gravity_(A_gravity.data()),
                                                           regulate_in_world_(regulate_in_world),
                                                           integral_limit_(integral_limit),
                                                           workspace_limits_(workspace_limits.data()),
                                                           arm_max_vel_(arm_max_vel),
                                                           arm_acc_upper_limit_(arm_acc_upper_limit),
                                                           arm_acc_lower_limit_(arm_acc_lower_limit)
{
  ///// Subscribers
  sub_arm_state_ = nh_.subscribe(topic_arm_state, 1,
                                 &PIDController::state_arm_callback, this,
                                 ros::TransportHints().reliable().tcpNoDelay());

  sub_ee_dynamics_ = nh_.subscribe(topic_ee_dynamics, 1,
                                   &PIDController::ee_dynamics_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

  sub_now_equilibrium_ = nh_.subscribe("/now_equilibrium", 1,
                                       &PIDController::now_equilibrium_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());

  sub_base_state_ = nh_.subscribe("/base_pose_ground_truth", 1,
                                  &PIDController::base_state_callback, this,
                                  ros::TransportHints().reliable().tcpNoDelay());

  sub_presuppose_ = nh_.subscribe("/presuppose", 1,
                                  &PIDController::presuppose_callback, this,
                                  ros::TransportHints().reliable().tcpNoDelay());

  sub_ee_ground_truth_ = nh_.subscribe("/ee_pose_ground_truth", 1,
                                       &PIDController::ee_ground_truth_state_callback, this,
                                       ros::TransportHints().reliable().tcpNoDelay());
  // Publishers
  pub_arm_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 1);
  pub_feedback_ = nh_.advertise<geometry_msgs::Twist>("/check", 1);
  pub_sample_age_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/pid_control/sample_age", 10);

  // Init integrator
  arm_desired_twist_final_.setZero();
  last_arm_desired_twist_final_.setZero();
  last_error_.setZero();
  error_integral_.setZero();
  // Eigen members are not zero-initialised. These all feed the control law
  // before their callbacks have fired for the first time (the feedforward pair
  // especially: /base_pose_ground_truth may be late or absent), so leaving them
  // uninitialised means commanding the arm from whatever was on the stack.
  arm_real_twist_.setZero();
  ee_map_real_twist_.setZero();
  model_affect_ee_twist_.setZero();
  last_model_effect_ee_twist_.setZero();
  model_affect_ee_acc_.setZero();
  model_real_.setZero();
  equilibrium_world_position_.setZero();
  equilibrium_world_orientation_.setIdentity();
  equilibrium_world_valid_ = false;
  have_base_state_ = false;
  if (regulate_in_world_)
  {
    ROS_INFO("pid_controller: regulating the end effector against a target "
             "fixed in world. The equilibrium arrives in base_link and is "
             "carried into world when it does, so the base moving is an error "
             "the feedback can see.");
  }
  else
  {
    ROS_INFO("pid_controller: regulating the end effector in base_link, so the "
             "base moving is invisible to the feedback and only the N_p/N_d "
             "feedforward stands against it");
  }

  op_inertia_.setZero();
  op_coriolis_.setZero();
  op_gravity_.setZero();
  dynamics_valid_ = false;
  last_dynamics_time_ = ros::Time(0);
  model_feedforward_enabled_ = (A_inertia_.norm() > 0.0) ||
                               (A_coriolis_.norm() > 0.0) ||
                               (A_gravity_.norm() > 0.0);
  if (model_feedforward_enabled_)
  {
    ROS_INFO("pid_controller: model feedforward on, admittance norms "
             "inertia %.4g, coriolis %.4g, gravity %.4g",
             A_inertia_.norm(), A_coriolis_.norm(), A_gravity_.norm());
  }
  else
  {
    ROS_INFO("pid_controller: model feedforward off (all three admittances are "
             "zero); %s is subscribed but not used", topic_ee_dynamics.c_str());
  }
  base_real_twist_.setZero();
  base_real_position_.setZero();
  base_real_orientation_.setIdentity();
  last_control_time_ = ros::Time(0);
  last_base_state_time_ = ros::Time(0);
  max_input_gap_ = 0.5;
  pid_flag_ = false;
  transformation = false;
  pid_ready_flag_ = false;
  eps_ = 0.001;
  // between zero and one that modulated the input force as to control the stiffness(刚度)

  wait_for_transformations();

  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  Matrix3d rotation_from_to, rotation_repair;
  bool flag = true;
  rotation_repair << -1., 0., 0., 0., -1., 0., 0., 0., 1.;

  while (flag)
  {
    ros::spinOnce();
    // ROS_INFO("ur5e_mode is: %d", ur5e_mode_);
    try
    {
      tf_listener.lookupTransform("base_link", "ee_link", ros::Time(0), transform);
      equilibrium_position_ << transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ();
      tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
      equilibrium_orientation_ = Quaterniond(rotation_repair * rotation_from_to);
      equilibrium_orientation_.coeffs() << equilibrium_orientation_.coeffs() / equilibrium_orientation_.coeffs().norm();
      flag = false;
    }
    catch (tf::TransformException ex)
    {
      flag = true;
      ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: "
                                      << "base_link"
                                      << " to: "
                                      << "ee_link");
      sleep(1);
    }
  }
  arm_real_orientation_ = equilibrium_orientation_;
  arm_real_position_ = equilibrium_position_;
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void PIDController::state_arm_callback(const nav_msgs::OdometryConstPtr msg)
{
  last_arm_state_time_ = msg->header.stamp;
  arm_real_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  arm_real_orientation_.coeffs() << msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w;

  // The Cartesian velocity controller publishes the measured end-effector twist
  // in the same message (nav_msgs/Odometry, so twist.twist - the commented-out
  // version of these lines read msg->twist.linear and could not compile). Using
  // it is what lets the damping term be a real velocity instead of a difference
  // of two consecutive pose errors.
  arm_real_twist_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

void PIDController::base_state_callback(const nav_msgs::OdometryConstPtr msg)
{
  const bool first = !have_base_state_;
  have_base_state_ = true;
  base_real_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;

  base_real_orientation_.coeffs() << msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w;

  base_real_twist_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
  // Rigid-body transport of the base motion to the end effector:
  //     v_ee = v_base + omega_base x r,     r = p_ee - p_base
  // r is the lever arm from the arm base to the end effector, so it is the
  // end-effector position expressed in base_link (which is what ee_state gives,
  // the chain being base_link -> ee_link) rotated into the frame v_base and
  // omega_base are expressed in - the world frame, for the p3d plugin.
  //
  // This used to add base_real_position_ as well, i.e. it crossed omega with the
  // end effector's *world* position instead of the lever arm. That adds a
  // spurious omega x p_base term, and p_base is metres away from the world
  // origin once the gantry has travelled (y was ~5 m here), so the term grew to
  // tens of m/s and swamped the whole command. The world position is still worth
  // printing, but it is not the lever arm.
  const Matrix3d rotation_base_world = base_real_orientation_.toRotationMatrix();
  const Vector3d lever_arm = rotation_base_world * arm_real_position_;
  const Vector3d arm_map_translation = lever_arm + base_real_position_;
  std::cout << "arm_map_translation" << arm_map_translation.transpose() << std::endl;
  Vector3d mid = base_real_twist_.bottomRows(3);
  model_affect_ee_twist_.bottomRows(3) = base_real_twist_.bottomRows(3);
  model_affect_ee_twist_.topRows(3) = base_real_twist_.topRows(3) + mid.cross(lever_arm);

  // This callback fires once per /base_pose_ground_truth message (50 Hz from the
  // p3d plugin), so the difference above spans one message interval - about
  // 20 ms - not one control period. Dividing it by expectedCycleTime() (1 ms at
  // the old frequency: 1000) inflated this feedforward acceleration by ~20x,
  // which is why N_d had to be tuned down to 1e-5 to stay usable. Use the stamps.
  const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  const double time_diff = (stamp - last_base_state_time_).toSec();
  if (last_base_state_time_.isZero() || time_diff <= 0.0 || time_diff > max_input_gap_)
  {
    // First message, a restarted clock, or a stalled publisher: no usable
    // interval, so report no acceleration rather than a spike.
    model_affect_ee_acc_.setZero();
  }
  else
  {
    model_affect_ee_acc_ = (model_affect_ee_twist_ - last_model_effect_ee_twist_) / time_diff;
  }
  last_base_state_time_ = stamp;
  last_model_effect_ee_twist_ = model_affect_ee_twist_;

  geometry_msgs::Twist model_affect_ee_twist_msgs;
  model_affect_ee_twist_msgs.linear.x = model_affect_ee_twist_(0);
  model_affect_ee_twist_msgs.linear.y = model_affect_ee_twist_(1);
  model_affect_ee_twist_msgs.linear.z = model_affect_ee_twist_(2);
  model_affect_ee_twist_msgs.angular.x = model_affect_ee_twist_(3);
  model_affect_ee_twist_msgs.angular.y = model_affect_ee_twist_(4);
  model_affect_ee_twist_msgs.angular.z = model_affect_ee_twist_(5);
  pub_feedback_.publish(model_affect_ee_twist_msgs);

  // An equilibrium that arrived before any base pose did could not be carried
  // into world at the time. Do it now that there is a base pose.
  if (first && regulate_in_world_ && !equilibrium_world_valid_)
    latch_equilibrium_in_world();

  // ROS_INFO("getting the model state");
  std::cout << "for feedforward: " << model_affect_ee_twist_.transpose() << std::endl;
}

/** \brief Carry the current equilibrium into the world frame and keep it
 *
 * The equilibrium arrives in base_link, which is the frame ee_state reports in
 * and the frame every caller of /now_equilibrium already speaks. Regulating
 * against it in that frame is what makes the base moving invisible to the
 * feedback: the base carries the arm along, the end effector keeps the same
 * base_link pose, and there is no error. Carrying it into world once, at the
 * moment it is received, turns the same request into "hold this point in the
 * room" without changing the interface.
 *
 * It is a latch rather than a running conversion on purpose. Converting every
 * cycle would track the base and give back exactly the base_link behaviour.
 */
void PIDController::latch_equilibrium_in_world()
{
  if (!have_base_state_)
  {
    ROS_WARN_THROTTLE(5.0, "pid_controller: cannot carry the equilibrium into "
                           "world yet, no base pose heard on "
                           "/base_pose_ground_truth");
    return;
  }
  const Matrix3d rotation_base_world = base_real_orientation_.toRotationMatrix();
  equilibrium_world_position_ =
      base_real_position_ + rotation_base_world * equilibrium_position_;
  equilibrium_world_orientation_ = base_real_orientation_ * equilibrium_orientation_;
  equilibrium_world_orientation_.normalize();
  equilibrium_world_valid_ = true;
  ROS_INFO("pid_controller: holding world (%.5f, %.5f, %.5f)",
           equilibrium_world_position_(0), equilibrium_world_position_(1),
           equilibrium_world_position_(2));
}

void PIDController::ee_ground_truth_state_callback(const nav_msgs::OdometryConstPtr msg)
{
  std::cout << "time: " << msg->header.stamp.toSec() << std::endl;
  std::cout << "ee real wrist: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << ", " << msg->twist.twist.linear.z << ", "
            << msg->twist.twist.angular.x << ", " << msg->twist.twist.angular.y << ", " << msg->twist.twist.angular.z << std::endl;
  std::cout << "----------------------------------------------------------------------------------------------------" << std::endl;
  ee_map_real_twist_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

void PIDController::presuppose_callback(const geometry_msgs::TwistConstPtr msg)
{
  // std::cout << "presuppose_twist: " << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z << ", "
  //  << msg->angular.x << ", " << msg->angular.y << ", " << msg->angular.z << std::endl;
  // std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
  model_real_ << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
}

void PIDController::ee_dynamics_callback(
    const sim_cartesian_velocity_control::OperationalSpaceDynamicsConstPtr msg)
{
  for (int r = 0; r < 6; r++)
  {
    for (int c = 0; c < 6; c++)
    {
      op_inertia_(r, c) = msg->inertia[6*r + c];
    }
    op_coriolis_(r) = msg->coriolis[r];
    op_gravity_(r)  = msg->gravity[r];
  }
  dynamics_valid_ = msg->valid;
  last_dynamics_time_ = msg->header.stamp.isZero() ? ros::Time::now()
                                                   : msg->header.stamp;
}

/** \brief Whether the last dynamics message can be used this cycle
 *
 * Invalid means the publisher said so - at or near a singularity, where the
 * operational-space inertia does not exist and what it sends instead is a
 * pseudo-inverse that looks reasonable and is not Lambda. This arm comes up at
 * exactly such a configuration, so this is the normal case at startup, not an
 * edge case.
 */
bool PIDController::dynamics_usable() const
{
  if (!dynamics_valid_ || last_dynamics_time_.isZero())
    return false;
  const double age = (ros::Time::now() - last_dynamics_time_).toSec();
  return (age >= 0.0) && (age <= max_input_gap_);
}

void PIDController::now_equilibrium_callback(const geometry_msgs::PoseStampedConstPtr msg)
{

  equilibrium_position_(0) = msg->pose.position.x;
  equilibrium_position_(1) = msg->pose.position.y;
  equilibrium_position_(2) = msg->pose.position.z;

  equilibrium_orientation_.coeffs() << msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w;

  if (regulate_in_world_)
    latch_equilibrium_in_world();

  if (msg->header.frame_id == "pid")
  {
    pid_ready_flag_ = true;
  }
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void PIDController::run()
{
  ROS_INFO("Running the pid control loop .................");
  while (nh_.ok())
  {
    if (pid_ready_flag_)
    {
      // Measured loop period. ros::Time follows /clock here (use_sim_time is
      // true in every bringup launch file), so this is simulated time and stays
      // consistent with the message stamps even when Gazebo runs slower than
      // real time.
      const ros::Time now = ros::Time::now();
      double dt = (now - last_control_time_).toSec();
      if (last_control_time_.isZero() || dt < 0.0 || dt > max_input_gap_)
      {
        // First iteration after becoming ready, a clock that went backwards
        // (Gazebo reset), or a gap long enough that the accumulated error says
        // nothing about the present: start the integrator over.
        dt = 0.0;
        error_integral_.setZero();
      }
      else if (dt == 0.0)
      {
        // Two iterations inside the same /clock tick. No time has passed, so
        // this cycle simply contributes nothing to the integral - it must not
        // reset it. Resetting here is what kept the integral term pinned near
        // zero: at 250 Hz against a 1 kHz clock this happens often enough that
        // the integrator never accumulated more than a fraction of a second.
        dt = 0.0;
      }
      last_control_time_ = now;

      // lowpass_filter();
      // Admittance Dynamics computation
      compute_pid(dt);

      // sum the vel from admittance to DS in this function
      // limit the the movement of the arm to the permitted workspace
      limit_to_workspace();

      // Copy commands to messages
      send_commands_to_robot();

      if (pub_sample_age_.getNumSubscribers() && !last_base_state_time_.isZero() &&
          !last_arm_state_time_.isZero())
      {
        // x: base sample age, y: arm sample age, z: PID period, all in seconds.
        geometry_msgs::Vector3Stamped sample_age;
        sample_age.header.stamp = now;
        sample_age.vector.x = (now - last_base_state_time_).toSec();
        sample_age.vector.y = (now - last_arm_state_time_).toSec();
        sample_age.vector.z = dt;
        pub_sample_age_.publish(sample_age);
      }

      // publishing visualization/debugging info
      //    publish_debuggings_signals();
    }
    else
    {
      // Not tracking yet: keep the integrator empty and the clock unset so the
      // first controlled cycle does not see a huge dt.
      last_control_time_ = ros::Time(0);
      error_integral_.setZero();
    }
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

///////////////////////////////////////////////////////////////
///////////////////// PID Dynamics /////////////////////
///////////////////////////////////////////////////////////////
void PIDController::compute_pid(double dt)
{
  Vector6d error;

  // The twist the damping term acts on. In base_link it is just what the
  // Cartesian controller measured; in world it is that plus the base's own
  // motion carried to the end effector, rotated back into base_link because
  // that is the frame the command goes out in.
  Vector6d damped_twist = arm_real_twist_;

  // Whether to regulate against the world-fixed target. It needs a base pose
  // to have been carried into world at least once; until then fall back to the
  // base_link error rather than command from an uninitialised target.
  const bool world = regulate_in_world_ && equilibrium_world_valid_;
  if (regulate_in_world_ && !world)
  {
    ROS_WARN_THROTTLE(5.0, "pid_controller: asked to regulate in world but the "
                           "equilibrium has not been carried there yet; "
                           "regulating in base_link this cycle");
  }

  if (world)
  {
    // Compose the end effector's world pose from the base pose and the arm's
    // own measurement. Deliberately not from /ee_pose_ground_truth: on a real
    // machine the gantry's position comes from its own encoders and the end
    // effector from the arm's forward kinematics, and neither is an external
    // ground truth of the tool.
    const Matrix3d rotation_base_world = base_real_orientation_.toRotationMatrix();
    const Vector3d ee_world_position =
        base_real_position_ + rotation_base_world * arm_real_position_;
    Quaterniond ee_world_orientation = base_real_orientation_ * arm_real_orientation_;
    ee_world_orientation.normalize();

    if (equilibrium_world_orientation_.coeffs().dot(ee_world_orientation.coeffs()) < 0.0)
      ee_world_orientation.coeffs() << -ee_world_orientation.coeffs();
    Eigen::Quaterniond quat_rot_err(
        ee_world_orientation * equilibrium_world_orientation_.inverse());
    if (quat_rot_err.coeffs().norm() > 1e-3)
      quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
    Eigen::AngleAxisd err_world_orient(quat_rot_err);

    // The error is formed in world and then rotated into base_link, which is
    // the frame the Cartesian velocity controller takes its twist in.
    error.topRows(3) =
        rotation_base_world.transpose() * (ee_world_position - equilibrium_world_position_);
    error.bottomRows(3) = rotation_base_world.transpose() *
        (err_world_orient.axis() * err_world_orient.angle());

    // Damp the world velocity, not the base_link one. model_affect_ee_twist_ is
    // the base's motion carried to the end effector, in world.
    damped_twist.topRows(3) += rotation_base_world.transpose() *
        model_affect_ee_twist_.topRows(3);
    damped_twist.bottomRows(3) += rotation_base_world.transpose() *
        model_affect_ee_twist_.bottomRows(3);
  }
  else
  {
    if (equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0)
    {
      arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
    }
    Eigen::Quaterniond quat_rot_err(arm_real_orientation_ * equilibrium_orientation_.inverse());
    if (quat_rot_err.coeffs().norm() > 1e-3)
    {
      // Normalize error quaternion
      quat_rot_err.coeffs() << quat_rot_err.coeffs() / quat_rot_err.coeffs().norm();
    }

    Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
    error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();
    error.topRows(3) = arm_real_position_ - equilibrium_position_;
  }

  // Integral term. Conditional integration: each component of the stored error
  // integral is clamped so that the term it produces cannot exceed
  // integral_limit_, which is in the unit of the output (m/s, rad/s). This
  // assumes a diagonal K_i, which is how pid_params.yaml defines it.
  error_integral_ += error * dt;
  for (int i = 0; i < 6; ++i)
  {
    const double k_i = K_i_(i, i);
    if (k_i > 0.0)
    {
      const double state_limit = integral_limit_ / k_i;
      if (error_integral_(i) > state_limit)
      {
        error_integral_(i) = state_limit;
      }
      else if (error_integral_(i) < -state_limit)
      {
        error_integral_(i) = -state_limit;
      }
    }
    else
    {
      error_integral_(i) = 0.0;
    }
  }

  // if (model_real_.norm() > 0.01)
  // {
  //   pid_flag_ = true;
  // }
  // if (pid_flag_)
  // {
  //   error.setZero();
  // }
  std::cout << "error: " << error.transpose() << std::endl;

  // Damping now acts on the measured end-effector twist. It used to act on
  // (error - last_error), an undivided difference of two pose errors sampled at
  // the loop rate: with the loop at 1000 Hz and ee_state arriving at 125 Hz that
  // difference was zero on seven cycles out of eight and a step on the eighth,
  // and it carried an implicit factor of dt, so K_d silently changed meaning
  // whenever the loop frequency changed.
  // Model-based feedforward, in the operational-space form
  //
  //     F = Lambda * a_ff + mu + p
  //
  // with a_ff the acceleration the arm has to produce at the end effector to
  // leave it where it is while its base moves, i.e. minus the base
  // acceleration carried to the end effector. The three wrenches are admitted
  // back to a velocity through A_inertia_, A_coriolis_ and A_gravity_, which
  // is what makes this an admittance rather than a force law: the arm is
  // commanded in velocity, so a wrench can only enter as the motion it would
  // produce through a chosen compliance.
  //
  // The terms are separate so that each can be switched on by itself, which is
  // what the comparison in the next step needs. All three default to zero, so
  // a parameter file written before this existed reproduces the old law
  // exactly.
  //
  // What this cannot do on the simulation as it stands: the joints are driven
  // through a position interface that Gazebo satisfies kinematically, so the
  // path from command to motion has no dynamics in it. Measured at the SRDF
  // init pose, 100 N applied at the end effector moves it by about 10 um and
  // it springs straight back, and with no external force the joints sit on the
  // commanded value to within 7e-7 rad. Lambda, mu and p are real quantities of
  // the modelled arm, but there is nothing in this plant for them to correct.
  // Giving the joints their own dynamics - gazebo_ros_control pid_gains, or an
  // effort interface - is what would make these terms do work.
  Vector6d model_feedforward_twist = Vector6d::Zero();
  if (model_feedforward_enabled_)
  {
    if (dynamics_usable())
    {
      const Vector6d a_ff = -model_affect_ee_acc_;
      model_feedforward_twist = A_inertia_ * (op_inertia_ * a_ff) +
                                A_coriolis_ * op_coriolis_ +
                                A_gravity_ * op_gravity_;
    }
    else
    {
      ROS_WARN_THROTTLE(5.0, "pid_controller: model feedforward is on but the "
                             "dynamics are not usable (valid %d, age %.3f s); "
                             "running on the feedback and kinematic "
                             "feedforward alone",
                        static_cast<int>(dynamics_valid_),
                        last_dynamics_time_.isZero() ? -1.0
                            : (ros::Time::now() - last_dynamics_time_).toSec());
    }
  }

  // With the world-fixed target the base's motion is already an error the
  // feedback acts on, so N_p and N_d are feeding forward against something that
  // is no longer invisible. They are left in the law - zero them in the
  // parameter file to see the feedback on its own - but they are redundant
  // there, and K_d now carries part of the same signal through damped_twist.
  arm_desired_twist_final_ = -K_p_ * error - K_i_ * error_integral_ - K_d_ * damped_twist - N_p_ * model_affect_ee_twist_ - N_d_ * model_affect_ee_acc_ + model_feedforward_twist;

  last_error_ = error;
  last_arm_desired_twist_final_ = arm_desired_twist_final_;
}

///////////////////////////////////////////////////////////////
//////////////////// Limit to workspace////////////////////////
///////////////////////////////////////////////////////////////
void PIDController::limit_to_workspace()
{
  if (arm_real_position_(0) < workspace_limits_(0) || arm_real_position_(0) > workspace_limits_(1))
  {
    ROS_WARN_STREAM_THROTTLE(1, "Out of permitted workspace.  x = "
                                    << arm_real_position_(0) << " not in [" << workspace_limits_(0) << " , "
                                    << workspace_limits_(1) << "]");
  }

  if (arm_real_position_(1) < workspace_limits_(2) || arm_real_position_(1) > workspace_limits_(3))
  {
    ROS_WARN_STREAM_THROTTLE(1, "Out of permitted workspace.  y = "
                                    << arm_real_position_(1) << " not in [" << workspace_limits_(2) << " , "
                                    << workspace_limits_(3) << "]");
  }

  if (arm_real_position_(2) < workspace_limits_(4) || arm_real_position_(2) > workspace_limits_(5))
  {
    ROS_WARN_STREAM_THROTTLE(1, "Out of permitted workspace.  z = "
                                    << arm_real_position_(2) << " not in [" << workspace_limits_(4) << " , "
                                    << workspace_limits_(5) << "]");
  }

  if (arm_desired_twist_final_(0) < 0 && arm_real_position_(0) < workspace_limits_(0))
  {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(0) > 0 && arm_real_position_(0) > workspace_limits_(1))
  {
    arm_desired_twist_final_(0) = 0;
  }

  if (arm_desired_twist_final_(1) < 0 && arm_real_position_(1) < workspace_limits_(2))
  {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(1) > 0 && arm_real_position_(1) > workspace_limits_(3))
  {
    arm_desired_twist_final_(1) = 0;
  }

  if (arm_desired_twist_final_(2) < 0 && arm_real_position_(2) < workspace_limits_(4))
  {
    arm_desired_twist_final_(2) = 0;
  }

  if (arm_desired_twist_final_(2) > 0 && arm_real_position_(2) > workspace_limits_(5))
  {
    arm_desired_twist_final_(2) = 0;
  }

  // velocity of the arm along x, y, and z axis
  double norm_vel_des = (arm_desired_twist_final_.segment(0, 3)).norm();

  if (norm_vel_des > arm_max_vel_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "pid generate fast arm movements! velocity norm: " << norm_vel_des);
    arm_desired_twist_final_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);
  }
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void PIDController::send_commands_to_robot()
{
  geometry_msgs::Twist arm_twist_cmd;
  arm_twist_cmd.linear.x = arm_desired_twist_final_(0);
  arm_twist_cmd.linear.y = arm_desired_twist_final_(1);
  arm_twist_cmd.linear.z = arm_desired_twist_final_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_final_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_final_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_final_(5);
  pub_arm_cmd_.publish(arm_twist_cmd);
}

//////////////////////
/// INITIALIZATION ///
//////////////////////
void PIDController::wait_for_transformations()
{
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // while (!get_rotation_matrix(rot_matrix, listener, "world", "base_link")) {
  //     sleep(1);
  // }

  while (!get_rotation_matrix(rotation_base_, listener, "base_link", "base_link"))
  {
    sleep(1);
  }

  while (!get_rotation_matrix(rot_matrix, listener, "base_link", "ee_link"))
  {
    sleep(1);
  }
  transformation = true;

  ROS_INFO("The Force/Torque sensor is ready to use.");
}

////////////
/// UTIL ///
////////////

bool PIDController::get_rotation_matrix(Matrix6d &rotation_matrix,
                                        tf::TransformListener &listener,
                                        std::string from_frame,
                                        std::string to_frame)
{
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try
  {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex)
  {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame);
    return false;
  }

  return true;
}
