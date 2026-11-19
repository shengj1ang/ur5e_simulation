#ifndef KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define KINEMATIC_CHAIN_CONTROLLER_BASE_H

#include <urdf/model.h>
#include <controller_interface/controller.h>

#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <cmath>
#include <memory>
#include <vector>

namespace controller_interface
{
template<typename JI>
class KinematicChainControllerBase: public Controller<JI>
{
public:
  KinematicChainControllerBase() {}
  ~KinematicChainControllerBase() {}

  bool init(JI *robot, ros::NodeHandle &n);

  /** \brief Read the joint positions from the hardware interface into
   * joint_msr_.q, and estimate joint_msr_.qdot from them
   *
   * The joint velocity reported by the hardware interface is not used, because
   * under Gazebo it is not the velocity of the joint. gazebo_ros_control's
   * DefaultRobotHWSim fills it from Gazebo's GetVelocity(), and this model
   * drives its joints through a PositionJointInterface with no
   * /gazebo_ros_control/pid_gains, which means SetPosition() moves them
   * kinematically without ever setting a velocity. What GetVelocity() returns
   * instead is whatever the constraint solver has, i.e. the velocity the links
   * would pick up from gravity if the position were not being re-asserted.
   *
   * Measured in Gazebo on this model, same run: while arm_joint_controller
   * swept elbow_joint through 1.565 rad in 11 s - a true 0.1424 rad/s - the
   * reported velocity averaged 0.00022 rad/s, three orders of magnitude out.
   * At rest it is not zero either, and it depends on the pose: shoulder_lift
   * reports 0.0416 rad/s fully extended and 0.0114 rad/s at the SRDF 'init'
   * pose, in both cases while its position does not move at all to five
   * decimals. So the signal is not a biased velocity, it is unrelated to the
   * velocity.
   *
   * Differentiating the position is sound here because the position is
   * trustworthy: the position interface tracks its setpoint exactly, so the
   * only error in q is Gazebo's own. Note what that means for anything built
   * on this: in simulation the plant between command and position is close to
   * a pure integrator, so a differentiated position is close to the commanded
   * velocity, and a twist computed from it will not expose a tracking failure
   * that shows up as a lag rather than as a position error. It is the velocity
   * of the simulated robot, which is what the controllers need, but it is not
   * independent evidence that the robot did what it was told.
   *
   * \param period The duration of an update cycle, as ros_control measures it
   */
  void readJointStates(const ros::Duration& period);

  /** \brief Forget the previous position, so that the next readJointStates()
   * starts the estimate over instead of differentiating across the gap
   *
   * Call this from starting(): a controller can be switched in long after the
   * last time it ran, and the joints may have moved in between.
   */
  void resetJointStateEstimate() { have_q_prev_ = false; }

protected:
  ros::NodeHandle nh_;
  KDL::Chain kdl_chain_;
  KDL::JntArrayVel joint_msr_;

  struct limits_
  {
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
  } joint_limits_;

  std::vector<typename JI::ResourceHandleType> joint_handles_;

  // Joint velocity estimation - see readJointStates()
  KDL::JntArray q_prev_;              // position at the previous cycle
  bool have_q_prev_ = false;          // false until the first cycle is in
  double velocity_filter_cutoff_ = 0.0;  // Hz; <=0 means no filtering
};

template <typename JI>
bool KinematicChainControllerBase<JI>::init(JI *robot, ros::NodeHandle &n)
{
  nh_ = n;

  // get URDF and name of root and tip from the parameter server
  std::string robot_description, root_name, tip_name;

  std::string name_space = nh_.getNamespace();
  std::cout<< "--------------------> name_space:  " << name_space << std::endl;

  if (!ros::param::search(name_space,"robot_description", robot_description))
  {
    ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF)"
                     "found on parameter server (" << n.getNamespace() <<
                     "/robot_description)");
    return false;
  }

  if (!nh_.getParam( name_space + "/root_name", root_name))
  {
    ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on "
                     "parameter server ("<<n.getNamespace()<<"/root_name)");
    return false;
  }

  if (!nh_.getParam(name_space + "/tip_name", tip_name))
  {
    ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on "
                     "parameter server ("<<n.getNamespace()<<"/tip_name)");
    return false;
  }

  // Construct an URDF model from the xml string
  std::string xml_string;

  if (nh_.hasParam(robot_description))
    nh_.getParam(robot_description.c_str(), xml_string);
  else
  {
    ROS_ERROR("Parameter %s not set, shutting down node...",
              robot_description.c_str());
    nh_.shutdown();
    return false;
  }

  if (xml_string.size() == 0)
  {
    ROS_ERROR("Unable to load robot model from parameter %s",
              robot_description.c_str());
    nh_.shutdown();
    return false;
  }

  //ROS_INFO("%s content\n%s", robot_description.c_str(), xml_string.c_str());

  // Get urdf model out of robot_description
  urdf::Model model;
  if (!model.initString(xml_string))
  {
    ROS_ERROR("Failed to parse urdf file");
    n.shutdown();
    return false;
  }
  ROS_INFO("Successfully parsed urdf file");

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    n.shutdown();
    return false;
  }

  // Populate the KDL chain
  if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
    ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
    ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
    ROS_ERROR_STREAM("  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree.getSegments();
    KDL::SegmentMap::iterator it;

    for( it=segment_map.begin(); it != segment_map.end(); it++ )
      ROS_ERROR_STREAM( "    "<<(*it).first);

    return false;
  }

  joint_msr_.resize(kdl_chain_.getNrOfJoints());
  q_prev_.resize(kdl_chain_.getNrOfJoints());
  KDL::SetToZero(joint_msr_.q);
  KDL::SetToZero(joint_msr_.qdot);
  KDL::SetToZero(q_prev_);
  have_q_prev_ = false;

  // Cutoff of the first order low pass on the differentiated velocity. The
  // differentiation is of an exact position in simulation, so the default is
  // light; on hardware, where the position is quantised by an encoder, it
  // wants to be lower. 0 or less turns the filter off.
  nh_.param("velocity_filter_cutoff", velocity_filter_cutoff_, 30.0);

  ROS_INFO("tip_name:  %s",tip_name.c_str());
  ROS_INFO("root_name: %s",root_name.c_str());
  ROS_INFO("Number of segments: %d", kdl_chain_.getNrOfSegments());
  ROS_INFO("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
  for(std::size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++){
    ROS_INFO_STREAM("segment("<<i<<"): " << kdl_chain_.getSegment(i).getName());
  }


  // Parsing joint limits from urdf model along kdl chain
  std::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
  std::shared_ptr<const urdf::Joint> joint_;
  joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
  joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
  joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
  int index;

  for (std::size_t i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
  {
    joint_ = model.getJoint(link_->parent_joint->name);
    ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
    index = kdl_chain_.getNrOfJoints() - i - 1;

    if(joint_->limits){
      joint_limits_.min(index) = joint_->limits->lower;
      joint_limits_.max(index) = joint_->limits->upper;
      joint_limits_.center(index) = (joint_limits_.min(index) +
                                     joint_limits_.max(index))/2;
    }else{
      joint_limits_.min(index) = 0;
      joint_limits_.max(index) = 0;
      joint_limits_.center(index) = 0;
      ROS_INFO("joint_->limits is NULL %s",joint_->name.c_str());
    }

    link_ = model.getLink(link_->getParent()->name);
  }

  ROS_INFO("Getting joint handles");
  // Get joint handles for all of the joints in the chain
  int count=0;
  for(std::vector<KDL::Segment>::const_iterator it =
      kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
  {

    ROS_INFO("%s type: %s", it->getJoint().getName().c_str(),
             it->getJoint().getTypeName().c_str() );
    if(it->getJoint().getTypeName() != "None" && count < 7) {
      joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
    }
    count++;
  }

  ROS_INFO("Number of joints in handle = %lu", joint_handles_.size() );
  ROS_INFO_STREAM("kdl_chain.getNrOfJoints: " << kdl_chain_.getNrOfJoints());

  ROS_INFO("Finished Kinematic Base init");
  ROS_INFO("Joint velocity is differentiated from position, low pass at %.1f Hz",
           velocity_filter_cutoff_);

  return true;
}

template <typename JI>
void KinematicChainControllerBase<JI>::readJointStates(const ros::Duration& period)
{
  for (std::size_t i = 0; i < joint_handles_.size(); i++)
    joint_msr_.q(i) = joint_handles_[i].getPosition();

  if (!have_q_prev_)
  {
    for (std::size_t i = 0; i < joint_handles_.size(); i++)
    {
      q_prev_(i) = joint_msr_.q(i);
      joint_msr_.qdot(i) = 0.0;
    }
    have_q_prev_ = true;
    return;
  }

  const double dt = period.toSec();
  if (dt <= 0.0)
    return;   // keep the previous estimate, and q_prev_ with it

  // First order low pass, alpha = dt/(tau+dt) with tau = 1/(2*pi*fc)
  double alpha = 1.0;
  if (velocity_filter_cutoff_ > 0.0)
  {
    const double tau = 1.0/(2.0*M_PI*velocity_filter_cutoff_);
    alpha = dt/(tau + dt);
  }

  for (std::size_t i = 0; i < joint_handles_.size(); i++)
  {
    const double raw = (joint_msr_.q(i) - q_prev_(i))/dt;
    joint_msr_.qdot(i) += alpha*(raw - joint_msr_.qdot(i));
    q_prev_(i) = joint_msr_.q(i);
  }
}

}

#endif
