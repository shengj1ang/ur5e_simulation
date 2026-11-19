#ifndef DISTAL_INERTIA_H
#define DISTAL_INERTIA_H

#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>

#include <cstddef>

namespace controller_interface
{

/** \brief Convert a urdf inertial block to a KDL rigid body inertia
 *
 * The same conversion kdl_parser does internally: the URDF gives the
 * rotational inertia about the centre of mass in the inertial frame, KDL wants
 * it in the link frame, and the only way to rotate a RotationalInertia with
 * KDL's own operators is through a RigidBodyInertia of zero mass.
 */
inline KDL::RigidBodyInertia toKdlInertia(const urdf::InertialSharedPtr &i)
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

inline KDL::Frame jointOrigin(const urdf::Joint &j)
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
inline KDL::RigidBodyInertia accumulateDistalInertia(const urdf::Link &link,
                                                     double *mass)
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

/** \brief kdl_chain_ with everything rigidly attached beyond its tip folded
 * into the tip segment
 *
 * Returns the chain unchanged, and leaves *mass at zero, when the robot
 * description has nothing with mass past the tip.
 */
inline KDL::Chain chainWithTipPayload(const KDL::Chain &chain,
                                      const urdf::Model &model,
                                      const std::string &tip_name,
                                      double *mass,
                                      KDL::Vector *centre_of_mass)
{
  *mass = 0.0;
  *centre_of_mass = KDL::Vector::Zero();

  urdf::LinkConstSharedPtr tip = model.getLink(tip_name);
  if (!tip || chain.getNrOfSegments() == 0)
    return chain;

  double walked = 0.0;
  const KDL::RigidBodyInertia payload = accumulateDistalInertia(*tip, &walked);
  if (payload.getMass() <= 0.0)
    return chain;

  *mass = payload.getMass();
  *centre_of_mass = payload.getCOG();

  const KDL::Segment &old_tip = chain.getSegment(chain.getNrOfSegments()-1);
  KDL::Chain extended;
  for (unsigned int s = 0; s + 1 < chain.getNrOfSegments(); s++)
    extended.addSegment(chain.getSegment(s));
  extended.addSegment(KDL::Segment(old_tip.getName(), old_tip.getJoint(),
                                   old_tip.getFrameToTip(),
                                   old_tip.getInertia() + payload));
  return extended;
}

} // namespace controller_interface

#endif
