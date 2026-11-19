#!/usr/bin/env python3
"""Drive one gantry axis sinusoidally, to disturb the arm's base on purpose.

The arm is mounted on the gantry. Measure its end effector in world with
/ee_pose_ground_truth while the gantry moves. For a translating base, publish
the derivative of the velocity command on /gantry/base_linear_acceleration
for the arm's inertial feedforward. This is the commanded acceleration.

The velocity is ramped in and out over `ramp` seconds with a raised cosine, so
that the run does not start with a step in base acceleration that tells you
more about the step than about the controller.

Parameters (private):
  axis        x | y | z | r         which gantry axis to move
  amplitude   m or rad              peak excursion, not peak velocity
  frequency   Hz
  duration    s, 0 for no limit
  ramp        s, raised cosine in and out
  rate        Hz, how often to publish

On exit it commands zero on all four axes.
"""

import math
import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_matrix

AXES = ['x', 'y', 'z', 'r']


def main():
    rospy.init_node('gantry_disturbance')

    axis = rospy.get_param('~axis', 'y')
    amplitude = float(rospy.get_param('~amplitude', 0.02))
    frequency = float(rospy.get_param('~frequency', 1.0))
    duration = float(rospy.get_param('~duration', 0.0))
    ramp = float(rospy.get_param('~ramp', 1.0))
    rate_hz = float(rospy.get_param('~rate', 200.0))

    if axis not in AXES:
        rospy.logerr("axis must be one of %s, got '%s'", AXES, axis)
        return
    if not all(math.isfinite(value) for value in (amplitude, frequency, duration, ramp, rate_hz)):
        rospy.logerr('amplitude, frequency, duration, ramp and rate must be finite')
        return
    if frequency <= 0.0 or rate_hz <= 0.0:
        rospy.logerr('frequency and rate must be positive, got %f and %f', frequency, rate_hz)
        return
    index = AXES.index(axis)

    pub = rospy.Publisher('/gantry_velocity_controller/command',
                          Float64MultiArray, queue_size=1)
    acceleration_pub = rospy.Publisher('/gantry/base_linear_acceleration',
                                       Vector3Stamped, queue_size=1)
    rate = rospy.Rate(rate_hz)
    base_orientation = None

    def read_base_orientation(msg):
        nonlocal base_orientation
        if msg.header.frame_id.lstrip('/') != 'world':
            return
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        if all(math.isfinite(value) for value in q) and sum(value * value for value in q) > 0.0:
            base_orientation = (msg.header.stamp, quaternion_matrix(q)[:3, :3].T)

    if axis != 'r':
        rospy.Subscriber('/base_pose_ground_truth', Odometry, read_base_orientation, queue_size=1)

    omega = 2.0 * math.pi * frequency
    peak_velocity = amplitude * omega
    peak_acceleration = amplitude * omega * omega
    rospy.loginfo('gantry_disturbance: %s axis, %.4f %s at %.2f Hz '
                  '(peak velocity %.4f, peak acceleration %.4f), ramp %.1f s, '
                  'duration %s',
                  axis, amplitude, 'rad' if axis == 'r' else 'm', frequency,
                  peak_velocity, peak_acceleration, ramp,
                  '%.1f s' % duration if duration > 0.0 else 'unlimited')

    # Wait for the clock, so that a sim-time run does not start at t = 0 and
    # see a huge first step.
    while not rospy.is_shutdown() and rospy.Time.now().is_zero():
        rate.sleep()
    start = rospy.Time.now()

    def send(stamp, velocity, acceleration):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(AXES)
        msg.data[index] = velocity
        pub.publish(msg)

        orientation = base_orientation
        if orientation is None:
            return
        orientation_stamp, world_to_base = orientation
        if not 0.0 <= (stamp - orientation_stamp).to_sec() <= 0.12:
            return
        acceleration_msg = Vector3Stamped()
        acceleration_msg.header.stamp = stamp
        acceleration_msg.header.frame_id = 'base_link'
        acceleration_msg.vector.x, acceleration_msg.vector.y, acceleration_msg.vector.z = (
            acceleration * world_to_base[:, index])
        acceleration_pub.publish(acceleration_msg)

    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        t = (stamp - start).to_sec()
        if duration > 0.0 and t > duration + ramp:
            break

        # Raised cosine in, and out again over the last `ramp` seconds.
        envelope = 1.0
        envelope_rate = 0.0
        if ramp > 0.0:
            if t < ramp:
                envelope = 0.5 * (1.0 - math.cos(math.pi * t / ramp))
                envelope_rate = 0.5 * math.pi / ramp * math.sin(math.pi * t / ramp)
            elif duration > 0.0 and t > duration:
                angle = math.pi * (duration + ramp - t) / ramp
                envelope = 0.5 * (1.0 - math.cos(angle))
                envelope_rate = -0.5 * math.pi / ramp * math.sin(angle)

        velocity = peak_velocity * math.cos(omega * t)
        acceleration = -peak_acceleration * math.sin(omega * t)
        send(stamp, envelope * velocity, envelope * acceleration + envelope_rate * velocity)
        rate.sleep()

    # Leave the axis stopped, and say so loudly enough to be seen in a log.
    for _ in range(10):
        send(rospy.Time.now(), 0.0, 0.0)
        rospy.sleep(0.01)
    rospy.loginfo('gantry_disturbance: stopped')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
