#!/usr/bin/env python3
"""Drive one gantry axis sinusoidally, to disturb the arm's base on purpose.

The arm is mounted on the gantry, so moving the gantry moves the arm's base
while the arm is holding its end effector somewhere. That is the disturbance
sim_pid_control's N_p and N_d terms feed forward against, and the one the
operational-space inertia in sim_cartesian_dynamics_controller describes the
cost of: to leave the end effector where it is, the arm has to accelerate its
own inertia against the base motion.

Note what the controllers do and do not see. ee_state reports the end-effector
pose in base_link, so the feedback loop is blind to the base moving - the arm
can hold a perfect base_link pose while the end effector swings through the
room. The quantity to measure is therefore the end effector in world, which
/ee_pose_ground_truth publishes.

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
from std_msgs.msg import Float64MultiArray

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
    if frequency <= 0.0:
        rospy.logerr('frequency must be positive, got %f', frequency)
        return
    index = AXES.index(axis)

    pub = rospy.Publisher('/gantry_velocity_controller/command',
                          Float64MultiArray, queue_size=1)
    rate = rospy.Rate(rate_hz)

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

    def send(velocity):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(AXES)
        msg.data[index] = velocity
        pub.publish(msg)

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - start).to_sec()
        if duration > 0.0 and t > duration + ramp:
            break

        # Raised cosine in, and out again over the last `ramp` seconds.
        envelope = 1.0
        if ramp > 0.0:
            if t < ramp:
                envelope = 0.5 * (1.0 - math.cos(math.pi * t / ramp))
            elif duration > 0.0 and t > duration:
                left = (duration + ramp - t) / ramp
                envelope = 0.5 * (1.0 - math.cos(math.pi * max(left, 0.0)))

        send(envelope * peak_velocity * math.cos(omega * t))
        rate.sleep()

    # Leave the axis stopped, and say so loudly enough to be seen in a log.
    for _ in range(10):
        send(0.0)
        rospy.sleep(0.01)
    rospy.loginfo('gantry_disturbance: stopped')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
