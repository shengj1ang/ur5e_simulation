#!/usr/bin/env python3
"""Drive one gantry axis sinusoidally, to disturb the arm's base on purpose.

The arm is mounted on the gantry. Measure its end effector in world with
/ee_pose_ground_truth while the gantry moves. For a translating base, publish
the derivative of the velocity command on /gantry/base_linear_acceleration for
the arm's inertial feedforward, corrected for the gantry's own velocity loop so
that the topic carries what the base does rather than what it was asked for.
See BaseResponse.

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

  base_response_frequency  Hz, the gantry velocity loop's natural frequency.
                           <=0 publishes the commanded acceleration uncorrected.
  base_response_damping    its damping ratio.

On exit it commands zero on all four axes.
"""

import math
import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_matrix

AXES = ['x', 'y', 'z', 'r']

# The gantry's velocity loop, identified in docs/experiments/inertia_mapping_200hz.
BASE_RESPONSE_FREQUENCY = 20.61
BASE_RESPONSE_DAMPING = 0.25


class BaseResponse:
    """The gantry's velocity loop, as the second-order low pass it measures as.

    This node knows the acceleration it asked the gantry for, not the one the
    base gets, and the two differ systematically. Measured over 0.5 to 8 Hz the
    base overshoots in amplitude and lags in phase, by 15.1 % and 12.5 degrees
    at 8 Hz, growing as about f squared and f respectively. The shape is the
    same at 20 and 30 mm of travel and the same whether the arm's own
    feedforward is on or off, so it is the gantry's linear response rather than
    an amplitude effect or a reaction from the arm.

    One natural frequency and one damping ratio account for all of it, to
    within 0.4 % and 0.4 degrees across that band. Passing the commanded
    acceleration through them leaves the velocity command alone - the
    disturbance itself is unchanged, so earlier measurements stay comparable -
    while the topic goes from 15 % wrong at the top of the band to 1.3 %,
    measured against the base's own motion in base-response-verification.csv.

    Integrated trapezoidally on the output and its rate, so a varying dt stays
    meaningful and the filter adds no delay of its own. At 200 Hz it tracks the
    continuous response to 0.002 and 0.1 degrees at 8 Hz; at 50 Hz that grows
    to 0.03 and 1.6 degrees, still far better than not correcting at all.

    Identified from docs/experiments/inertia_mapping_200hz/base-acceleration-
    comparison.csv at a 200 Hz publish rate. That fit folds in the hold on the
    velocity command, so re-identify it if `rate` changes much.
    """

    def __init__(self, frequency, damping):
        self.omega = 2.0 * math.pi * frequency
        self.damping = damping
        self.output = 0.0
        self.output_rate = 0.0
        self.last_input = None

    def __call__(self, acceleration, time_step):
        """The base's acceleration, given the one commanded of the gantry."""
        if self.last_input is None or time_step <= 0.0:
            self.last_input = acceleration
            return self.output
        half = 0.5 * time_step
        stiffness = half * self.omega * self.omega
        decay = 2.0 * half * self.damping * self.omega
        determinant = 1.0 + decay + half * stiffness
        first = self.output + half * self.output_rate
        second = (-stiffness * self.output + (1.0 - decay) * self.output_rate
                  + stiffness * (self.last_input + acceleration))
        self.output = ((1.0 + decay) * first + half * second) / determinant
        self.output_rate = (-stiffness * first + second) / determinant
        self.last_input = acceleration
        return self.output


def main():
    rospy.init_node('gantry_disturbance')

    axis = rospy.get_param('~axis', 'y')
    amplitude = float(rospy.get_param('~amplitude', 0.02))
    frequency = float(rospy.get_param('~frequency', 1.0))
    duration = float(rospy.get_param('~duration', 0.0))
    ramp = float(rospy.get_param('~ramp', 1.0))
    rate_hz = float(rospy.get_param('~rate', 200.0))
    response_frequency = float(rospy.get_param('~base_response_frequency',
                                               BASE_RESPONSE_FREQUENCY))
    response_damping = float(rospy.get_param('~base_response_damping',
                                             BASE_RESPONSE_DAMPING))

    if axis not in AXES:
        rospy.logerr("axis must be one of %s, got '%s'", AXES, axis)
        return
    if not all(math.isfinite(value) for value in (amplitude, frequency, duration, ramp,
                                                  rate_hz, response_frequency,
                                                  response_damping)):
        rospy.logerr('amplitude, frequency, duration, ramp, rate and the base response '
                     'parameters must be finite')
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

    base_response = None
    if axis == 'r':
        rospy.loginfo('gantry_disturbance: the r axis is rotational, so no linear acceleration will be published')
    else:
        rospy.Subscriber('/base_pose_ground_truth', Odometry, read_base_orientation, queue_size=1)
        if response_frequency > 0.0:
            base_response = BaseResponse(response_frequency, response_damping)
            rospy.loginfo('gantry_disturbance: correcting the published acceleration for the '
                          'gantry velocity loop, %.2f Hz at damping %.3f',
                          response_frequency, response_damping)
        else:
            rospy.logwarn('gantry_disturbance: publishing the commanded acceleration '
                          'uncorrected, which measures 15 %% high and 12.5 degrees late '
                          'at 8 Hz')

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
    last_stamp = None

    def send(stamp, velocity, acceleration):
        msg = Float64MultiArray()
        msg.data = [0.0] * len(AXES)
        msg.data[index] = velocity
        pub.publish(msg)

        if axis == 'r':
            return
        orientation = base_orientation
        if orientation is None:
            rospy.logwarn_once('gantry_disturbance: no valid base orientation received from /base_pose_ground_truth, '
                               'withholding linear acceleration')
            return
        orientation_stamp, world_to_base = orientation
        if not 0.0 <= (stamp - orientation_stamp).to_sec() <= 0.12:
            rospy.logwarn_once('gantry_disturbance: base orientation timestamp is stale or in the future, '
                               'withholding linear acceleration')
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

        # The product rule, because the envelope is part of the motion.
        commanded_acceleration = envelope * acceleration + envelope_rate * velocity
        if base_response is None:
            published_acceleration = commanded_acceleration
        else:
            time_step = 0.0 if last_stamp is None else (stamp - last_stamp).to_sec()
            published_acceleration = base_response(commanded_acceleration, time_step)
        last_stamp = stamp

        send(stamp, envelope * velocity, published_acceleration)
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
