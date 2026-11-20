"""Exercise simulated-time and wall-time failures without waiting in real time."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import rospkg


SCRIPT = Path(rospkg.RosPack().get_path("sim_bringup"))/"scripts"/"init_pose_start_controllers.py"
SPEC = importlib.util.spec_from_file_location("init_pose_start_controllers", SCRIPT)
handoff = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(handoff)


class SimulatedWait:
    def __init__(self, rtf, velocity=0.0, start_after=0.0, pause_after=float("inf")):
        self.rtf = rtf
        self.velocity = velocity
        self.start_after = start_after
        self.pause_after = pause_after
        self.wall = 0.0
        self.origin = 10.0
        self.next_sample = self.origin + .02
        self.subscriber = Mock()
        self.ros = SimpleNamespace(Subscriber=self.subscribe, get_time=self.ros_time,
                                   is_shutdown=lambda: False, logerr=Mock())

    def ros_time(self):
        if self.wall < self.start_after:
            return 0.0
        return self.origin + self.rtf*(min(self.wall, self.pause_after)-self.start_after)

    def subscribe(self, topic, message_type, callback, queue_size):
        self.callback = callback
        return self.subscriber

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def notify(self):
        pass

    def wait(self, timeout):
        self.wall += timeout
        if self.wall > 130.0:
            raise AssertionError("Settle check did not return in simulated time")
        while self.next_sample <= self.ros_time():
            stamp = self.next_sample
            header = SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp))
            self.callback(SimpleNamespace(header=header, name=["arm_joint"],
                                          position=[self.velocity*(stamp-self.origin)]))
            self.next_sample += .02

    def run(self):
        wall_clock = SimpleNamespace(monotonic=lambda: self.wall)
        threading = SimpleNamespace(Condition=lambda: self)
        with patch.multiple(handoff, rospy=self.ros, time=wall_clock, threading=threading):
            result = handoff.wait_for_arm_settle(["arm_joint"])
        self.subscriber.unregister.assert_called_once()
        return result


class SettleTest(unittest.TestCase):
    def test_slow_clock_can_take_more_than_sixty_wall_seconds_to_settle(self):
        sim = SimulatedWait(rtf=.005)
        self.assertTrue(sim.run())
        self.assertGreater(sim.wall, 60.0)
        sim.ros.logerr.assert_not_called()

    def test_moving_arm_exhausts_five_ros_seconds(self):
        sim = SimulatedWait(rtf=.05, velocity=.1)
        self.assertFalse(sim.run())
        self.assertAlmostEqual(sim.ros_time()-sim.origin, 5.0, delta=.01)
        self.assertIn("within 5 seconds of ROS time", sim.ros.logerr.call_args[0][0])

    def test_pause_exhausts_wall_watchdog_after_last_clock_progress(self):
        sim = SimulatedWait(rtf=.025, pause_after=8.0)
        self.assertFalse(sim.run())
        self.assertAlmostEqual(sim.wall, 68.0, delta=.2)
        self.assertIn("60 seconds of wall time", sim.ros.logerr.call_args[0][0])

    def test_first_clock_sample_can_start_at_nonzero_simulation_time(self):
        sim = SimulatedWait(rtf=.5, start_after=2.0)
        self.assertTrue(sim.run())
        self.assertGreater(sim.wall, 2.0)
        sim.ros.logerr.assert_not_called()


if __name__ == "__main__":
    unittest.main()
