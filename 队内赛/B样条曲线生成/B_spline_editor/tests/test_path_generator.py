import math
import unittest

import numpy as np

from path.path_generator import generate_path, yaw_from_heading


class YawConventionTests(unittest.TestCase):
    def test_forward_axis_has_zero_yaw(self):
        self.assertAlmostEqual(yaw_from_heading(math.pi / 2.0, math.pi / 2.0), 0.0)
        self.assertAlmostEqual(
            yaw_from_heading(-3.0 * math.pi / 2.0, math.pi / 2.0), 0.0, places=12
        )

    def test_counter_clockwise_is_positive_when_forward_is_plus_y(self):
        # Facing +Y, turning left (towards -X) gives +90 degrees.
        self.assertAlmostEqual(yaw_from_heading(math.pi, math.pi / 2.0), math.pi / 2.0)
        # Facing +Y, turning right (towards +X) gives -90 degrees.
        self.assertAlmostEqual(yaw_from_heading(0.0, math.pi / 2.0), -math.pi / 2.0)

    def test_yaw_wraps_at_pi(self):
        yaw = yaw_from_heading(-math.pi / 2.0, math.pi / 2.0)
        self.assertAlmostEqual(abs(yaw), math.pi)
        self.assertGreaterEqual(yaw, -math.pi)
        self.assertLessEqual(yaw, math.pi)

    def test_plus_x_forward_configuration_is_still_supported(self):
        self.assertAlmostEqual(yaw_from_heading(0.0, 0.0), 0.0)
        self.assertAlmostEqual(yaw_from_heading(math.pi / 2.0, 0.0), math.pi / 2.0)

    def test_upward_path_gets_zero_yaw(self):
        points = np.column_stack((np.full(101, 500.0), np.linspace(0.0, 1000.0, 101)))
        path = generate_path(
            points,
            lateral_acceleration=2.0,
            speed_scale=1.0,
            acceleration=1.0,
            deceleration=1.0,
        )
        self.assertAlmostEqual(path[0].theta, math.pi / 2.0, places=3)
        self.assertAlmostEqual(yaw_from_heading(path[0].theta, math.pi / 2.0), 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
