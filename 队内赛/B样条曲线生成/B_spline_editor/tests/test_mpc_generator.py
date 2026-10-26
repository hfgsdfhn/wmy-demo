import math
import unittest

import numpy as np

from path.exporter import export_mpc_c_array, export_mpc_csv
from path.mpc_generator import DEFAULT_SAMPLE_TIME, generate_mpc_states
from path.path_generator import PathPoint, generate_path


def straight_points(length_mm=1000.0, spacing_mm=1.0, speed=1.0):
    count = int(round(length_mm / spacing_mm)) + 1
    return [
        PathPoint(
            x=min(index * spacing_mm, length_mm),
            y=0.0,
            theta=0.0,
            curvature=0.0,
            s=min(index * spacing_mm, length_mm),
            velocity=speed,
        )
        for index in range(count)
    ]


def arc_points(radius_mm=500.0, speed=0.5, steps=1000):
    points = []
    for index in range(steps + 1):
        angle = (math.pi / 2.0) * index / steps
        points.append(
            PathPoint(
                x=radius_mm * math.cos(angle),
                y=radius_mm * math.sin(angle),
                theta=angle + math.pi / 2.0,
                curvature=1000.0 / radius_mm,
                s=radius_mm * angle,
                velocity=speed,
            )
        )
    return points


class MpcGeneratorTests(unittest.TestCase):
    def test_uses_default_sample_time_of_ten_milliseconds(self):
        self.assertEqual(DEFAULT_SAMPLE_TIME, 0.01)

    def test_uniform_time_grid_hits_the_path_end(self):
        states = generate_mpc_states(straight_points(1000.0), DEFAULT_SAMPLE_TIME)
        self.assertEqual(len(states), 101)
        self.assertEqual(states[0].time, 0.0)
        self.assertEqual(states[0].x, 0.0)
        self.assertAlmostEqual(states[-1].time, 1.0)
        self.assertAlmostEqual(states[-1].s, 1000.0)
        for previous, current in zip(states, states[1:]):
            self.assertAlmostEqual(current.time - previous.time, 0.01)
        midpoint = states[50]
        self.assertAlmostEqual(midpoint.time, 0.5)
        self.assertAlmostEqual(midpoint.x, 500.0)
        self.assertAlmostEqual(midpoint.velocity, 1.0)
        self.assertEqual(midpoint.omega, 0.0)
        self.assertEqual(midpoint.acceleration, 0.0)

    def test_appends_a_short_final_step_for_a_partial_period(self):
        states = generate_mpc_states(straight_points(1055.0), 0.01)
        self.assertEqual(len(states), 107)
        self.assertAlmostEqual(states[-2].time, 1.05)
        self.assertAlmostEqual(states[-1].time, 1.055)
        self.assertAlmostEqual(states[-1].s, 1055.0)
        self.assertAlmostEqual(states[-1].time - states[-2].time, 0.005)

    def test_time_sampling_matches_the_planned_speed_profile(self):
        points = np.column_stack((np.linspace(0.0, 1200.0, 1201), np.zeros(1201)))
        path = generate_path(
            points,
            lateral_acceleration=2.0,
            speed_scale=1.0,
            acceleration=1.0,
            deceleration=1.0,
        )
        states = generate_mpc_states(path, 0.02)
        self.assertAlmostEqual(states[0].velocity, 0.0)
        self.assertLess(abs(states[-1].velocity), 1e-9)
        self.assertAlmostEqual(states[-1].x, 1200.0)
        travelled = sum(
            0.5 * (first.velocity + second.velocity) * (second.time - first.time)
            for first, second in zip(states, states[1:])
        )
        self.assertAlmostEqual(travelled, 1.2, delta=1e-3)
        self.assertAlmostEqual(states[-1].s, 1200.0)
        self.assertLessEqual(
            max(abs(state.acceleration) for state in states), 1.0 + 1e-9
        )
        for first, second in zip(states, states[1:]):
            self.assertLessEqual(second.time - first.time, 0.02 + 1e-9)

    def test_finer_sample_time_produces_proportionally_more_states(self):
        points = straight_points(1000.0)
        fine = generate_mpc_states(points, 0.01)
        coarse = generate_mpc_states(points, 0.05)
        self.assertAlmostEqual(len(fine), 5 * len(coarse), delta=6)

    def test_arc_state_keeps_radius_speed_and_yaw_rate(self):
        states = generate_mpc_states(arc_points(), 0.01)
        for state in states[::10]:
            self.assertAlmostEqual(math.hypot(state.x, state.y), 500.0, delta=0.05)
            self.assertAlmostEqual(state.velocity, 0.5, places=6)
            self.assertAlmostEqual(state.curvature, 2.0, places=3)
            self.assertAlmostEqual(state.omega, 1.0, places=4)
            self.assertGreater(state.theta, 0.0)

    def test_theta_is_unwrapped_across_the_pi_boundary(self):
        points = [
            PathPoint(0.0, 0.0, 3.0, 0.0, 0.0, 1.0),
            PathPoint(1.0, 0.0, 3.1, 0.0, 1.0, 1.0),
            PathPoint(2.0, 0.0, -3.1, 0.0, 2.0, 1.0),
            PathPoint(3.0, 0.0, -3.0, 0.0, 3.0, 1.0),
        ]
        states = generate_mpc_states(points, 0.01)
        self.assertGreater(states[-1].theta, 3.0)
        self.assertLess(states[-1].theta, 3.4)
        for previous, current in zip(states, states[1:]):
            self.assertLess(abs(current.theta - previous.theta), 1.0)

    def test_rejects_stationary_speed_profile(self):
        points = straight_points(100.0, speed=0.0)
        with self.assertRaises(ValueError):
            generate_mpc_states(points, 0.01)

    def test_rejects_invalid_sample_time(self):
        points = straight_points(100.0)
        for sample_time in (0.0, -0.01, 5.0, float("nan")):
            with self.assertRaises(ValueError):
                generate_mpc_states(points, sample_time)

    def test_handles_empty_and_degenerate_paths(self):
        self.assertEqual(generate_mpc_states([]), [])
        single = [PathPoint(10.0, 20.0, 0.5, 0.1, 0.0, 0.0)]
        states = generate_mpc_states(single)
        self.assertEqual(len(states), 1)
        self.assertEqual(states[0].time, 0.0)
        self.assertEqual((states[0].x, states[0].y), (10.0, 20.0))
        duplicate = [
            PathPoint(10.0, 20.0, 0.0, 0.0, 0.0, 1.0),
            PathPoint(10.0, 20.0, 0.0, 0.0, 0.0, 1.0),
        ]
        self.assertEqual(len(generate_mpc_states(duplicate)), 1)

    def test_c_array_and_csv_exports(self):
        states = generate_mpc_states(straight_points(100.0), 0.01)
        source = export_mpc_c_array(states, 0.01)
        self.assertIn("#define MPC_SAMPLE_TIME 0.010000f", source)
        self.assertIn("#define MPC_STATE_COUNT 11", source)
        self.assertIn("const MpcState mpc_reference[MPC_STATE_COUNT]", source)
        self.assertEqual(source.count("\n    {"), 11)
        csv = export_mpc_csv(states)
        self.assertTrue(csv.startswith("step,time_s,x_mm,y_mm,theta_rad"))
        self.assertEqual(len(csv.strip().splitlines()), 12)
        with self.assertRaises(ValueError):
            export_mpc_c_array([], 0.01)
        with self.assertRaises(ValueError):
            export_mpc_csv([])


if __name__ == "__main__":
    unittest.main()
