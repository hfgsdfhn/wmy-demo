import json
import tempfile
import unittest
from pathlib import Path

from config.path_config import CONFIG_FORMAT, CONFIG_VERSION, PathConfigError, load_path_config, save_path_config
from config.coordinate_system import CoordinateSystem


def sample_config():
    return {"format": CONFIG_FORMAT, "version": CONFIG_VERSION,
            "control_points": [[100.0, 200.0], [300.0, 400.0]],
            "field": {"width_mm": 1200.0, "height_mm": 800.0},
            "sampling": {"mode": "interval", "interval_mm": 50.0, "count": 200},
            "speed": {"lateral_acceleration": 2.0, "speed_scale": 1.0,
                      "acceleration": 1.0, "deceleration": 1.0},
            "mpc": {"sample_time": 0.01},
            "coordinate_system": {"origin": "bottom_left", "origin_offset_mm": [0.0, 0.0],
                                   "x_direction": "positive_right",
                                   "y_direction": "positive_up"}}


class PathConfigTests(unittest.TestCase):
    def test_round_trip_preserves_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "nested" / "path.json"
            config = sample_config()
            save_path_config(filename, config)
            self.assertEqual(load_path_config(filename), config)

    def test_allows_fewer_than_four_control_points(self):
        config = sample_config()
        config["control_points"] = []
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "path.json"
            save_path_config(filename, config)
            self.assertEqual(load_path_config(filename)["control_points"], [])

    def test_rejects_invalid_point(self):
        config = sample_config()
        config["control_points"] = [[1300.0, 0.0]]
        with self.assertRaises(PathConfigError):
            save_path_config(Path("unused.json"), config)

    def test_rejects_unrelated_json(self):
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "bad.json"
            filename.write_text('{"hello": "world"}', encoding="utf-8")
            with self.assertRaises(PathConfigError):
                load_path_config(filename)

    def test_version_one_defaults_to_legacy_coordinate_system(self):
        config = sample_config()
        config.pop("coordinate_system")
        config["version"] = 1
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "legacy.json"
            filename.write_text(json.dumps(config), encoding="utf-8")
            loaded = load_path_config(filename)
        self.assertEqual(loaded["version"], CONFIG_VERSION)
        self.assertEqual(loaded["coordinate_system"]["origin"], "bottom_left")
        self.assertEqual(loaded["coordinate_system"]["origin_offset_mm"], [0.0, 0.0])

    def test_version_two_defaults_to_ten_millisecond_mpc_period(self):
        config = sample_config()
        config.pop("mpc")
        config["version"] = 2
        with tempfile.TemporaryDirectory() as directory:
            filename = Path(directory) / "old.json"
            filename.write_text(json.dumps(config), encoding="utf-8")
            loaded = load_path_config(filename)
        self.assertEqual(loaded["mpc"]["sample_time"], 0.01)
        self.assertEqual(loaded["version"], CONFIG_VERSION)

    def test_rejects_mpc_period_outside_supported_range(self):
        config = sample_config()
        config["mpc"]["sample_time"] = 0.0001
        with self.assertRaises(PathConfigError):
            save_path_config(Path("unused.json"), config)

    def test_coordinate_system_round_trip_and_inverse(self):
        system = CoordinateSystem("top_right", "positive_left", "positive_down")
        display = system.to_display((100.0, 200.0), 1000.0, 800.0)
        self.assertEqual(display, (-900.0, -600.0))
        self.assertEqual(system.to_coordinate((100.0, 200.0), 1000.0, 800.0), (900.0, 600.0))
        self.assertEqual(system.to_field(display, 1000.0, 800.0), (100.0, 200.0))

    def test_all_coordinate_system_rectangles_keep_field_size(self):
        for origin in ("bottom_left", "top_left", "bottom_right", "top_right"):
            for x_direction, y_direction in (("positive_right", "positive_up"),
                                              ("positive_right", "positive_down"),
                                              ("positive_left", "positive_up"),
                                              ("positive_left", "positive_down")):
                rect = CoordinateSystem(origin, x_direction, y_direction).field_rect(1000.0, 800.0)
                self.assertEqual(abs(rect[2]), 1000.0)
                self.assertEqual(abs(rect[3]), 800.0)

    def test_origin_offset_changes_origin_and_rejects_out_of_bounds(self):
        system = CoordinateSystem("bottom_left", origin_offset=(100.0, 200.0))
        self.assertEqual(system.origin_point(1000.0, 800.0), (100.0, 200.0))
        config = sample_config()
        config["coordinate_system"]["origin_offset_mm"] = [-1.0, 0.0]
        with self.assertRaises(PathConfigError):
            save_path_config(Path("unused.json"), config)


if __name__ == "__main__":
    unittest.main()
