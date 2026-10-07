import tempfile
import unittest
from pathlib import Path

import yaml

from waypoint_mission import load_mission


class MissionConfigTest(unittest.TestCase):
    def load(self, waypoints):
        config = {
            'gps_reference': {'latitude': 55.0, 'longitude': 37.0, 'altitude': 100.0},
            'waypoints': waypoints,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.yaml'
            path.write_text(yaml.safe_dump(config))
            return load_mission(path)

    def test_empty_route_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load([])

    def test_nonfinite_goal_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load([{'latitude': float('nan'), 'longitude': 37.0}])

    def test_goal_outside_walls_is_rejected(self):
        with self.assertRaises(ValueError):
            self.load([{'latitude': 55.001, 'longitude': 37.0}])

    def test_demo_route_finishes_at_b(self):
        path = Path(__file__).resolve().parents[1] / 'config' / 'waypoints.yaml'
        _, _, goals = load_mission(path)
        self.assertEqual(len(goals), 6)
        self.assertAlmostEqual(goals[-1][0], 4.0, delta=0.002)
        self.assertAlmostEqual(goals[-1][1], 3.0, delta=0.002)


if __name__ == '__main__':
    unittest.main()
