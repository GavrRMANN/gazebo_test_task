import math
import unittest
from pathlib import Path

from check_navigation import clearance, obstacles, obstacle_contact, polygon_clearance, rectangle


class NavigationGeometryTest(unittest.TestCase):
    def test_ground_contact_is_allowed(self):
        self.assertFalse(obstacle_contact('autonomy_robot::wheel::collision', 'ground_plane::link::collision'))

    def test_obstacle_contact_fails_in_either_order(self):
        robot, wall = 'autonomy_robot::base::collision', 'maze_baffle_west::link::collision'
        self.assertTrue(obstacle_contact(robot, wall))
        self.assertTrue(obstacle_contact(wall, robot))

    def test_other_objects_are_not_robot_contacts(self):
        self.assertFalse(obstacle_contact('wall::link::collision', 'ground_plane::link::collision'))

    def test_gap_between_axis_aligned_boxes(self):
        robot = rectangle(0, 0, 0, 0.27, 0.23)
        obstacle = rectangle(1, 0, 0, 0.2, 0.2)
        self.assertAlmostEqual(polygon_clearance(robot, obstacle), 0.53)

    def test_rotation_changes_clearance(self):
        robot = rectangle(0, 0, math.pi / 2, 0.27, 0.23)
        obstacle = rectangle(1, 0, 0, 0.2, 0.2)
        self.assertAlmostEqual(polygon_clearance(robot, obstacle), 0.57)

    def test_overlapping_boxes_fail(self):
        robot = rectangle(0, 0, 0, 0.27, 0.23)
        obstacle = rectangle(0.4, 0, 0.3, 0.2, 0.2)
        self.assertLess(polygon_clearance(robot, obstacle), 0)

    def test_contact_is_not_positive_clearance(self):
        robot = rectangle(0, 0, 0, 0.25, 0.25)
        obstacle = rectangle(0.5, 0, 0, 0.25, 0.25)
        self.assertLessEqual(polygon_clearance(robot, obstacle), 0)

    def test_cylinder_distance_accounts_for_robot_size(self):
        self.assertAlmostEqual(clearance((0, 0, 0), [('circle', (1, 0, 0.45))]), 0.28)

    def test_all_world_barriers_are_checked(self):
        path = Path(__file__).resolve().parents[1] / 'src/autonomy_gazebo/worlds/test_world.world'
        self.assertEqual(len(obstacles(path)), 9)

    def test_new_baffles_are_present_in_saved_map(self):
        root = Path(__file__).resolve().parents[1]
        pgm = (root / 'src/autonomy_navigation/maps/test_map.pgm').read_bytes()
        magic, dimensions, maximum, pixels = pgm.split(b'\n', 3)
        self.assertEqual(magic, b'P5')
        self.assertEqual(maximum, b'255')
        width, height = map(int, dimensions.split())
        for x, y in [(-2.3, 1.8), (0, 2.8)]:
            col = int((x + 5.04) / .05)
            row = height - 1 - int((y + 4.04) / .05)
            self.assertEqual(pixels[row * width + col], 0)


if __name__ == '__main__':
    unittest.main()
