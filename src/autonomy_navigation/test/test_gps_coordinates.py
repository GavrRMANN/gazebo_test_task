import unittest

from gps_coordinates import GpsProjection, ecef


class GpsCoordinatesTest(unittest.TestCase):
    def test_reference_is_world_origin(self):
        projection = GpsProjection(55.0, 37.0, 100.0)
        self.assertEqual(projection.to_world(55.0, 37.0), (0.0, 0.0))

    def test_measured_gazebo_start_antenna_position(self):
        projection = GpsProjection(55.0, 37.0, 100.0)
        x, y = projection.to_world(
            55.00002694799619, 37.0000640656013, 100.17498701717705
        )
        self.assertAlmostEqual(x, -4.1, delta=0.002)
        self.assertAlmostEqual(y, -3.0, delta=0.002)

    def test_heading_rotates_world_axes(self):
        fix = (55.00002694799619, 37.0000640656013, 100.17498701717705)
        x, y = GpsProjection(55.0, 37.0, 100.0).to_world(*fix)
        rotated_x, rotated_y = GpsProjection(55.0, 37.0, 100.0, 90).to_world(*fix)
        self.assertAlmostEqual(rotated_x, -y, places=7)
        self.assertAlmostEqual(rotated_y, x, places=7)

    def test_wgs84_equator(self):
        x, y, z = ecef(0.0, 0.0, 100.0)
        self.assertEqual(x, 6378237.0)
        self.assertEqual(y, 0.0)
        self.assertEqual(z, 0.0)


if __name__ == '__main__':
    unittest.main()
