#!/usr/bin/env python3

import math
import unittest

import numpy as np

from zed_box_cuboid_scene import (
    Cuboid,
    Measurement,
    Plane,
    fit_support_aligned_cuboid,
    fit_overhead_support_plane,
    fit_wall_plane,
    infer_stacks,
    observed_wall_gap,
    overhead_height_tiers,
    stack_roots,
    visible_stack_counts,
)
from zed_box_geometry import SupportPlane


class CuboidSceneTests(unittest.TestCase):
    def setUp(self):
        self.support = SupportPlane(
            normal=np.array([0.0, -1.0, 0.0]),
            offset=0.8,
            inlier_ratio=1.0,
            camera_height_m=0.8,
        )

    def test_support_aligned_cuboid(self):
        rng = np.random.default_rng(40)
        # Visible points on top, front and side of a 0.60 x 0.35 x 0.25 m box.
        count = 5000
        x = rng.uniform(-0.30, 0.30, count)
        z = rng.uniform(1.00, 1.35, count)
        top = np.column_stack((x, np.full(count, 0.55), z))
        y = rng.uniform(0.55, 0.80, count)
        front = np.column_stack((x, y, np.full(count, 1.00)))
        side = np.column_stack((np.full(count, 0.30), y, z))
        cuboid = fit_support_aligned_cuboid(np.vstack((top, front, side)), self.support)
        self.assertIsNotNone(cuboid)
        self.assertAlmostEqual(cuboid.length_m, 0.60, delta=0.025)
        self.assertAlmostEqual(cuboid.breadth_m, 0.35, delta=0.025)
        self.assertAlmostEqual(cuboid.height_m, 0.25, delta=0.025)
        self.assertTrue(cuboid.reliable_3d)

    def test_wall_plane_and_gap(self):
        rng = np.random.default_rng(41)
        wall_points = np.column_stack(
            (rng.uniform(-1, 1, 3000), rng.uniform(-0.2, 0.8, 3000), np.full(3000, 2.0))
        )
        wall = fit_wall_plane(wall_points, self.support.normal, threshold_m=0.004)
        self.assertIsNotNone(wall)
        box_points = np.column_stack(
            (rng.uniform(-0.3, 0.3, 2000), rng.uniform(0.55, 0.8, 2000), rng.uniform(1.2, 1.5, 2000))
        )
        gap = observed_wall_gap(box_points, wall, 1.0)
        self.assertAlmostEqual(gap, 0.50, delta=0.02)

    def test_stack_relation(self):
        def measurement(track_id, bbox, bottom, top):
            return Measurement(
                track_id, bbox, 0.4, 0.3, top - bottom,
                bottom, top, 1.0, 0.5, 0.2, True, 1000,
            )
        lower = measurement(1, (100, 200, 350, 400), 0.0, 0.20)
        upper = measurement(2, (110, 80, 340, 202), 0.20, 0.38)
        side = measurement(3, (400, 220, 600, 400), 0.0, 0.18)
        self.assertEqual(infer_stacks([lower, upper, side]), {2: 1})

    def test_overhead_height_uses_floor_reference(self):
        rng = np.random.default_rng(42)
        points = np.column_stack(
            (
                rng.uniform(-0.30, 0.30, 3000),
                np.full(3000, 0.55),
                rng.uniform(1.00, 1.35, 3000),
            )
        )
        cuboid = fit_support_aligned_cuboid(
            points, self.support, height_bounds_m=(0.0, 0.25)
        )
        self.assertIsNotNone(cuboid)
        self.assertAlmostEqual(cuboid.height_m, 0.25, delta=0.005)
        self.assertTrue(cuboid.reliable_3d)

    def test_overhead_support_plane_uses_known_camera_height(self):
        rng = np.random.default_rng(44)
        floor = np.column_stack(
            (
                rng.uniform(-0.5, 0.5, 6000),
                rng.uniform(-0.3, 0.3, 6000),
                rng.normal(0.49, 0.0015, 6000),
            )
        )
        near_rail = np.column_stack(
            (
                rng.uniform(-0.5, 0.5, 2500),
                rng.uniform(-0.05, 0.05, 2500),
                rng.normal(0.19, 0.001, 2500),
            )
        )
        plane = fit_overhead_support_plane(
            np.vstack((floor, near_rail)), expected_distance_m=0.49
        )
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane.camera_height_m, 0.49, delta=0.008)
        self.assertGreater(abs(float(plane.normal[2])), 0.98)

    def test_overhead_visible_stack_tiers(self):
        height, width = 200, 200
        xyz = np.full((height, width, 4), np.nan, dtype=np.float32)
        component = np.zeros((height, width), dtype=bool)
        component[20:180, 20:180] = True
        yy, xx = np.mgrid[0:height, 0:width]
        xyz[..., 0] = (xx - width / 2) * 0.002
        xyz[..., 2] = 1.0 + yy * 0.002
        xyz[..., 1] = 0.70  # 10 cm over the floor
        xyz[70:140, 70:140, 1] = 0.58  # 22 cm over the floor
        tiers = overhead_height_tiers(component, xyz, self.support, 1.0, 400)
        self.assertEqual(len(tiers), 2)
        self.assertAlmostEqual(tiers[0][1][1], 0.10, delta=0.015)
        self.assertAlmostEqual(tiers[1][1][0], tiers[0][1][1], delta=1e-6)
        self.assertAlmostEqual(tiers[1][1][1], 0.22, delta=0.015)

    def test_visible_stack_count(self):
        def measurement(track_id):
            return Measurement(
                track_id, (0, 0, 100, 100), 0.4, 0.3, 0.1,
                0.0, 0.1, 1.0, math.nan, 0.2, True, 1000,
            )
        measurements = [measurement(1), measurement(2), measurement(3), measurement(4)]
        roots = stack_roots(measurements, {2: 1, 3: 2})
        self.assertEqual(visible_stack_counts(roots), {1: 3, 4: 1})


if __name__ == "__main__":
    unittest.main(verbosity=2)
