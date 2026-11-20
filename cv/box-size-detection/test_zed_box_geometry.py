#!/usr/bin/env python3

import math
import unittest

import numpy as np

from zed_box_geometry import (
    BoxObservation,
    CuboidEstimate,
    assign_stack_ids,
    component_boxes,
    detect_rectangular_boxes,
    fit_support_plane,
    foreground_mask,
    heights_above_support,
    infer_stack_supports,
    median_depth_in_bbox,
    robust_oriented_cuboid,
    visible_face_dimensions,
)


class GeometryTests(unittest.TestCase):
    def test_floor_plane_and_height_coordinates(self):
        rng = np.random.default_rng(22)
        floor = np.column_stack(
            (
                rng.uniform(-1.0, 1.0, 2500),
                np.full(2500, 0.75) + rng.normal(0.0, 0.001, 2500),
                rng.uniform(0.5, 3.0, 2500),
            )
        )
        plane = fit_support_plane(floor, threshold_m=0.006)
        self.assertIsNotNone(plane)
        self.assertAlmostEqual(plane.camera_height_m, 0.75, delta=0.01)
        box_face = np.column_stack(
            (
                rng.uniform(-0.2, 0.2, 1000),
                rng.uniform(0.45, 0.75, 1000),
                np.full(1000, 1.2),
            )
        )
        bottom, top = heights_above_support(box_face, plane)
        self.assertAlmostEqual(bottom, 0.0, delta=0.015)
        self.assertAlmostEqual(top, 0.30, delta=0.015)

    def test_stack_graph_from_geometry(self):
        estimate = CuboidEstimate(0.4, 0.2, math.nan, False, 0.0, 1.0, 500)
        lower = BoxObservation(
            1, (90, 200, 310, 320), 0.4, 0.2, math.nan, 1.0, 1.0,
            0.0, 0.20, estimate,
        )
        upper = BoxObservation(
            2, (100, 100, 300, 202), 0.4, 0.2, math.nan, 1.02, 1.0,
            0.20, 0.40, estimate,
        )
        side = BoxObservation(
            3, (350, 210, 480, 320), 0.3, 0.2, math.nan, 1.0, 1.0,
            0.0, 0.18, estimate,
        )
        supports = infer_stack_supports([lower, upper, side])
        self.assertEqual(supports, {2: 1})
        self.assertEqual(assign_stack_ids([lower, upper, side], supports), {1: 1, 2: 1, 3: 3})

    def test_automatic_rectangle_excludes_short_attached_clutter(self):
        frame = np.full((320, 520, 3), 210, dtype=np.uint8)
        cv2_colour = (75, 90, 105)
        import cv2
        cv2.rectangle(frame, (160, 55), (455, 255), cv2_colour, -1)
        cv2.rectangle(frame, (95, 165), (160, 255), cv2_colour, -1)
        cv2.rectangle(frame, (160, 55), (455, 255), (20, 20, 20), 3)
        boxes = detect_rectangular_boxes(frame, min_area_px=3000, max_boxes=1)
        self.assertEqual(len(boxes), 1)
        x1, y1, x2, y2 = boxes[0]
        self.assertLessEqual(abs(x1 - 160), 8)
        self.assertLessEqual(abs(y1 - 55), 8)
        self.assertLessEqual(abs(x2 - 456), 8)
        self.assertLessEqual(abs(y2 - 256), 8)

    def test_automatic_detector_separates_two_stacked_boxes(self):
        import cv2
        frame = np.full((500, 700, 3), 220, dtype=np.uint8)
        cv2.rectangle(frame, (180, 80), (520, 250), (90, 110, 130), -1)
        cv2.rectangle(frame, (180, 80), (520, 250), (20, 20, 20), 4)
        cv2.rectangle(frame, (160, 250), (540, 440), (75, 95, 115), -1)
        cv2.rectangle(frame, (160, 250), (540, 440), (20, 20, 20), 4)
        boxes = detect_rectangular_boxes(frame, min_area_px=5000, max_boxes=8)
        self.assertEqual(len(boxes), 2)
        boxes.sort(key=lambda box: box[1])
        self.assertLessEqual(abs(boxes[0][1] - 80), 8)
        self.assertLessEqual(abs(boxes[0][3] - 251), 8)
        self.assertLessEqual(abs(boxes[1][1] - 250), 8)
        self.assertLessEqual(abs(boxes[1][3] - 441), 8)

    def test_calibrated_bbox_dimensions_and_centre_depth(self):
        depth = np.full((300, 500), 2.0, dtype=np.float32)
        depth[130:170, 220:230] = np.nan
        measured_depth, valid_fraction = median_depth_in_bbox(
            depth, (100, 50, 400, 250), 0.2, 8.0
        )
        width, height = visible_face_dimensions((100, 50, 400, 250), measured_depth, 700.0, 700.0)
        self.assertAlmostEqual(measured_depth, 2.0)
        self.assertGreater(valid_fraction, 0.9)
        self.assertAlmostEqual(width, 300 * 2.0 / 700.0)
        self.assertAlmostEqual(height, 200 * 2.0 / 700.0)

    def test_foreground_component_becomes_bbox(self):
        background = np.full((120, 160), 3.0, dtype=np.float32)
        current = background.copy()
        current[30:90, 50:120] = 2.0
        mask = foreground_mask(background, current, 0.06, 0.2, 8.0, kernel_size=3)
        components = component_boxes(mask, min_area_px=500)
        self.assertEqual(len(components), 1)
        x1, y1, x2, y2 = components[0][0]
        self.assertLessEqual(abs(x1 - 50), 2)
        self.assertLessEqual(abs(y1 - 30), 2)
        self.assertLessEqual(abs(x2 - 120), 2)
        self.assertLessEqual(abs(y2 - 90), 2)

    def test_rotated_cuboid_extents(self):
        rng = np.random.default_rng(8)
        dimensions = np.array([0.80, 0.50, 0.30])
        points = rng.uniform(-0.5, 0.5, size=(22000, 3)) * dimensions
        angle_y = np.deg2rad(27.0)
        angle_z = np.deg2rad(18.0)
        rotate_y = np.array(
            [[np.cos(angle_y), 0.0, np.sin(angle_y)], [0.0, 1.0, 0.0], [-np.sin(angle_y), 0.0, np.cos(angle_y)]]
        )
        rotate_z = np.array(
            [[np.cos(angle_z), -np.sin(angle_z), 0.0], [np.sin(angle_z), np.cos(angle_z), 0.0], [0.0, 0.0, 1.0]]
        )
        points = points @ (rotate_z @ rotate_y).T + np.array([0.2, -0.1, 2.2])
        points += rng.normal(0.0, 0.0008, size=points.shape)
        estimate = robust_oriented_cuboid(points)
        self.assertAlmostEqual(estimate.length_m, 0.80, delta=0.045)
        self.assertAlmostEqual(estimate.breadth_m, 0.50, delta=0.045)
        self.assertAlmostEqual(estimate.depth_m, 0.30, delta=0.045)
        self.assertTrue(estimate.third_dimension_reliable)

    def test_planar_face_does_not_invent_depth(self):
        u, v = np.meshgrid(np.linspace(-0.4, 0.4, 80), np.linspace(-0.25, 0.25, 60))
        points = np.column_stack((u.ravel(), v.ravel(), np.full(u.size, 2.0)))
        estimate = robust_oriented_cuboid(points)
        self.assertFalse(estimate.third_dimension_reliable)
        self.assertTrue(math.isnan(estimate.depth_m))
        self.assertAlmostEqual(estimate.length_m, 0.8, delta=0.03)
        self.assertAlmostEqual(estimate.breadth_m, 0.5, delta=0.03)


if __name__ == "__main__":
    unittest.main(verbosity=2)
