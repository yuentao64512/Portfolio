#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

import numpy as np

from hospital_context import (
    ContextConfig,
    ContextGeometry,
    PolygonRegion,
    load_context_config,
    rectangle_overlap_fraction,
    save_context_config,
)


class ContextTests(unittest.TestCase):
    def setUp(self):
        self.config = ContextConfig(
            1,
            "ward_cam_1",
            1000,
            500,
            [
                PolygonRegion("bed_1", "bed", [[0.1, 0.2], [0.5, 0.2], [0.5, 0.8], [0.1, 0.8]]),
                PolygonRegion("floor_1", "floor", [[0.0, 0.8], [1.0, 0.8], [1.0, 1.0], [0.0, 1.0]]),
            ],
        )

    def test_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "context.json"
            save_context_config(self.config, path)
            self.assertEqual(load_context_config(path), self.config)

    def test_overlap_scales_with_resolution(self):
        geometry = ContextGeometry(self.config)
        first = geometry.bbox_overlaps((100, 100, 500, 400), 1000, 500)
        second = geometry.bbox_overlaps((200, 200, 1000, 800), 2000, 1000)
        self.assertAlmostEqual(first["bed"], 1.0, places=2)
        self.assertAlmostEqual(second["bed"], 1.0, places=2)
        partial = geometry.bbox_overlaps((400, 300, 600, 450), 1000, 500)
        self.assertGreater(partial["bed"], 0.0)
        self.assertGreater(partial["floor"], 0.0)

    def test_rectangle_overlap_is_person_fraction(self):
        self.assertAlmostEqual(rectangle_overlap_fraction((0, 0, 100, 100), (50, 0, 150, 100)), 0.5)

    def test_keypoint_region_fractions_ignore_face_and_wrists(self):
        geometry = ContextGeometry(self.config)
        points = np.zeros((17, 2), dtype=np.float32)
        confidence = np.ones(17, dtype=np.float32)
        # Shoulders, hips, knees and ankles all lie in bed_1.
        points[list((5, 6, 11, 12, 13, 14, 15, 16))] = (250, 250)
        # A face and both wrists lie on the floor but are excluded by default.
        points[list((0, 9, 10))] = (500, 450)
        fractions = geometry.keypoint_region_fractions(points, confidence, 1000, 500)
        self.assertAlmostEqual(fractions["bed"], 1.0)
        self.assertAlmostEqual(fractions["floor"], 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
