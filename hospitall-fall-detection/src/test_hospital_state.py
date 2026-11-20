#!/usr/bin/env python3

import unittest

from hospital_state import (
    HospitalStateMachine,
    classify_patient_location,
    horizontal_pose,
)


class HospitalStateTests(unittest.TestCase):
    def test_bed_and_floor_are_distinct(self):
        bed = classify_patient_location(
            {"bed": 0.70, "floor": 0.08}, {"bed": 0.88, "floor": 0.0}
        )
        floor = classify_patient_location(
            {"bed": 0.04, "floor": 0.76}, {"bed": 0.0, "floor": 0.90}
        )
        self.assertEqual(bed.location, "ON_BED")
        self.assertEqual(floor.location, "ON_FLOOR")

    def test_horizontal_aspect_ratio(self):
        self.assertTrue(horizontal_pose((0, 0, 200, 100)))
        self.assertFalse(horizontal_pose((0, 0, 80, 180)))

    def test_bed_to_floor_down_generates_alert_after_timer(self):
        machine = HospitalStateMachine(lying_alert_seconds=1.0, transition_window_seconds=8.0)
        self.assertEqual(
            machine.update(1.0, "ON_BED", "lying_down", True).state,
            "PATIENT_ON_BED",
        )
        first_floor = machine.update(3.0, "ON_FLOOR", "lying_down", True)
        self.assertEqual(first_floor.state, "PATIENT_DOWN_ON_FLOOR")
        self.assertFalse(first_floor.alert)
        alert = machine.update(4.1, "ON_FLOOR", "lying_down", True)
        self.assertEqual(alert.state, "FALL_ALERT_ON_FLOOR")
        self.assertTrue(alert.alert)

    def test_floor_lying_without_transition_is_not_a_fall_alert(self):
        machine = HospitalStateMachine(lying_alert_seconds=1.0, transition_window_seconds=8.0)
        machine.update(1.0, "ON_FLOOR", "lying_down", True)
        result = machine.update(3.0, "ON_FLOOR", "lying_down", True)
        self.assertEqual(result.state, "PATIENT_DOWN_ON_FLOOR")
        self.assertFalse(result.alert)


if __name__ == "__main__":
    unittest.main(verbosity=2)
