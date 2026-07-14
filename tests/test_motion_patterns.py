#!/usr/bin/env python3
"""
test_motion_patterns.py

Unit tests for motion pattern classes.
"""

import math
import time
import pytest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from motion_patterns import CircularMotion, SupersonicLinearMotion


class TestCircularMotion:
    """Test circular motion pattern."""

    def test_initialization(self):
        motion = CircularMotion(center_lat=-34.0, center_lon=138.0, radius_deg=0.05, angular_speed=0.01)

        assert motion.center_lat == -34.0
        assert motion.center_lon == 138.0
        assert motion.radius_deg == 0.05
        assert motion.angular_speed == 0.01

    def test_position_at_zero(self):
        motion = CircularMotion(center_lat=0.0, center_lon=0.0, radius_deg=1.0, angular_speed=0.01)

        lat, lon = motion.get_position(0.0)

        assert abs(lat - 1.0) < 0.001
        assert abs(lon - 0.0) < 0.001

    def test_position_at_quarter_circle(self):
        motion = CircularMotion(center_lat=0.0, center_lon=0.0, radius_deg=1.0, angular_speed=1.0)

        t_quarter = math.pi / 2
        lat, lon = motion.get_position(t_quarter)

        assert abs(lat - 0.0) < 0.001
        assert abs(lon - 1.0) < 0.001

    def test_velocity_positive(self):
        motion = CircularMotion(center_lat=-34.0, center_lon=138.0, radius_deg=0.05, angular_speed=0.01)

        velocity = motion.get_velocity(0.0)

        assert velocity > 0
        assert isinstance(velocity, float)

    def test_heading_range(self):
        motion = CircularMotion(center_lat=-34.0, center_lon=138.0, radius_deg=0.05, angular_speed=0.01)

        heading = motion.get_heading(0.0)

        assert 0 <= heading < 360


class TestSupersonicLinearMotion:
    """Test supersonic linear motion pattern."""

    def test_initialization(self):
        motion = SupersonicLinearMotion(
            start_lat=-34.0, start_lon=138.0, mach_number=3.0, direction_deg=45.0, start_time=0.0
        )

        assert motion.start_lat == -34.0
        assert motion.start_lon == 138.0
        assert motion.mach_number == 3.0
        assert abs(motion.direction_rad - math.radians(45.0)) < 0.001
        assert motion.start_time == 0.0

    def test_velocity_supersonic(self):
        for mach in [2.0, 3.0, 4.0, 5.0]:
            motion = SupersonicLinearMotion(
                start_lat=-34.0, start_lon=138.0, mach_number=mach, direction_deg=0.0, start_time=0.0
            )

            velocity_knots = motion.get_velocity(0.0)
            velocity_ms = velocity_knots / 1.94384

            expected_ms = mach * 343.0
            assert abs(velocity_ms - expected_ms) < 1.0

    def test_mach_2_velocity(self):
        motion = SupersonicLinearMotion(
            start_lat=-34.0, start_lon=138.0, mach_number=2.0, direction_deg=0.0, start_time=0.0
        )

        velocity_knots = motion.get_velocity(0.0)

        assert velocity_knots > 1300

    def test_mach_5_velocity(self):
        motion = SupersonicLinearMotion(
            start_lat=-34.0, start_lon=138.0, mach_number=5.0, direction_deg=0.0, start_time=0.0
        )

        velocity_knots = motion.get_velocity(0.0)

        assert velocity_knots > 3300

    def test_position_at_start(self):
        motion = SupersonicLinearMotion(
            start_lat=-34.0, start_lon=138.0, mach_number=3.0, direction_deg=0.0, start_time=0.0
        )

        lat, lon = motion.get_position(0.0)

        assert abs(lat - (-34.0)) < 0.0001
        assert abs(lon - 138.0) < 0.0001

    def test_position_moves_north(self):
        motion = SupersonicLinearMotion(
            start_lat=-34.0, start_lon=138.0, mach_number=3.0, direction_deg=0.0, start_time=0.0
        )

        lat_start, lon_start = motion.get_position(0.0)
        lat_later, lon_later = motion.get_position(10.0)

        assert lat_later > lat_start
        assert abs(lon_later - lon_start) < 0.01

    def test_position_moves_east(self):
        motion = SupersonicLinearMotion(
            start_lat=-34.0, start_lon=138.0, mach_number=3.0, direction_deg=90.0, start_time=0.0
        )

        lat_start, lon_start = motion.get_position(0.0)
        lat_later, lon_later = motion.get_position(10.0)

        assert lon_later > lon_start
        assert abs(lat_later - lat_start) < 0.01

    def test_heading_matches_direction(self):
        for direction in [0.0, 45.0, 90.0, 180.0, 270.0]:
            motion = SupersonicLinearMotion(
                start_lat=-34.0, start_lon=138.0, mach_number=3.0, direction_deg=direction, start_time=0.0
            )

            heading = motion.get_heading(0.0)

            assert abs(heading - direction) < 0.01


class TestMotionPatternIntegration:
    """Integration tests for motion patterns."""

    def test_circular_motion_completes_circle(self):
        motion = CircularMotion(center_lat=0.0, center_lon=0.0, radius_deg=1.0, angular_speed=1.0)

        lat_0, lon_0 = motion.get_position(0.0)
        lat_2pi, lon_2pi = motion.get_position(2 * math.pi)

        assert abs(lat_0 - lat_2pi) < 0.001
        assert abs(lon_0 - lon_2pi) < 0.001

    def test_supersonic_linear_distance(self):
        motion = SupersonicLinearMotion(
            start_lat=0.0, start_lon=0.0, mach_number=3.0, direction_deg=0.0, start_time=0.0
        )

        lat_0, lon_0 = motion.get_position(0.0)
        lat_10, lon_10 = motion.get_position(10.0)

        distance_deg = math.sqrt((lat_10 - lat_0) ** 2 + (lon_10 - lon_0) ** 2)

        expected_distance_m = 3.0 * 343.0 * 10.0
        expected_distance_deg = expected_distance_m / 111320

        assert abs(distance_deg - expected_distance_deg) < 0.001


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
