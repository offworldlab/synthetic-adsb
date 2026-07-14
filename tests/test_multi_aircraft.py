#!/usr/bin/env python3
"""
test_multi_aircraft.py

Unit tests for multi-aircraft support in server.py.
"""

import pytest
import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

os.environ["TX_LAT"] = "37.7644"
os.environ["TX_LON"] = "-122.3954"
os.environ["TX_ALT"] = "23"
os.environ["FC_MHZ"] = "204.64"
os.environ["RADIUS_DEG"] = "0.05"
os.environ["ANGULAR_SPEED"] = "0.01"
os.environ["ALT_BARO_FT"] = "30000"
os.environ["ICAO_HEX"] = "AEF123"
os.environ["HOST"] = "0.0.0.0"
os.environ["PORT"] = "5001"
os.environ["RADARS"] = '[{"id": "rx1", "lat": -34.9192, "lon": 138.6027, "alt": 110, "port": 49158}]'
os.environ["ENABLE_ANOMALIES"] = "true"
os.environ["NORMAL_AIRCRAFT_COUNT"] = "1"
os.environ["ANOMALOUS_AIRCRAFT_COUNT"] = "1"
os.environ["SUPERSONIC_MACH_MIN"] = "2.0"
os.environ["SUPERSONIC_MACH_MAX"] = "5.0"
os.environ["SUPERSONIC_ALTITUDE_FT"] = "50000"

from server import Aircraft, AircraftManager
from motion_patterns import CircularMotion, SupersonicLinearMotion


class TestAircraft:
    """Test Aircraft class."""

    def test_aircraft_initialization(self):
        motion = CircularMotion(37.7644, -122.3954, 0.05, 0.01)
        aircraft = Aircraft(
            icao_hex="AEF123", motion_pattern=motion, altitude_ft=30000, flight_number="SYN001  ", is_anomalous=False
        )

        assert aircraft.icao_hex == "AEF123"
        assert aircraft.altitude_ft == 30000
        assert aircraft.flight_number == "SYN001  "
        assert aircraft.is_anomalous is False

    def test_aircraft_get_data(self):
        motion = CircularMotion(37.7644, -122.3954, 0.05, 0.01)
        aircraft = Aircraft(
            icao_hex="AEF123", motion_pattern=motion, altitude_ft=30000, flight_number="SYN001  ", is_anomalous=False
        )

        now = time.time()
        data = aircraft.get_data(now, now)

        assert data["hex"] == "AEF123"
        assert "lat" in data
        assert "lon" in data
        assert data["alt_baro"] == 30000
        assert data["alt_geom"] == 30100
        assert data["gs"] > 0
        assert 0 <= data["track"] < 360
        assert data["flight"] == "SYN001  "
        assert data["seen_pos"] == 0


class TestAircraftManager:
    """Test AircraftManager class."""

    def test_aircraft_manager_initialization(self):
        manager = AircraftManager()

        assert len(manager.aircraft_list) == 2

    def test_icao_hex_uniqueness(self):
        manager = AircraftManager()

        icao_hexes = [aircraft.icao_hex for aircraft in manager.aircraft_list]

        assert len(icao_hexes) == len(set(icao_hexes))

    def test_normal_aircraft_count(self):
        manager = AircraftManager()

        normal_aircraft = [aircraft for aircraft in manager.aircraft_list if not aircraft.is_anomalous]

        assert len(normal_aircraft) == 1

    def test_anomalous_aircraft_count(self):
        manager = AircraftManager()

        anomalous_aircraft = [aircraft for aircraft in manager.aircraft_list if aircraft.is_anomalous]

        assert len(anomalous_aircraft) == 1

    def test_anomalous_aircraft_supersonic(self):
        manager = AircraftManager()

        anomalous_aircraft = [aircraft for aircraft in manager.aircraft_list if aircraft.is_anomalous]

        for aircraft in anomalous_aircraft:
            assert isinstance(aircraft.motion_pattern, SupersonicLinearMotion)
            velocity_knots = aircraft.motion_pattern.get_velocity(time.time())
            assert velocity_knots > 1300

    def test_normal_aircraft_circular(self):
        manager = AircraftManager()

        normal_aircraft = [aircraft for aircraft in manager.aircraft_list if not aircraft.is_anomalous]

        for aircraft in normal_aircraft:
            assert isinstance(aircraft.motion_pattern, CircularMotion)

    def test_get_all_aircraft_data(self):
        manager = AircraftManager()

        now = time.time()
        all_data = manager.get_all_aircraft_data(now, now)

        assert len(all_data) == 2
        assert all(isinstance(data, dict) for data in all_data)
        assert all("hex" in data for data in all_data)
        assert all("lat" in data for data in all_data)
        assert all("lon" in data for data in all_data)

    def test_flight_numbers_unique(self):
        manager = AircraftManager()

        flight_numbers = [aircraft.flight_number for aircraft in manager.aircraft_list]

        assert len(flight_numbers) == len(set(flight_numbers))

    def test_anomalous_aircraft_altitude(self):
        manager = AircraftManager()

        anomalous_aircraft = [aircraft for aircraft in manager.aircraft_list if aircraft.is_anomalous]

        for aircraft in anomalous_aircraft:
            assert aircraft.altitude_ft == 50000




if __name__ == "__main__":
    pytest.main([__file__, "-v"])
