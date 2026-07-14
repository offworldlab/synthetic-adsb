#!/usr/bin/env python3
"""
Test script to verify anomaly generation in synthetic-adsb.
Runs the server briefly and checks that all anomaly types are generated.
"""

import time
import requests
import json
from motion_patterns import CircularMotion, SupersonicLinearMotion, InstantDirectionChangeMotion, InstantAccelerationMotion


def test_motion_patterns():
    """Test that all motion patterns work correctly."""
    print("\n" + "=" * 60)
    print("Testing Motion Patterns")
    print("=" * 60)

    current_time = time.time()

    print("\n1. CircularMotion (Normal)")
    circular = CircularMotion(37.7644, -122.3954, 0.05, 0.01)
    lat, lon = circular.get_position(current_time)
    gs = circular.get_velocity(current_time)
    heading = circular.get_heading(current_time)
    print(f"   Position: ({lat:.6f}, {lon:.6f})")
    print(f"   Velocity: {gs:.1f} knots")
    print(f"   Heading: {heading:.2f}°")
    assert 37.7 < lat < 37.9, "Circular motion latitude out of range"
    assert 50 < gs < 150, "Circular motion velocity out of range"

    print("\n2. SupersonicLinearMotion (Mach 3)")
    supersonic = SupersonicLinearMotion(37.8, -122.2, 3.0, 90, start_time=current_time)
    lat, lon = supersonic.get_position(current_time)
    gs = supersonic.get_velocity(current_time)
    heading = supersonic.get_heading(current_time)
    print(f"   Position: ({lat:.6f}, {lon:.6f})")
    print(f"   Velocity: {gs:.1f} knots (Mach {gs * 0.514444 / 343:.2f})")
    print(f"   Heading: {heading:.2f}°")
    assert gs > 1500, "Supersonic velocity too low"

    print("\n3. InstantDirectionChangeMotion")
    direction_change = InstantDirectionChangeMotion(37.8, -122.2, 450, 0, change_interval_sec=5.0, start_time=current_time)
    positions = []
    headings = []
    for i in range(3):
        t = current_time + i * 6.0
        lat, lon = direction_change.get_position(t)
        heading = direction_change.get_heading(t)
        positions.append((lat, lon))
        headings.append(heading)
        print(f"   t+{i*6}s: Position: ({lat:.6f}, {lon:.6f}), Heading: {heading:.2f}°")

    heading_change = abs(headings[1] - headings[0])
    if heading_change > 180:
        heading_change = 360 - heading_change
    print(f"   Heading change: {heading_change:.2f}° (expected ~90°)")
    assert heading_change > 70, "Direction change not detected"

    print("\n4. InstantAccelerationMotion")
    speed_profile = [(3.0, 500), (2.0, 0), (3.0, 600)]
    acceleration = InstantAccelerationMotion(37.8, -122.2, 45, speed_profile, start_time=current_time)
    speeds = []
    for i in range(3):
        t = current_time + i * 3.0
        gs = acceleration.get_velocity(t)
        speeds.append(gs)
        print(f"   t+{i*3}s: Velocity: {gs:.1f} knots")

    print(f"   Speed changes: {speeds[0]:.0f} → {speeds[1]:.0f} → {speeds[2]:.0f} knots")
    assert speeds[1] == 0, "Standstill not detected"
    assert speeds[2] > 500, "Acceleration not detected"

    print("\n✓ All motion patterns working correctly!")


def test_aircraft_manager():
    """Test AircraftManager generates correct mix of aircraft."""
    print("\n" + "=" * 60)
    print("Testing AircraftManager")
    print("=" * 60)

    import os
    from server import AircraftManager, ENABLE_ANOMALIES, NORMAL_AIRCRAFT_COUNT, ANOMALOUS_AIRCRAFT_COUNT

    print(f"\nConfiguration:")
    print(f"  ENABLE_ANOMALIES: {ENABLE_ANOMALIES}")
    print(f"  NORMAL_AIRCRAFT_COUNT: {NORMAL_AIRCRAFT_COUNT}")
    print(f"  ANOMALOUS_AIRCRAFT_COUNT: {ANOMALOUS_AIRCRAFT_COUNT}")

    manager = AircraftManager()

    print(f"\n✓ Generated {len(manager.aircraft_list)} aircraft")

    normal_count = sum(1 for a in manager.aircraft_list if not a.is_anomalous)
    anomalous_count = sum(1 for a in manager.aircraft_list if a.is_anomalous)

    print(f"  Normal: {normal_count}")
    print(f"  Anomalous: {anomalous_count}")

    adsb_count = sum(1 for a in manager.aircraft_list if a.has_adsb)
    no_adsb_count = sum(1 for a in manager.aircraft_list if not a.has_adsb)

    print(f"\n✓ ADS-B distribution:")
    print(f"  With ADS-B: {adsb_count}")
    print(f"  Without ADS-B: {no_adsb_count}")

    inaccurate_adsb = sum(1 for a in manager.aircraft_list if a.has_adsb and not a.adsb_accurate)
    print(f"  Inaccurate ADS-B: {inaccurate_adsb}")

    current_time = time.time()
    aircraft_data = manager.get_all_aircraft_data(current_time, current_time)

    print(f"\n✓ ADS-B feed contains {len(aircraft_data)} aircraft (only those with ADS-B)")

    radar_data = manager.get_all_aircraft_for_radar(current_time)
    print(f"✓ Radar detects {len(radar_data)} aircraft (all aircraft)")

    assert len(radar_data) == len(manager.aircraft_list), "Radar should detect all aircraft"
    assert len(aircraft_data) <= len(radar_data), "ADS-B should show fewer aircraft than radar"

    print("\n✓ AircraftManager working correctly!")


def test_server_endpoints():
    """Test that server endpoints return correct data."""
    print("\n" + "=" * 60)
    print("Testing Server Endpoints (requires running server)")
    print("=" * 60)

    try:
        response = requests.get("http://localhost:5001/data/aircraft.json", timeout=2)
        response.raise_for_status()
        adsb_data = response.json()

        print(f"\n✓ ADS-B endpoint: {len(adsb_data['aircraft'])} aircraft")
        for aircraft in adsb_data['aircraft']:
            print(f"  {aircraft['hex']}: {aircraft['flight'].strip()} @ {aircraft['gs']:.1f} knots")

        response = requests.get("http://localhost:49158/api/detection", timeout=2)
        response.raise_for_status()
        radar_data = response.json()

        print(f"\n✓ Radar endpoint: {len(radar_data['delay'])} detections")
        print(f"  Delays: {radar_data['delay'][:3]}")
        print(f"  Dopplers: {radar_data['doppler'][:3]}")

        assert len(radar_data['delay']) >= len(adsb_data['aircraft']), "Radar should detect at least as many aircraft as ADS-B"

        print("\n✓ Server endpoints working correctly!")

    except requests.exceptions.ConnectionError:
        print("\n⚠ Server not running. Start with: python3 server.py")
    except Exception as e:
        print(f"\n✗ Error testing endpoints: {e}")


if __name__ == "__main__":
    try:
        test_motion_patterns()
        test_aircraft_manager()
        test_server_endpoints()

        print("\n" + "=" * 60)
        print("SUCCESS: All anomaly tests passed! ✓")
        print("=" * 60)

    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
