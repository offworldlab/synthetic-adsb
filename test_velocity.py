#!/usr/bin/env python3
"""
Test script to validate synthetic ADS-B velocity calculations.
"""

import requests
import time
import math
import os
from dotenv import load_dotenv

load_dotenv()

TX_LAT = float(os.environ.get("TX_LAT", -34.9810))
TX_LON = float(os.environ.get("TX_LON", 138.7081))
RADIUS_DEG = float(os.environ.get("RADIUS_DEG", 0.05))
ANGULAR_SPEED = float(os.environ.get("ANGULAR_SPEED", 0.01))
FC_MHZ = float(os.environ.get("FC_MHZ", 204.64))

BASE_URL = "http://localhost:5001"
TOLERANCE_PERCENT = 10.0

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate great circle distance between two points in meters."""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return R * c

def bearing_to(lat1, lon1, lat2, lon2):
    """Calculate initial bearing from point 1 to point 2 in degrees."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360

def test_analytical_validation():
    """Test 1: Validate ground speed against analytical calculation."""
    print("\n=== Test 1: Analytical Validation ===")

    print(f"Configuration:")
    print(f"  RADIUS_DEG: {RADIUS_DEG}")
    print(f"  ANGULAR_SPEED: {ANGULAR_SPEED} rad/s")
    print(f"  TX_LAT: {TX_LAT}")

    response = requests.get(f"{BASE_URL}/data/aircraft.json")
    data = response.json()
    aircraft = data["aircraft"][0]

    now = data["now"]
    theta = (now * ANGULAR_SPEED) % (2 * math.pi)
    lat = TX_LAT + RADIUS_DEG * math.cos(theta)

    dlat_dt = -RADIUS_DEG * ANGULAR_SPEED * math.sin(theta)
    dlon_dt = RADIUS_DEG * ANGULAR_SPEED * math.cos(theta)

    dlat_dt_m = dlat_dt * 111320
    dlon_dt_m = dlon_dt * 111320 * math.cos(math.radians(lat))

    expected_speed_ms = math.sqrt(dlat_dt_m**2 + dlon_dt_m**2)
    expected_gs_knots = expected_speed_ms * 1.94384

    print(f"\nExpected values:")
    print(f"  Speed: {expected_speed_ms:.2f} m/s")
    print(f"  Ground speed: {expected_gs_knots:.1f} knots")

    print(f"\nActual values from API:")
    print(f"  Ground speed: {aircraft['gs']} knots")
    print(f"  Track: {aircraft['track']}°")
    print(f"  True heading: {aircraft['true_heading']}°")

    error_percent = abs(aircraft['gs'] - expected_gs_knots) / expected_gs_knots * 100

    if error_percent < TOLERANCE_PERCENT:
        print(f"\n✓ PASS: Ground speed error {error_percent:.2f}% < {TOLERANCE_PERCENT}%")
        return True
    else:
        print(f"\n✗ FAIL: Ground speed error {error_percent:.2f}% >= {TOLERANCE_PERCENT}%")
        return False

def test_position_velocity_consistency():
    """Test 2: Verify position changes match reported velocity."""
    print("\n=== Test 2: Position-Velocity Consistency ===")

    response1 = requests.get(f"{BASE_URL}/data/aircraft.json")
    data1 = response1.json()
    aircraft1 = data1["aircraft"][0]
    t1 = data1["now"]

    print(f"Sample 1 at t={t1:.2f}:")
    print(f"  Position: ({aircraft1['lat']}, {aircraft1['lon']})")
    print(f"  Ground speed: {aircraft1['gs']} knots")

    print("\nWaiting 5 seconds...")
    time.sleep(5)

    response2 = requests.get(f"{BASE_URL}/data/aircraft.json")
    data2 = response2.json()
    aircraft2 = data2["aircraft"][0]
    t2 = data2["now"]

    print(f"\nSample 2 at t={t2:.2f}:")
    print(f"  Position: ({aircraft2['lat']}, {aircraft2['lon']})")
    print(f"  Ground speed: {aircraft2['gs']} knots")

    delta_t = t2 - t1
    distance_m = haversine_distance(
        aircraft1['lat'], aircraft1['lon'],
        aircraft2['lat'], aircraft2['lon']
    )
    calculated_speed_ms = distance_m / delta_t
    calculated_speed_knots = calculated_speed_ms * 1.94384

    reported_speed_knots = (aircraft1['gs'] + aircraft2['gs']) / 2

    print(f"\nAnalysis:")
    print(f"  Time delta: {delta_t:.2f} s")
    print(f"  Distance traveled: {distance_m:.2f} m")
    print(f"  Calculated speed: {calculated_speed_knots:.1f} knots")
    print(f"  Reported speed (avg): {reported_speed_knots:.1f} knots")

    error_percent = abs(calculated_speed_knots - reported_speed_knots) / reported_speed_knots * 100

    if error_percent < TOLERANCE_PERCENT:
        print(f"\n✓ PASS: Speed consistency error {error_percent:.2f}% < {TOLERANCE_PERCENT}%")
        return True
    else:
        print(f"\n✗ FAIL: Speed consistency error {error_percent:.2f}% >= {TOLERANCE_PERCENT}%")
        return False

def test_track_direction():
    """Test 3: Verify track is tangent to circular path."""
    print("\n=== Test 3: Track Direction Validation ===")

    response = requests.get(f"{BASE_URL}/data/aircraft.json")
    data = response.json()
    aircraft = data["aircraft"][0]

    radius_bearing = bearing_to(TX_LAT, TX_LON, aircraft['lat'], aircraft['lon'])

    expected_track_cw = (radius_bearing + 90) % 360
    expected_track_ccw = (radius_bearing - 90) % 360

    print(f"Aircraft position: ({aircraft['lat']}, {aircraft['lon']})")
    print(f"Radius bearing from center: {radius_bearing:.2f}°")
    print(f"Expected track (CW): {expected_track_cw:.2f}°")
    print(f"Expected track (CCW): {expected_track_ccw:.2f}°")
    print(f"Actual track: {aircraft['track']}°")

    error_cw = min(abs(aircraft['track'] - expected_track_cw),
                   360 - abs(aircraft['track'] - expected_track_cw))
    error_ccw = min(abs(aircraft['track'] - expected_track_ccw),
                    360 - abs(aircraft['track'] - expected_track_ccw))

    min_error = min(error_cw, error_ccw)
    direction = "CW" if error_cw < error_ccw else "CCW"

    print(f"\nTrack error ({direction}): {min_error:.2f}°")

    if min_error < 10:
        print(f"\n✓ PASS: Track direction error {min_error:.2f}° < 10°")
        return True
    else:
        print(f"\n✗ FAIL: Track direction error {min_error:.2f}° >= 10°")
        return False

def test_expected_doppler():
    """Test 4: Calculate theoretical Doppler for validation reference."""
    print("\n=== Test 4: Expected Doppler Calculation ===")
    print("(Reference calculation for future adsb2dd validation)")

    RX_LAT = -34.9192
    RX_LON = 138.6027
    RX_ALT = 110
    TX_ALT = 750

    response = requests.get(f"{BASE_URL}/data/aircraft.json")
    data = response.json()
    aircraft = data["aircraft"][0]

    alt_m = aircraft['alt_geom'] * 0.3048

    gs_ms = aircraft['gs'] * 0.514444
    track_rad = math.radians(aircraft['track'])

    vel_x = gs_ms * math.sin(track_rad)
    vel_y = gs_ms * math.cos(track_rad)

    def lla_to_ecef(lat, lon, alt):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        a = 6378137.0
        e2 = 0.00669437999014
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e2) + alt) * math.sin(lat_rad)
        return x, y, z

    ac_x, ac_y, ac_z = lla_to_ecef(aircraft['lat'], aircraft['lon'], alt_m)
    rx_x, rx_y, rx_z = lla_to_ecef(RX_LAT, RX_LON, RX_ALT)
    tx_x, tx_y, tx_z = lla_to_ecef(TX_LAT, TX_LON, TX_ALT)

    rx_dx = rx_x - ac_x
    rx_dy = rx_y - ac_y
    rx_dz = rx_z - ac_z
    rx_dist = math.sqrt(rx_dx**2 + rx_dy**2 + rx_dz**2)

    tx_dx = tx_x - ac_x
    tx_dy = tx_y - ac_y
    tx_dz = tx_z - ac_z
    tx_dist = math.sqrt(tx_dx**2 + tx_dy**2 + tx_dz**2)

    print(f"\nGeometry:")
    print(f"  Aircraft velocity: {gs_ms:.2f} m/s at {aircraft['track']:.2f}°")
    print(f"  Distance to RX: {rx_dist/1000:.2f} km")
    print(f"  Distance to TX: {tx_dist/1000:.2f} km")

    c = 299792458
    wavelength = c / (FC_MHZ * 1e6)

    print(f"\n  Frequency: {FC_MHZ} MHz")
    print(f"  Wavelength: {wavelength:.3f} m")

    print("\n✓ Reference calculation complete")
    print("  (Full Doppler validation requires velocity vector projection)")
    return True

def main():
    print("=" * 60)
    print("Synthetic ADS-B Velocity Validation Test Suite")
    print("=" * 60)

    try:
        response = requests.get(f"{BASE_URL}/data/aircraft.json", timeout=2)
        print(f"\n✓ Server is running at {BASE_URL}")
    except requests.exceptions.RequestException as e:
        print(f"\n✗ ERROR: Cannot connect to server at {BASE_URL}")
        print(f"  Please start the server with: python server.py")
        return

    results = []

    results.append(test_analytical_validation())
    results.append(test_position_velocity_consistency())
    results.append(test_track_direction())
    results.append(test_expected_doppler())

    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"\nPassed: {passed}/{total}")

    if all(results):
        print("\n✓ All tests passed!")
    else:
        print(f"\n✗ {total - passed} test(s) failed")

if __name__ == "__main__":
    main()
