#!/usr/bin/env python3
"""
Simple test - import server and check aircraft count.
"""

import sys
import os

os.chdir("/Users/jonnyspicer/repos/retina/synthetic-adsb")

from server import aircraft_manager, ENABLE_ANOMALIES, NORMAL_AIRCRAFT_COUNT, ANOMALOUS_AIRCRAFT_COUNT

print("Configuration:")
print(f"  ENABLE_ANOMALIES: {ENABLE_ANOMALIES}")
print(f"  NORMAL_AIRCRAFT_COUNT: {NORMAL_AIRCRAFT_COUNT}")
print(f"  ANOMALOUS_AIRCRAFT_COUNT: {ANOMALOUS_AIRCRAFT_COUNT}")

print(f"\nAircraft Manager:")
print(f"  Total aircraft: {len(aircraft_manager.aircraft_list)}")

for i, aircraft in enumerate(aircraft_manager.aircraft_list):
    print(f"\n  Aircraft {i+1}:")
    print(f"    ICAO: {aircraft.icao_hex}")
    print(f"    Flight: {aircraft.flight_number.strip()}")
    print(f"    Altitude: {aircraft.altitude_ft} ft")
    print(f"    Anomalous: {aircraft.is_anomalous}")
    print(f"    Motion: {type(aircraft.motion_pattern).__name__}")

if len(aircraft_manager.aircraft_list) == 2:
    print("\n✅ SUCCESS: 2 aircraft initialized correctly")
    sys.exit(0)
else:
    print(f"\n❌ FAILED: Expected 2 aircraft, got {len(aircraft_manager.aircraft_list)}")
    sys.exit(1)
