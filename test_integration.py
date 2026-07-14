#!/usr/bin/env python3
"""
Integration test for synthetic-adsb server with anomalies.
"""

import requests
import json
import time
import subprocess
import os
import signal

def test_integration():
    print("Starting synthetic-adsb server...")

    server_process = subprocess.Popen(
        ["python3", "server.py"],
        cwd="/Users/jonnyspicer/repos/retina/synthetic-adsb",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    time.sleep(5)

    try:
        print("\n=== Testing ADS-B Endpoint ===")
        response = requests.get("http://localhost:5001/data/aircraft.json")
        data = response.json()

        print(f"Response status: {response.status_code}")
        print(f"Number of aircraft: {len(data['aircraft'])}")

        for aircraft in data["aircraft"]:
            print(f"\nAircraft {aircraft['hex']}:")
            print(f"  Flight: {aircraft['flight'].strip()}")
            print(f"  Position: {aircraft['lat']:.6f}, {aircraft['lon']:.6f}")
            print(f"  Altitude: {aircraft['alt_baro']} ft")
            print(f"  Ground Speed: {aircraft['gs']:.1f} knots")
            print(f"  Track: {aircraft['track']:.1f}°")

            if aircraft['gs'] > 660:
                print(f"  ⚠️  SUPERSONIC (Mach {aircraft['gs'] / 661.5:.1f})")

        print("\n=== Testing Radar Detection Endpoint ===")
        response = requests.get("http://localhost:49158/api/detection")
        detection_data = response.json()

        print(f"Response status: {response.status_code}")
        print(f"Number of detections: {len(detection_data['delay'])}")
        print(f"Delays: {detection_data['delay']}")
        print(f"Dopplers: {detection_data['doppler']}")
        print(f"SNRs: {detection_data['snr']}")

        print("\n✅ Integration test passed!")
        return True

    except Exception as e:
        print(f"\n❌ Integration test failed: {e}")
        return False

    finally:
        print("\nStopping server...")
        server_process.terminate()
        server_process.wait(timeout=5)

if __name__ == "__main__":
    success = test_integration()
    exit(0 if success else 1)
