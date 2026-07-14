#!/usr/bin/env python3
"""
Real-time monitor for anomaly generation and detection.
Shows what synthetic-adsb is generating and highlights anomalies.
"""

import time
import requests
import json
from datetime import datetime
import sys


def get_anomaly_info():
    """Get info about generated anomalies from server's aircraft manager."""
    from server import aircraft_manager

    anomaly_info = []
    for aircraft in aircraft_manager.aircraft_list:
        if aircraft.is_anomalous:
            motion_type = type(aircraft.motion_pattern).__name__
            info = {
                'hex': aircraft.icao_hex,
                'flight': aircraft.flight_number.strip(),
                'type': motion_type,
                'has_adsb': aircraft.has_adsb,
                'adsb_accurate': aircraft.adsb_accurate if aircraft.has_adsb else None,
            }
            anomaly_info.append(info)
    return anomaly_info


def monitor_live():
    """Monitor synthetic-adsb and radar detections in real-time."""
    print("=" * 80)
    print("RETINA Anomaly Generation Monitor".center(80))
    print("=" * 80)
    print()

    # Get static anomaly info
    anomaly_info = get_anomaly_info()

    print("CONFIGURED ANOMALIES:")
    print("-" * 80)
    for info in anomaly_info:
        motion_type = info['type'].replace('Motion', '')
        adsb_status = "No ADS-B"
        if info['has_adsb']:
            adsb_status = "Accurate ADS-B" if info['adsb_accurate'] else "Inaccurate ADS-B"

        print(f"  {info['hex']} ({info['flight']}): {motion_type:25s} | {adsb_status}")

    print()
    print("=" * 80)
    print()
    print("Press Ctrl+C to stop monitoring...")
    print()

    iteration = 0
    try:
        while True:
            iteration += 1
            timestamp = datetime.now().strftime("%H:%M:%S")

            # Fetch ADS-B data
            try:
                adsb_resp = requests.get("http://localhost:5001/data/aircraft.json", timeout=2)
                adsb_data = adsb_resp.json()
                adsb_aircraft = adsb_data.get('aircraft', [])
            except Exception as e:
                print(f"[{timestamp}] Error fetching ADS-B: {e}")
                adsb_aircraft = []

            # Fetch radar data
            try:
                radar_resp = requests.get("http://localhost:49158/api/detection", timeout=2)
                radar_data = radar_resp.json()
                radar_count = len(radar_data.get('delay', []))
            except Exception as e:
                print(f"[{timestamp}] Error fetching radar: {e}")
                radar_count = 0

            # Display status
            print(f"\r[{timestamp}] ADS-B: {len(adsb_aircraft):2d} aircraft | Radar: {radar_count:2d} detections", end='', flush=True)

            # Every 10 seconds, show detailed view
            if iteration % 10 == 0:
                print()  # New line
                print("-" * 80)
                print(f"DETAILED VIEW at {timestamp}")
                print("-" * 80)

                if adsb_aircraft:
                    print("\nADS-B FEED:")
                    for ac in adsb_aircraft:
                        velocity_ms = ac['gs'] * 0.514444
                        mach = velocity_ms / 343.0

                        # Highlight anomalies
                        anomaly_marker = ""
                        if mach > 1.0:
                            anomaly_marker = " 🚀 SUPERSONIC"

                        print(f"  {ac['hex']}: {ac['flight']:8s} | {ac['gs']:6.1f} kts (Mach {mach:.2f}) | {ac['track']:6.1f}°{anomaly_marker}")

                if radar_count > 0:
                    print(f"\nRADAR DETECTIONS: {radar_count} targets")
                    print(f"  Delay range: {min(radar_data['delay']):.2f} - {max(radar_data['delay']):.2f} km")
                    print(f"  Doppler range: {min(radar_data['doppler']):.1f} - {max(radar_data['doppler']):.1f} Hz")

                print()

            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")


def show_snapshot():
    """Show a single snapshot of current state with full details."""
    print("\n" + "=" * 80)
    print("RETINA Anomaly Generation Snapshot".center(80))
    print("=" * 80)
    print()

    # Get static anomaly info
    anomaly_info = get_anomaly_info()

    print("CONFIGURED ANOMALIES:")
    print("-" * 80)
    for info in anomaly_info:
        motion_type = info['type'].replace('Motion', '')
        adsb_status = "No ADS-B"
        if info['has_adsb']:
            adsb_status = "✓ Accurate ADS-B" if info['adsb_accurate'] else "⚠ Inaccurate ADS-B"

        print(f"  {info['hex']} ({info['flight']})")
        print(f"    Type: {motion_type}")
        print(f"    ADS-B: {adsb_status}")
        print()

    print("=" * 80)
    print()

    # Fetch current state
    try:
        adsb_resp = requests.get("http://localhost:5001/data/aircraft.json", timeout=2)
        adsb_data = adsb_resp.json()
        adsb_aircraft = adsb_data.get('aircraft', [])

        print("CURRENT ADS-B FEED:")
        print("-" * 80)
        if adsb_aircraft:
            for ac in adsb_aircraft:
                velocity_ms = ac['gs'] * 0.514444
                mach = velocity_ms / 343.0

                print(f"\n{ac['hex']} - {ac['flight'].strip()}")
                print(f"  Position: ({ac['lat']:.6f}, {ac['lon']:.6f}) @ {ac['alt_baro']} ft")
                print(f"  Velocity: {ac['gs']:.1f} knots ({velocity_ms:.1f} m/s, Mach {mach:.2f})")
                print(f"  Heading: {ac['track']:.1f}°")

                if mach > 1.0:
                    print(f"  ⚠️  SUPERSONIC: {mach:.2f}x speed of sound")
        else:
            print("  No aircraft broadcasting ADS-B")

        print()
    except Exception as e:
        print(f"Error fetching ADS-B: {e}")
        print()

    # Fetch radar data
    try:
        radar_resp = requests.get("http://localhost:49158/api/detection", timeout=2)
        radar_data = radar_resp.json()

        print("RADAR DETECTIONS (RX1):")
        print("-" * 80)
        print(f"  Total detections: {len(radar_data['delay'])}")
        print(f"  Timestamp: {radar_data['timestamp']}")

        if radar_data['delay']:
            print(f"\n  Detection Details:")
            for i in range(min(5, len(radar_data['delay']))):
                print(f"    [{i+1}] Delay: {radar_data['delay'][i]:.2f} km | Doppler: {radar_data['doppler'][i]:.1f} Hz | SNR: {radar_data['snr'][i]:.1f} dB")

            if len(radar_data['delay']) > 5:
                print(f"    ... and {len(radar_data['delay']) - 5} more detections")

        print()
    except Exception as e:
        print(f"Error fetching radar: {e}")
        print()

    print("=" * 80)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "snapshot":
        show_snapshot()
    else:
        print("\nStarting live monitor...")
        print("(Use 'python3 monitor_anomalies.py snapshot' for a single detailed snapshot)\n")
        time.sleep(2)
        monitor_live()
