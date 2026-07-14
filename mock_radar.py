#!/usr/bin/env python3
"""
mock_radar.py

Creates mock radar detection endpoints that convert ADS-B aircraft positions 
into realistic radar detections (delay and Doppler) for RETINASolver testing.

This creates three radar receivers at different locations around Adelaide
and calculates bistatic range and Doppler shift for the synthetic aircraft.
"""

import math
import time
import requests
import json
from flask import Flask, jsonify
from threading import Thread
import logging

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)

# Radar receiver locations - San Francisco (matching blah2-arm config)
RADARS = {
    "49158": {
        "id": "rx1",
        "lat": 37.7644,
        "lon": -122.3954,
        "alt": 23,
        "frequency": 503000000  # 503 MHz (VHF-TV)
    },
    "49159": {
        "id": "rx2",
        "lat": 37.7644,
        "lon": -122.3954,
        "alt": 23,
        "frequency": 503000000
    },
    "49160": {
        "id": "rx3",
        "lat": 37.7644,
        "lon": -122.3954,
        "alt": 23,
        "frequency": 503000000
    }
}

# Transmitter location - San Francisco (KSCZ-LD)
TRANSMITTER = {
    "lat": 37.49917,
    "lon": -121.87222,
    "alt": 783
}

# Constants
SPEED_OF_LIGHT = 299792458  # m/s
EARTH_RADIUS = 6371000  # meters


def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two points on Earth in meters."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2)**2 + 
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * 
         math.sin(dlon/2)**2)
    c = 2 * math.asin(math.sqrt(a))
    return EARTH_RADIUS * c


def calculate_doppler(aircraft_lat, aircraft_lon, aircraft_alt, prev_lat, prev_lon, 
                     rx_lat, rx_lon, rx_alt, tx_lat, tx_lon, tx_alt, frequency, dt=1.0):
    """Calculate Doppler shift for bistatic radar."""
    
    # Calculate current distances
    tx_to_aircraft = haversine_distance(tx_lat, tx_lon, aircraft_lat, aircraft_lon)
    aircraft_to_rx = haversine_distance(aircraft_lat, aircraft_lon, rx_lat, rx_lon)
    current_bistatic_range = tx_to_aircraft + aircraft_to_rx
    
    # Calculate previous distances if we have previous position
    if prev_lat is not None and prev_lon is not None:
        prev_tx_to_aircraft = haversine_distance(tx_lat, tx_lon, prev_lat, prev_lon)
        prev_aircraft_to_rx = haversine_distance(prev_lat, prev_lon, rx_lat, rx_lon)
        prev_bistatic_range = prev_tx_to_aircraft + prev_aircraft_to_rx
        
        # Range rate (m/s)
        range_rate = (current_bistatic_range - prev_bistatic_range) / dt
        
        # Doppler shift (Hz)
        doppler = -(2 * frequency * range_rate) / SPEED_OF_LIGHT
    else:
        # No previous position, assume small random Doppler
        doppler = (hash(f"{aircraft_lat}{aircraft_lon}{rx_lat}") % 200) - 100
    
    return doppler, current_bistatic_range


def get_aircraft_data():
    """Fetch current aircraft data from the ADS-B endpoint."""
    try:
        response = requests.get("http://localhost:5001/data/aircraft.json", timeout=2)
        if response.status_code == 200:
            data = response.json()
            if data.get("aircraft"):
                return data["aircraft"][0]  # Return first aircraft
    except Exception as e:
        logging.warning(f"Failed to get aircraft data: {e}")
    return None


# Store previous aircraft position for Doppler calculation
previous_aircraft = {}


def create_radar_app(port):
    """Create a Flask app for a specific radar."""
    radar = RADARS[port]
    radar_app = Flask(f"radar_{port}")
    
    @radar_app.route('/api/detection')
    def get_detection():
        """Return radar detection data."""
        global previous_aircraft
        
        aircraft = get_aircraft_data()
        if not aircraft:
            return jsonify([])
        
        # Get previous position for this radar
        prev_key = f"{port}_prev"
        prev_aircraft = previous_aircraft.get(prev_key)
        
        # Calculate bistatic range and Doppler
        prev_lat = prev_aircraft.get("lat") if prev_aircraft else None
        prev_lon = prev_aircraft.get("lon") if prev_aircraft else None
        
        doppler, bistatic_range = calculate_doppler(
            aircraft["lat"], aircraft["lon"], aircraft.get("alt_geom", 30000),
            prev_lat, prev_lon,
            radar["lat"], radar["lon"], radar["alt"],
            TRANSMITTER["lat"], TRANSMITTER["lon"], TRANSMITTER["alt"],
            radar["frequency"]
        )
        
        # Store current as previous for next calculation
        previous_aircraft[prev_key] = {
            "lat": aircraft["lat"], 
            "lon": aircraft["lon"],
            "timestamp": time.time()
        }
        
        # Convert bistatic range to delay (km to microseconds * speed of light)
        delay_km = bistatic_range / 1000.0  # Convert to km
        
        detection = [{
            "timestamp": int(time.time() * 1000),  # milliseconds
            "delay": delay_km,  # km
            "doppler": doppler,  # Hz
            "snr": 25.0 + (hash(f"{aircraft['lat']}{aircraft['lon']}") % 10),  # Mock SNR
            "aircraft_hex": aircraft.get("hex", "UNKNOWN"),
            "radar_id": radar["id"]
        }]
        
        logging.info(f"Radar {port}: delay={delay_km:.2f}km, doppler={doppler:.1f}Hz")
        return jsonify(detection)
    
    @radar_app.route('/api/config')
    def get_config():
        """Return radar configuration."""
        config = {
            "radar_id": radar["id"],
            "location": {
                "rx": {
                    "latitude": radar["lat"],
                    "longitude": radar["lon"],
                    "altitude": radar["alt"]
                },
                "tx": {
                    "latitude": TRANSMITTER["lat"],
                    "longitude": TRANSMITTER["lon"], 
                    "altitude": TRANSMITTER["alt"]
                }
            },
            "capture": {
                "fc": radar["frequency"]
            },
            "truth": {
                "adsb": {
                    "tar1090": "http://synthetic-adsb-test:5001"
                }
            },
            "frequency": radar["frequency"],
            "sample_rate": 2048000,
            "status": "active"
        }
        return jsonify(config)
    
    @radar_app.route('/api/status')
    def get_status():
        """Return radar status."""
        return jsonify({
            "status": "active",
            "radar_id": radar["id"],
            "last_detection": int(time.time() * 1000),
            "detection_count": 1
        })
    
    return radar_app


def run_radar_server(port):
    """Run a radar server on the specified port."""
    radar_app = create_radar_app(port)
    logging.info(f"Starting radar server on port {port}")
    radar_app.run(host='0.0.0.0', port=int(port), debug=False)


if __name__ == "__main__":
    # Start radar servers in separate threads
    threads = []
    for port in RADARS.keys():
        thread = Thread(target=run_radar_server, args=(port,))
        thread.daemon = True
        thread.start()
        threads.append(thread)
        time.sleep(0.5)  # Stagger startup
    
    logging.info("All radar servers started")
    
    # Keep main thread alive
    try:
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        logging.info("Shutting down radar servers")