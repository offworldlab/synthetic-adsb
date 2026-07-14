#!/bin/bash
set -e

echo "Starting synthetic ADS-B and radar simulation..."

# Start ADS-B server in background
echo "Starting ADS-B server on port 5001..."
python server.py &
ADSB_PID=$!

# Wait for ADS-B server to start
sleep 3

# Start mock radar servers
echo "Starting mock radar servers on ports 49158-49160..."
python mock_radar.py &
RADAR_PID=$!

# Function to handle shutdown
shutdown() {
    echo "Shutting down services..."
    kill $ADSB_PID $RADAR_PID 2>/dev/null || true
    wait $ADSB_PID $RADAR_PID 2>/dev/null || true
    exit 0
}

# Set up signal handling
trap shutdown SIGTERM SIGINT

echo "All services started successfully!"
echo "- ADS-B data: http://localhost:5001/data/aircraft.json"
echo "- Radar 1: http://localhost:49158/api/detection"
echo "- Radar 2: http://localhost:49159/api/detection" 
echo "- Radar 3: http://localhost:49160/api/detection"

# Wait for both processes
wait $ADSB_PID $RADAR_PID