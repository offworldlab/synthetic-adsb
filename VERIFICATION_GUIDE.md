# Synthetic Anomaly Data Generator - Verification Guide

## Quick Verification

### 1. Unit Tests
```bash
cd /Users/jonnyspicer/repos/retina/synthetic-adsb
python3 -m pytest tests/ -v
```

Expected: All 26 tests pass

### 2. Server Module Test
```bash
cd /Users/jonnyspicer/repos/retina/synthetic-adsb
python3 test_server_simple.py
```

Expected output:
- Configuration shows ENABLE_ANOMALIES=True
- Aircraft Manager has 2 aircraft total
- Aircraft 1: Normal (CircularMotion, 30,000 ft)
- Aircraft 2: Anomalous (SupersonicLinearMotion, 50,000 ft, Mach 2-5)

### 3. HTTP Endpoint Test
```bash
cd /Users/jonnyspicer/repos/retina/synthetic-adsb
python3 -c "
from server import app
with app.test_client() as client:
    response = client.get('/data/aircraft.json')
    data = response.get_json()
    print(f'Aircraft count: {len(data[\"aircraft\"])}')
    for ac in data['aircraft']:
        mach = ac['gs'] / 661.5
        print(f'{ac[\"hex\"]}: {ac[\"flight\"].strip()} - {ac[\"gs\"]:.1f}kt (Mach {mach:.1f})')
"
```

Expected output:
- 2 aircraft
- First aircraft: ~80-120 knots (subsonic)
- Second aircraft: >1,300 knots (Mach 2+, supersonic)

### 4. Detection Endpoint Test
```bash
cd /Users/jonnyspicer/repos/retina/synthetic-adsb
python3 -c "
from server import app
with app.test_client() as client:
    response = client.get('/api/detection')
    data = response.get_json()
    print(f'Detections: {len(data[\"delay\"])}')
    print(f'Dopplers: {data[\"doppler\"]}')
"
```

Expected output:
- 2 detections
- Doppler values showing significant difference (normal vs supersonic)

## Full System Verification (Docker Compose)

### 5. Start Full System
```bash
cd /Users/jonnyspicer/repos/retina
docker compose down
docker compose up -d --build synthetic-adsb retina-tracker tar1090
```

### 6. Verify Services Running
```bash
docker compose ps
docker compose logs -f synthetic-adsb | head -20
```

Expected: synthetic-adsb shows "starting on http://0.0.0.0:5001"

### 7. Check ADS-B Endpoint
```bash
curl -s http://localhost:5001/data/aircraft.json | python3 -m json.tool
```

Expected JSON response with 2 aircraft:
```json
{
  "now": <timestamp>,
  "aircraft": [
    {
      "hex": "AEF123",
      "flight": "SYN001",
      "gs": 95.0,
      "lat": 37.xxx,
      "lon": -122.xxx,
      "alt_baro": 30000,
      ...
    },
    {
      "hex": "AEF124",
      "flight": "ANOM01",
      "gs": 1800.0,
      "lat": 37.xxx,
      "lon": -122.xxx,
      "alt_baro": 50000,
      ...
    }
  ]
}
```

### 8. Check Radar Detection Endpoint
```bash
curl -s http://localhost:49158/api/detection | python3 -m json.tool
```

Expected: Arrays with 2 elements each for delay, doppler, snr

### 9. Browser Verification (tar1090 UI)

Open http://localhost:8080 in your browser and verify:

#### Visual Checks:
- [ ] **Two aircraft visible** on the map
- [ ] **Normal aircraft (SYN001)**: Shows circular flight path around center point
- [ ] **Anomalous aircraft (ANOM01)**: Shows linear flight path
- [ ] **Distinct ICAO codes**: AEF123 and AEF124
- [ ] **Speed difference**: ANOM01 shows >1300 knots, SYN001 shows ~100 knots
- [ ] **Altitude difference**: ANOM01 at 50,000 ft, SYN001 at 30,000 ft

#### Track Details:
Click on the anomalous aircraft (ANOM01) and verify:
- [ ] Ground speed shows >1300 knots (>660 knots indicates supersonic)
- [ ] Calculated Mach number: GS / 661.5 should show Mach 2-5
- [ ] Aircraft appears to move much faster than normal aircraft
- [ ] Linear flight path (not circular)

### 10. Tracker Anomaly Detection
```bash
docker compose logs retina-tracker | grep -i anomaly
```

Expected: Log messages showing anomaly detection for supersonic targets

Check tracker creates anomalous track:
```bash
curl -s http://localhost:8080/data/aircraft.json | python3 -c "
import sys, json
data = json.load(sys.stdin)
for ac in data.get('aircraft', []):
    if ac.get('hex') == 'AEF124':
        print(f'Anomalous track found:')
        print(f'  Speed: {ac.get(\"gs\")} knots')
        print(f'  Mach: {ac.get(\"gs\", 0) / 661.5:.1f}')
        print(f'  Anomaly flag: {ac.get(\"is_anomalous\", \"not set\")}')
"
```

## Verification Checklist

### Phase 1: Supersonic Anomalies (MVP)
- [x] Motion pattern module created
- [x] Configuration added to .env
- [x] Multi-aircraft support implemented
- [x] Detection endpoints handle multiple aircraft
- [x] Unit tests pass (26 tests)
- [x] HTTP endpoints return 2 aircraft
- [x] Supersonic velocities generated (Mach 2-5)
- [ ] Docker Compose system test (requires user verification)
- [ ] Browser visualization test (requires user verification)
- [ ] Tracker anomaly detection test (requires user verification)

### Success Criteria
- [x] Generate supersonic aircraft (Mach 2-5)
- [x] Support multiple simultaneous aircraft (normal + anomalous)
- [x] Output valid ADS-B JSON format with multiple aircraft
- [x] Output valid radar delay/Doppler arrays
- [ ] retina-tracker flags supersonic tracks as anomalous (pending Docker test)
- [x] All unit tests pass
- [ ] Integration test passes end-to-end (pending Docker test)
- [ ] Visual verification in tar1090 (pending browser test)

## Troubleshooting

### Only 1 aircraft appears
- Check .env file has `ENABLE_ANOMALIES=true`
- Check `ANOMALOUS_AIRCRAFT_COUNT=1` or higher
- Restart server/container to reload configuration
- Verify environment variables: `python3 -c "from dotenv import load_dotenv; import os; load_dotenv(); print(os.environ.get('ENABLE_ANOMALIES'))"`

### Anomalous aircraft not supersonic
- Check `SUPERSONIC_MACH_MIN` and `SUPERSONIC_MACH_MAX` in .env
- Expected range: 2.0 - 5.0
- Verify with: `curl localhost:5001/data/aircraft.json | jq '.aircraft[1].gs'`
- Should show >1300 knots

### Anomalous aircraft not moving
- SupersonicLinearMotion starts from random position near TX
- May move off-screen quickly at high speed
- Check coordinates: `curl localhost:5001/data/aircraft.json | jq '.aircraft[1] | {lat, lon, gs}'`
- Refresh browser to see current position

### No detections generated
- Check radar endpoints: `curl localhost:49158/api/detection`
- Verify arrays have 2 elements
- Check Docker network connectivity
- Restart synthetic-adsb container

## Configuration Reference

Default configuration (`.env`):
```bash
ENABLE_ANOMALIES=true
NORMAL_AIRCRAFT_COUNT=1
ANOMALOUS_AIRCRAFT_COUNT=1
SUPERSONIC_MACH_MIN=2.0
SUPERSONIC_MACH_MAX=5.0
SUPERSONIC_ALTITUDE_FT=50000
```

## Next Steps

After verifying Phase 1 (Supersonic Anomalies):
- Implement Phase 2: Direction Change Anomalies
- Implement Phase 3: Instant Acceleration Anomalies
- Implement Phase 4: Combination Anomalies
- Implement Phase 5: Realism Features (ADS-B intermittency, noise, dropout)
- Implement Phase 6: Real Radar Overlay

See full implementation plan in project documentation.
