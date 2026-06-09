# Biped Robot Control System

A complete end-to-end control system for a bipedal robot with an offline NLP pipeline, BLE command forwarding, WebSockets, and a live control dashboard.

## Setup
1. `cd idpl`
2. `pip install -r requirements.txt`
3. If you want the React dashboard, also run:
   - `cd frontend`
   - `npm install`
   - `npm run build`
   - `cd ..`
4. Connect the ESP32 running `Voxbot.ino`.
5. `python main.py`

The dashboard will open automatically at `http://localhost:8000`.
Alternatively, run `./run.sh` from `idpl/` after making it executable with `chmod +x run.sh`.
## What works
- Browser UI sends manual commands over WebSockets.
- Python backend forwards command payloads to the ESP32 via BLE.
- The ESP32 BLE server receives commands and pushes them into the drive queue.
- Telemetry and BLE connection status update on the dashboard.

## Notes
- If `idpl/frontend/dist` exists, the Python HTTP server will serve the built React app at `http://localhost:8000`.
- If no React build is present, the legacy `idpl/dashboard.html` page is used.
- The legacy dashboard remains available at `http://localhost:8000/dashboard`.
- Voice toggle is supported when audio dependencies are installed.
- If `sounddevice` / PortAudio or `faster-whisper` are unavailable, the app will still run with voice disabled.
