import sys
import time
import asyncio
import threading
import webbrowser
from dataclasses import dataclass, field

from nlp.pipeline import NLPPipeline
from ble.client import BLEClient
from server.ws_server import WebSocketServer
from server.http_server import HTTPServer

try:
    from stt.recorder import VADAudioRecorder
    from stt.transcriber import WhisperTranscriber
    stt_available = True
except Exception as exc:
    print(f"[STT] Warning: voice features disabled: {exc}")
    stt_available = False

    class VADAudioRecorder:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("VADAudioRecorder is unavailable")

    class WhisperTranscriber:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("WhisperTranscriber is unavailable")

@dataclass
class SharedState:
    """Shared telemetry state — matches VoxBot firmware telemetry.
    
    The self-balancing robot provides:
      - pitch (degrees, from complementary filter)
      - pwm (motor output, from PID)
      - ble_connected (BLE connection state)
    
    No height, no encoders (N20 motors have no encoders).
    """
    pitch: float = 0.0
    pwm: float = 0.0
    uptime: float = 0.0
    last_cmd: str = "NONE"
    last_intent: str = "NONE"
    last_confidence: float = 0.0
    last_raw_text: str = ""
    last_vel_extracted: float = 0.5
    ble_connected: bool = False
    voice_active: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)

def ws_thread_func(state, ble_client):
    server = WebSocketServer(state, ble_client)
    asyncio.run(server.serve())

def voice_thread_func(state, pipeline, recorder, transcriber, ble_client):
    """Continuous voice recognition loop.
    
    When voice is active:
      1. VAD listens for speech onset
      2. Records until silence
      3. Whisper transcribes audio to text
      4. NLP pipeline extracts intent
      5. BLE sends command to robot (if connected)
    """
    while True:
        with state.lock:
            active = state.voice_active
        
        if not active:
            time.sleep(0.3)
            continue
            
        try:
            audio = recorder.record_utterance()
            if audio is not None:
                text = transcriber.transcribe(audio)
                if text:
                    result = pipeline.infer(text)
                    with state.lock:
                        state.last_raw_text = result["raw_text"]
                        state.last_intent = result["intent"]
                        state.last_confidence = result["confidence"]
                        state.last_vel_extracted = result["velocity"]
                        state.last_cmd = f"VOICE → {result['intent']}"
                    
                    print(f"[VOICE] \"{text}\" → {result['intent']} (conf: {result['confidence']*100:.0f}%)")
                    
                    # Send to robot (will silently fail if not connected)
                    ble_payload = result["ble_payload"]
                    try:
                        asyncio.run(ble_client.send_command(ble_payload))
                    except Exception as e:
                        print(f"[VOICE] Command send error (robot not connected?): {e}")
                else:
                    print("[VOICE] Could not transcribe audio.")
        except Exception as e:
            print(f"[VOICE] Recording error: {e}")
            time.sleep(1)

def uptime_updater(state):
    start_time = time.time()
    while True:
        with state.lock:
            state.uptime = time.time() - start_time
        time.sleep(1)

def main():
    print("\n╔══════════════════════════════════════════════╗")
    print("║   VoxBot Control System — Starting...       ║")
    print("╚══════════════════════════════════════════════╝\n")

    print("[INIT] Loading NLP pipeline...")
    pipeline = NLPPipeline()
    pipeline.startup()

    state = SharedState()
    
    # BLE — will keep scanning for VoxBot in the background
    # Dashboard works even without BLE connection
    print("[BLE]  Starting BLE scan thread (looking for 'VoxBot')...")
    ble_client = BLEClient(state)
    t_ble = threading.Thread(target=lambda: asyncio.run(ble_client.run_forever()), daemon=True)
    t_ble.start()

    # STT — voice recognition (if available)
    if stt_available:
        print("[STT]  Loading Whisper tiny.en model...")
        try:
            recorder = VADAudioRecorder()
            transcriber = WhisperTranscriber()
            print("[STT]  Whisper ready — voice features available.")
        except Exception as e:
            print(f"[STT]  Failed to initialize: {e}")
            stt_available_runtime = False
            recorder = None
            transcriber = None
    else:
        recorder = None
        transcriber = None
        print("[STT]  Voice features disabled (missing dependencies).")

    # WebSocket — real-time dashboard communication
    print("[WS]   WebSocket server starting on ws://localhost:8765")
    t_ws = threading.Thread(target=ws_thread_func, args=(state, ble_client), daemon=True)
    t_ws.start()

    # Voice thread
    if recorder and transcriber:
        t_voice = threading.Thread(target=voice_thread_func, args=(state, pipeline, recorder, transcriber, ble_client), daemon=True)
        t_voice.start()
        print("[STT]  Voice thread started.")
    else:
        print("[STT]  Voice thread disabled.")
    
    # Uptime counter
    t_uptime = threading.Thread(target=uptime_updater, args=(state,), daemon=True)
    t_uptime.start()

    # HTTP server + open browser
    print("[HTTP] Dashboard at http://localhost:8000")
    http_server = HTTPServer()
    
    print("[MAIN] All systems nominal. Opening browser...\n")
    webbrowser.open('http://localhost:8000')

    try:
        http_server.run()
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
        sys.exit(0)

if __name__ == "__main__":
    main()
