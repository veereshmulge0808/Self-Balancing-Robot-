import json
import asyncio
import websockets

class WebSocketServer:
    def __init__(self, shared_state, ble_client, host="localhost", port=8765):
        self.state = shared_state
        self.ble_client = ble_client
        self.host = host
        self.port = port
        self.clients = set()

    async def send_loop(self, websocket):
        try:
            while True:
                with self.state.lock:
                    data = {
                        "pitch": self.state.pitch,
                        "pwm": self.state.pwm,
                        "uptime": self.state.uptime,
                        "last_cmd": self.state.last_cmd,
                        "last_intent": self.state.last_intent,
                        "last_confidence": self.state.last_confidence,
                        "last_raw_text": self.state.last_raw_text,
                        "last_vel_extracted": self.state.last_vel_extracted,
                        "ble_connected": self.state.ble_connected,
                        "voice_active": self.state.voice_active
                    }
                await websocket.send(json.dumps(data))
                await asyncio.sleep(0.1)
        except websockets.exceptions.ConnectionClosed:
            pass

    async def recv_loop(self, websocket):
        """Handle incoming commands from the dashboard.
        
        Commands from dashboard buttons are plain text matching the firmware:
          DRIVE_FORWARD, DRIVE_BACKWARD, TURN_LEFT, TURN_RIGHT, STOP, SPEED:x.xx
        """
        try:
            async for message in websocket:
                data = json.loads(message)
                if "cmd" in data:
                    cmd = data["cmd"]
                    if cmd == "VOICE_TOGGLE":
                        with self.state.lock:
                            self.state.voice_active = not self.state.voice_active
                            active = self.state.voice_active
                        print(f"[WS] Voice {'ACTIVATED' if active else 'DEACTIVATED'}")
                    elif cmd == "NLP_TEXT":
                        # Process typed text through NLP
                        text = data.get("text", "")
                        if text:
                            from nlp.pipeline import NLPPipeline
                            pipeline = NLPPipeline()
                            result = pipeline.infer(text)
                            with self.state.lock:
                                self.state.last_raw_text = result["raw_text"]
                                self.state.last_intent = result["intent"]
                                self.state.last_confidence = result["confidence"]
                                self.state.last_vel_extracted = result["velocity"]
                                self.state.last_cmd = f"TEXT → {result['intent']}"
                            
                            # Send to robot if connected
                            ble_payload = result["ble_payload"]
                            try:
                                await self.ble_client.send_command(ble_payload)
                            except Exception as exc:
                                print(f"[WS] BLE send failed (robot may not be connected): {exc}")
                            print(f"[WS] NLP: \"{text}\" → {result['intent']} (conf: {result['confidence']*100:.0f}%)")
                    elif cmd == "SPEED":
                        val = data.get("val", 0.5)
                        ble_payload = f"SPEED:{float(val):.2f}"
                        with self.state.lock:
                            self.state.last_cmd = f"MANUAL → SPEED:{val}"
                            self.state.last_intent = "SPEED"
                            self.state.last_vel_extracted = float(val)
                        try:
                            await self.ble_client.send_command(ble_payload)
                        except Exception as exc:
                            print(f"[WS] BLE send failed: {exc}")
                    else:
                        # Direct movement commands: DRIVE_FORWARD, TURN_LEFT, etc.
                        with self.state.lock:
                            self.state.last_cmd = f"MANUAL → {cmd}"
                            self.state.last_intent = cmd
                        try:
                            await self.ble_client.send_command(cmd)
                        except Exception as exc:
                            print(f"[WS] BLE send failed: {exc}")
                        print(f"[WS] Manual command: {cmd}")
        except websockets.exceptions.ConnectionClosed:
            pass
        except json.JSONDecodeError:
            pass

    async def handler(self, websocket):
        self.clients.add(websocket)
        send_task = asyncio.create_task(self.send_loop(websocket))
        recv_task = asyncio.create_task(self.recv_loop(websocket))
        
        done, pending = await asyncio.wait(
            [send_task, recv_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        
        for task in pending:
            task.cancel()
            
        self.clients.discard(websocket)

    async def serve(self):
        async with websockets.serve(self.handler, self.host, self.port):
            await asyncio.Future()  # run forever
