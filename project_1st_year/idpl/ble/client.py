import re
import asyncio
import threading
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# Must match BLE_DEVICE_NAME in Voxbot.ino
TARGET_NAME = "VoxBot"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_CMD_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHAR_STATUS_UUID = "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

class BLEClient:
    """Thread-safe BLE client for VoxBot.
    
    Architecture:
      - Owns a dedicated asyncio event loop running in a background thread.
      - All async BLE operations happen on THIS loop only (Bleak requirement).
      - External threads (voice, WS) call send_command_threadsafe() which
        safely schedules work onto the BLE loop via run_coroutine_threadsafe().
    """
    def __init__(self, shared_state):
        self.client = None
        self.connected = False
        self.state = shared_state
        self._loop = None  # Set when the BLE thread starts

    def _on_notify(self, sender, data: bytearray):
        """Handle BLE notifications from the firmware's status characteristic.
        
        Firmware sends: "P:+1.23,W:+045" (pitch degrees, base PWM)
        """
        try:
            text = data.decode("utf-8", errors="ignore").strip()
            # Parse "P:+1.23,W:+045"
            match = re.match(r"P:([+-]?[\d.]+),W:([+-]?[\d.]+)", text)
            if match:
                pitch = float(match.group(1))
                pwm = float(match.group(2))
                with self.state.lock:
                    self.state.pitch = pitch
                    self.state.pwm = pwm
        except Exception as e:
            print(f"[BLE] Notify parse error: {e}")

    async def scan_and_connect(self) -> bool:
        print(f"[BLE] Scanning for {TARGET_NAME}...")
        try:
            device = await BleakScanner.find_device_by_name(TARGET_NAME, timeout=10.0)
        except Exception as e:
            print(f"[BLE] Scanner error: {e}")
            return False
        
        if not device:
            print("[BLE] Device not found, retry in 5s")
            return False
            
        try:
            self.client = BleakClient(device)
            await self.client.connect()
            self.connected = True
            with self.state.lock:
                self.state.ble_connected = True
            print("[BLE] Connected!")

            # Subscribe to telemetry notifications (pitch + PWM from firmware)
            try:
                await self.client.start_notify(CHAR_STATUS_UUID, self._on_notify)
                print("[BLE] Subscribed to telemetry notifications.")
            except Exception as e:
                print(f"[BLE] Warning: could not subscribe to notifications: {e}")

            return True
        except BleakError as e:
            print(f"[BLE] Connect error: {e}")
            return False

    async def send_command(self, payload: str) -> bool:
        """Send a plain-text command to the robot.
        
        The VoxBot firmware expects plain strings like:
          DRIVE_FORWARD, DRIVE_BACKWARD, TURN_LEFT, TURN_RIGHT, STOP, SPEED:0.75
        
        IMPORTANT: This must only be called from the BLE event loop.
        External threads should use send_command_threadsafe() instead.
        """
        if not self.connected or not self.client:
            return False
            
        try:
            await self.client.write_gatt_char(CHAR_CMD_UUID, payload.encode())
            print(f"[BLE] Sent: {payload}")
            return True
        except BleakError:
            self.connected = False
            with self.state.lock:
                self.state.ble_connected = False
            return False

    def send_command_threadsafe(self, payload: str) -> bool:
        """Thread-safe command sender for use from ANY thread.
        
        Schedules the async send_command() on the BLE event loop and
        waits for the result with a timeout. Safe to call from the
        voice thread, WS server thread, or any other thread.
        """
        if not self._loop or not self.connected:
            return False
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.send_command(payload), self._loop
            )
            return future.result(timeout=3.0)
        except Exception as e:
            print(f"[BLE] Threadsafe send error: {e}")
            return False

    async def run_forever(self):
        """Main BLE loop — runs in the BLE thread's event loop."""
        # Capture this loop so send_command_threadsafe can post work here
        self._loop = asyncio.get_running_loop()
        
        while True:
            if not self.connected:
                success = await self.scan_and_connect()
                if not success:
                    await asyncio.sleep(5)
            else:
                try:
                    if not self.client.is_connected:
                        print("[BLE] Disconnected")
                        self.connected = False
                        with self.state.lock:
                            self.state.ble_connected = False
                        await asyncio.sleep(3)
                    else:
                        await asyncio.sleep(1)
                except Exception:
                    self.connected = False
                    with self.state.lock:
                        self.state.ble_connected = False
                    await asyncio.sleep(3)
