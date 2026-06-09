import json
import asyncio
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# Must match BLE_DEVICE_NAME in Voxbot.ino
TARGET_NAME = "VoxBot"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class BLEClient:
    def __init__(self, shared_state):
        self.client = None
        self.connected = False
        self.state = shared_state

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
            return True
        except BleakError as e:
            print(f"[BLE] Connect error: {e}")
            return False

    async def send_command(self, payload: str) -> bool:
        """Send a plain-text command to the robot.
        
        The VoxBot firmware expects plain strings like:
          DRIVE_FORWARD, DRIVE_BACKWARD, TURN_LEFT, TURN_RIGHT, STOP, SPEED:0.75
        """
        if not self.connected or not self.client:
            return False
            
        try:
            await self.client.write_gatt_char(CHAR_UUID, payload.encode())
            print(f"[BLE] Sent: {payload}")
            return True
        except BleakError:
            self.connected = False
            with self.state.lock:
                self.state.ble_connected = False
            return False

    async def run_forever(self):
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
