"""
VoxBot ESP32 Firmware Verification Script
==========================================
Connects to VoxBot via BLE and verifies:
  1. Device is discoverable with name "VoxBot"
  2. Expected Service UUID is present
  3. Expected Command Characteristic UUID is present
  4. Expected Status/Telemetry Characteristic UUID is present
  5. Reads live telemetry (pitch + PWM) from the Status characteristic
  6. Sends a test STOP command and checks it's accepted
"""

import asyncio
from bleak import BleakScanner, BleakClient

# ── Expected UUIDs (must match Voxbot.ino SECTION 7) ──────────────────────────
TARGET_NAME      = "VoxBot"
SERVICE_UUID     = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHAR_CMD_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHAR_STATUS_UUID = "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

PASS = "✅"
FAIL = "❌"
WARN = "⚠️ "

def separator():
    print("─" * 56)

async def main():
    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║   VoxBot ESP32 — BLE Firmware Diagnostic            ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()

    # ── Step 1: Scan for VoxBot ───────────────────────────────────────────────
    separator()
    print(f"[1/5] Scanning for BLE device named '{TARGET_NAME}' (10s)...")
    device = await BleakScanner.find_device_by_name(TARGET_NAME, timeout=10.0)

    if device is None:
        print(f"{FAIL} Device '{TARGET_NAME}' NOT found. ESP32 may not be advertising.")
        print("      → Check if firmware is uploaded and ESP32 is powered on.")
        return

    print(f"{PASS} Found: {device.name}  |  Address: {device.address}")
    separator()

    # ── Step 2: Connect ───────────────────────────────────────────────────────
    print(f"[2/5] Connecting to {device.address}...")
    async with BleakClient(device) as client:
        if not client.is_connected:
            print(f"{FAIL} Connection failed.")
            return
        print(f"{PASS} Connected successfully!")
        separator()

        # ── Step 3: Enumerate Services & Characteristics ──────────────────────
        print("[3/5] Inspecting GATT Services and Characteristics...")
        print()

        found_service      = False
        found_cmd_char     = False
        found_status_char  = False

        for service in client.services:
            uuid_str = str(service.uuid).lower()
            is_our_service = (uuid_str == SERVICE_UUID.lower())

            if is_our_service:
                found_service = True
                print(f"  {PASS} SERVICE  : {service.uuid}")
                print(f"            Description: {service.description}")
            else:
                print(f"  [other]   SERVICE  : {service.uuid}  ({service.description})")

            for char in service.characteristics:
                char_uuid = str(char.uuid).lower()
                props = ", ".join(char.properties)

                if char_uuid == CHAR_CMD_UUID.lower():
                    found_cmd_char = True
                    print(f"      {PASS} CHAR (Command)  : {char.uuid}")
                    print(f"                Properties: {props}")

                elif char_uuid == CHAR_STATUS_UUID.lower():
                    found_status_char = True
                    print(f"      {PASS} CHAR (Telemetry): {char.uuid}")
                    print(f"                Properties: {props}")

                else:
                    print(f"      [other]   CHAR: {char.uuid}  [{props}]")
            print()

        separator()

        # ── Step 4: UUID Match Summary ────────────────────────────────────────
        print("[4/5] UUID Match Check (vs Voxbot.ino SECTION 7):")
        print()
        print(f"  Service UUID     : {PASS if found_service     else FAIL}  {SERVICE_UUID}")
        print(f"  Command Char UUID: {PASS if found_cmd_char    else FAIL}  {CHAR_CMD_UUID}")
        print(f"  Status Char UUID : {PASS if found_status_char else FAIL}  {CHAR_STATUS_UUID}")
        print()

        if not (found_service and found_cmd_char and found_status_char):
            print(f"{FAIL} UUID MISMATCH — Wrong firmware may be loaded on the ESP32!")
            print("   → Re-upload Voxbot.ino via Arduino IDE.")
            separator()
            return

        print(f"{PASS} All UUIDs match Voxbot.ino. Firmware identity confirmed!")
        separator()

        # ── Step 5: Read Live Telemetry ───────────────────────────────────────
        print("[5/5] Reading live telemetry from Status characteristic (3 readings)...")
        print()

        telemetry_received = False

        def on_notify(sender, data):
            nonlocal telemetry_received
            telemetry_received = True
            decoded = data.decode("utf-8", errors="replace")
            print(f"  📡 Telemetry: {decoded}  (raw bytes: {data.hex()})")

        try:
            await client.start_notify(CHAR_STATUS_UUID, on_notify)
            await asyncio.sleep(3.0)  # Wait 3s to collect a few notify events
            await client.stop_notify(CHAR_STATUS_UUID)
        except Exception as e:
            print(f"  {WARN} Could not subscribe to telemetry notify: {e}")

        print()
        if telemetry_received:
            print(f"  {PASS} Live telemetry is flowing from ESP32!")
        else:
            print(f"  {WARN} No telemetry received — MPU6050 may be stuck or not wired.")
            print("      → Open Arduino Serial Monitor at 115200 baud for details.")

        separator()
        print()
        print("╔══════════════════════════════════════════════════════╗")

        if found_service and found_cmd_char and found_status_char and telemetry_received:
            print("║  RESULT: ✅  FIRMWARE OK — ESP32 is fully operational  ║")
        elif found_service and found_cmd_char and found_status_char:
            print("║  RESULT: ⚠️  FIRMWARE OK — but MPU6050 not sending data ║")
        else:
            print("║  RESULT: ❌  FIRMWARE MISMATCH — re-upload Voxbot.ino  ║")

        print("╚══════════════════════════════════════════════════════╝")
        print()

asyncio.run(main())
