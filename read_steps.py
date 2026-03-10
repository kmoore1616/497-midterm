
"""
Simple BLE step reader for ESP32-StepCounter.

This script:
1) Scans for the BLE device
2) Connects to it
3) Reads the current step count once
4) Subscribes to notifications and prints updates
"""

import asyncio
import argparse
import sys
import time

from bleak import BleakScanner
from bleak import BleakClient


DEFAULT_DEVICE_NAME = "ESP32-StepCounter"
DEFAULT_SERVICE_UUID = "6cfb5360-8c88-4f50-9f24-6ed6bd8d3f8f"
DEFAULT_STEP_UUID = "0b8dd7d2-e8ad-4a32-8f56-f191d0fc3c42"
DEFAULT_SCAN_SECONDS = 8.0


def convert_4_bytes_little_endian_to_int(input_bytes):
    """
    Convert 4 little-endian bytes to an integer.
    Example: b'\\x0A\\x00\\x00\\x00' -> 10
    """
    if input_bytes is None:
        return None

    if len(input_bytes) < 4:
        return None

    b0 = input_bytes[0]
    b1 = input_bytes[1]
    b2 = input_bytes[2]
    b3 = input_bytes[3]

    value = b0 + (b1 << 8) + (b2 << 16) + (b3 << 24)
    return value


def bytes_to_hex_string(input_bytes):
    if input_bytes is None:
        return ""

    pieces = []
    index = 0
    while index < len(input_bytes):
        pieces.append("{:02X}".format(input_bytes[index]))
        index = index + 1
    return " ".join(pieces)


async def find_target_device(target_name, scan_seconds):
    print("Scanning for BLE devices for {} seconds...".format(scan_seconds))
    print("Looking for device name: {}".format(target_name))

    devices = await BleakScanner.discover(timeout=scan_seconds)

    found = None
    i = 0
    while i < len(devices):
        device = devices[i]
        name_text = str(device.name)
        print("  Found: name='{}' address='{}'".format(name_text, device.address))
        if device.name == target_name:
            found = device
            break
        i = i + 1

    return found


def build_notification_handler():
    def notification_handler(characteristic_handle, data):
        # characteristic_handle is provided by bleak and may be integer/object by backend.
        step_value = convert_4_bytes_little_endian_to_int(data)
        hex_text = bytes_to_hex_string(data)
        now_text = time.strftime("%H:%M:%S")
        print("[{}] Notification from {}: raw=[{}] steps={}".format(
            now_text,
            str(characteristic_handle),
            hex_text,
            str(step_value),
        ))

    return notification_handler


async def connect_and_stream(address, step_uuid):
    print("Connecting to address: {}".format(address))

    async with BleakClient(address) as client:
        is_connected = client.is_connected
        print("Connected: {}".format(str(is_connected)))

        print("Reading current value from step characteristic...")
        first_value_bytes = await client.read_gatt_char(step_uuid)
        first_value_int = convert_4_bytes_little_endian_to_int(first_value_bytes)
        print("Initial read: raw=[{}] steps={}".format(
            bytes_to_hex_string(first_value_bytes),
            str(first_value_int),
        ))

        print("Enabling notifications...")
        handler = build_notification_handler()
        await client.start_notify(step_uuid, handler)
        print("Notifications enabled. Press Ctrl+C to stop.")

        try:
            while True:
                await asyncio.sleep(1.0)
        finally:
            print("Disabling notifications...")
            await client.stop_notify(step_uuid)
            print("Disconnected.")


async def async_main(args):
    chosen_address = args.address

    if chosen_address is None:
        found_device = await find_target_device(args.name, args.scan_seconds)
        if found_device is None:
            print("ERROR: Could not find device named '{}'.".format(args.name))
            print("Tip: make sure ESP32 is powered and advertising.")
            return 1
        chosen_address = found_device.address

    await connect_and_stream(chosen_address, args.step_uuid)
    return 0


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Read step count data over BLE from ESP32 step counter"
    )
    parser.add_argument(
        "--name",
        default=DEFAULT_DEVICE_NAME,
        help="BLE device name to scan for (default: {})".format(DEFAULT_DEVICE_NAME),
    )
    parser.add_argument(
        "--address",
        default=None,
        help="Connect directly to a BLE address (skip scan)",
    )
    parser.add_argument(
        "--scan-seconds",
        type=float,
        default=DEFAULT_SCAN_SECONDS,
        help="How long to scan for devices (default: {})".format(DEFAULT_SCAN_SECONDS),
    )
    parser.add_argument(
        "--service-uuid",
        default=DEFAULT_SERVICE_UUID,
        help="Activity service UUID (kept for clarity; not required by client)",
    )
    parser.add_argument(
        "--step-uuid",
        default=DEFAULT_STEP_UUID,
        help="Step characteristic UUID (default matches firmware)",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    print("BLE Step Reader starting...")
    print("Target name: {}".format(args.name))
    print("Step characteristic UUID: {}".format(args.step_uuid))

    try:
        exit_code = asyncio.run(async_main(args))
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("")
        print("Stopped by user.")
        sys.exit(0)
    except Exception as ex:
        print("ERROR: {}".format(str(ex)))
        sys.exit(1)


if __name__ == "__main__":
    main()
