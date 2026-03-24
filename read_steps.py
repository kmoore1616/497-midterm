
"""
Simple BLE reader for ESP32-StepCounter.

This script:
1) Scans for the BLE device
2) Connects to it
3) Reads the current step count and temperature once
4) Subscribes to step notifications
5) Optionally polls temperature on an interval
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
DEFAULT_TEMP_UUID = "0b9ee8e3-f9be-5b43-9067-02a2e10d4d53"
DEFAULT_SCAN_SECONDS = 8.0
DEFAULT_TEMP_POLL_SECONDS = 5.0


def print_status(message):
    now_text = time.strftime("%H:%M:%S")
    print("[{}] {}".format(now_text, message))


def convert_4_bytes_little_endian_to_int(input_bytes):
    """
    Convert 4 little-endian bytes to an integer.
    Example: b'\\x0A\\x00\\x00\\x00' -> 10
    """
    if input_bytes is None:
        return None

    if len(input_bytes) < 4:
        return None

    return int.from_bytes(input_bytes[:4], byteorder="little", signed=True)


async def find_target_device(target_name, scan_seconds):
    print_status(
        "Scanning for '{}' for up to {} seconds...".format(
            target_name,
            scan_seconds,
        )
    )

    devices = await BleakScanner.discover(timeout=scan_seconds)

    found = None
    i = 0
    while i < len(devices):
        device = devices[i]
        name_text = str(device.name)
        if device.name == target_name:
            found = device
            break
        i = i + 1

    return found


def build_step_notification_handler():
    def notification_handler(_, data):
        step_value = convert_4_bytes_little_endian_to_int(data)
        print_status("Step count updated: {}".format(str(step_value)))

    return notification_handler


async def read_and_print_value(client, label, characteristic_uuid):
    raw_value = await client.read_gatt_char(characteristic_uuid)
    decoded_value = convert_4_bytes_little_endian_to_int(raw_value)
    print_status("{}: {}".format(label, str(decoded_value)))
    return decoded_value


async def connect_and_stream(address, step_uuid, temp_uuid, temp_poll_seconds):
    print_status("Connecting to {}".format(address))

    async with BleakClient(address) as client:
        is_connected = client.is_connected
        print_status("Connected: {}".format(str(is_connected)))

        print("")
        print("Current readings")
        print("----------------")
        await read_and_print_value(client, "Initial step count", step_uuid)
        await read_and_print_value(client, "Initial temperature", temp_uuid)

        print("")
        print("Live updates")
        print("------------")
        handler = build_step_notification_handler()
        await client.start_notify(step_uuid, handler)
        print_status("Listening for step-count updates.")
        if temp_poll_seconds > 0:
            print_status(
                "Temperature will refresh every {} seconds.".format(
                    temp_poll_seconds
                )
            )
        else:
            print_status("Temperature auto-refresh is disabled.")
        print_status("Press Ctrl+C to stop.")

        try:
            while True:
                if temp_poll_seconds > 0:
                    await asyncio.sleep(temp_poll_seconds)
                    await read_and_print_value(
                        client,
                        "Temperature",
                        temp_uuid,
                    )
                else:
                    await asyncio.sleep(1.0)
        finally:
            print_status("Stopping notifications...")
            await client.stop_notify(step_uuid)
            print_status("Disconnected.")


async def async_main(args):
    chosen_address = args.address

    if chosen_address is None:
        found_device = await find_target_device(args.name, args.scan_seconds)
        if found_device is None:
            print_status("Could not find a device named '{}'.".format(args.name))
            print_status("Make sure the ESP32 is powered on and advertising.")
            return 1
        chosen_address = found_device.address
        print_status("Found device at {}".format(chosen_address))

    await connect_and_stream(
        chosen_address,
        args.step_uuid,
        args.temp_uuid,
        args.temp_poll_seconds,
    )
    return 0


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Read step count and temperature data over BLE from ESP32"
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
    parser.add_argument(
        "--temp-uuid",
        default=DEFAULT_TEMP_UUID,
        help="Temperature characteristic UUID (default matches firmware)",
    )
    parser.add_argument(
        "--temp-poll-seconds",
        type=float,
        default=DEFAULT_TEMP_POLL_SECONDS,
        help=(
            "Seconds between temperature reads after connect "
            "(0 disables polling; default: {})"
        ).format(DEFAULT_TEMP_POLL_SECONDS),
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    print("ESP32 Step and Temperature Reader")
    print("=================================")
    print("Device name: {}".format(args.name))

    try:
        exit_code = asyncio.run(async_main(args))
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("")
        print_status("Stopped by user.")
        sys.exit(0)
    except Exception as ex:
        print_status("Error: {}".format(str(ex)))
        sys.exit(1)


if __name__ == "__main__":
    main()
