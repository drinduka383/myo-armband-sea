#!/usr/bin/env python3
import argparse
import asyncio

try:
    from bleak import BleakScanner
except ImportError as error:
    raise SystemExit("bleak is missing; run: pip install -r requirements.txt") from error

MYO_CONTROL_SERVICE = "d5060001-a904-deb9-4748-2c7f4a124842"


def is_myo(name, service_uuids):
    return "myo" in (name or "").lower() or "thalmic" in (name or "").lower() or MYO_CONTROL_SERVICE in {
        uuid.lower() for uuid in service_uuids
    }


async def scan(seconds):
    print(f"Scanning for {seconds:g} seconds...")
    devices = await BleakScanner.discover(timeout=seconds, return_adv=True)
    if not devices:
        print("No BLE advertisements found.")
    found = 0
    for address, (device, advertisement) in sorted(devices.items()):
        name = device.name or advertisement.local_name or "(unknown)"
        detected = is_myo(name, advertisement.service_uuids)
        found += detected
        marker = "MYO" if detected else ""
        print(f"{marker:4} {address}  {name}  RSSI={advertisement.rssi} dBm")
    if not found:
        print("No Myo service advertisement found. Wake/charge the armband and ensure it is not connected elsewhere.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20)
    args = parser.parse_args()
    asyncio.run(scan(args.seconds))


if __name__ == "__main__":
    main()
