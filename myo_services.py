#!/usr/bin/env python3
import argparse
import asyncio

try:
    from bleak import BleakClient, BleakScanner
except ImportError as error:
    raise SystemExit("bleak is missing; run: pip install -r requirements.txt") from error


async def inspect(identifier):
    device = await BleakScanner.find_device_by_address(identifier, timeout=15)
    if device is None:
        device = await BleakScanner.find_device_by_name(identifier, timeout=15)
    if device is None:
        raise SystemExit(f"Myo not found: {identifier}")
    print(f"Connecting to {device.name or '(unknown)'} {device.address}...")
    async with BleakClient(device) as client:
        for service in client.services:
            print(f"SERVICE {service.uuid}  {service.description}")
            for characteristic in service.characteristics:
                props = ",".join(characteristic.properties)
                print(f"  CHAR {characteristic.uuid}  [{props}] {characteristic.description}")
                for descriptor in characteristic.descriptors:
                    print(f"    DESC {descriptor.uuid}  handle={descriptor.handle}")


parser = argparse.ArgumentParser()
parser.add_argument("identifier", help="BLE address or exact advertised name")
args = parser.parse_args()
asyncio.run(inspect(args.identifier))
