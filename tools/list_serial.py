#!/usr/bin/env python3
try:
    from serial.tools import list_ports
except ImportError as error:
    raise SystemExit("pyserial is missing; run: pip install -r requirements.txt") from error

ports = list(list_ports.comports())
if not ports:
    print("No serial ports found.")
for port in ports:
    print(f"{port.device}\t{port.description}\t{port.hwid}")
