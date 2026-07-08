#!/usr/bin/env python3
import argparse
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError as error:
    raise SystemExit("pyserial is missing; run: pip install -r requirements.txt") from error


def default_port():
    ports = list(list_ports.comports())
    for port in ports:
        if "STM" in port.description.upper() or "STLINK" in port.description.upper() or "ST-LINK" in port.description.upper():
            return port.device
    return ports[0].device if ports else "/dev/ttyACM0"


def send(link, command):
    print(f"> {command}")
    link.write((command + "\n").encode("ascii"))
    link.flush()
    reply = link.readline().decode("ascii", "replace").strip()
    print(f"< {reply or '(no ACK)'}")


def main():
    parser = argparse.ArgumentParser(description="Safe STM32 serial command test")
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pattern", action="store_true", help="STOP 2 s, RUN 2 s, STOP")
    args = parser.parse_args()
    port = args.port or default_port()
    with serial.serial_for_url(port, args.baud, timeout=1, write_timeout=1) as link:
        time.sleep(0.2)
        send(link, "STOP")
        if args.pattern:
            time.sleep(2)
            send(link, "RUN")
            time.sleep(2)
            send(link, "STOP")
            return
        print("Commands: 0/STOP, 1/RUN, p 0..100, s/STATUS, q")
        while True:
            try:
                command = input("serial> ").strip()
                if command.lower() == "q":
                    send(link, "STOP")
                    return
                if command:
                    send(link, command)
            except (EOFError, KeyboardInterrupt):
                print()
                send(link, "STOP")
                return


if __name__ == "__main__":
    main()
