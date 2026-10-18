

import sys
import csv
import time
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

BAUD = 115200
SAVE_DIR = Path(__file__).resolve().parent / "test_logs"

# USB-serial chips used on ESP32 dev boards (CP210x, CH340, FTDI, ESP native)
KNOWN_VIDS = {0x10C4, 0x1A86, 0x0403, 0x303A}


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid in KNOWN_VIDS:
            return p.device
    if len(ports) == 1:
        return ports[0].device
    print("Could not find the ESP32 automatically. Ports found:")
    for p in ports:
        print(f"  {p.device}  {p.description}")
    print("Run again with the port name, e.g.  python log_capture.py COM5")
    sys.exit(1)


def open_port(name):
    # Open without toggling DTR/RTS so the ESP32 isn't reset on connect
    ser = serial.Serial()
    ser.port = name
    ser.baudrate = BAUD
    ser.timeout = 0.5
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def save_rows(header, rows):
    SAVE_DIR.mkdir(exist_ok=True)
    name = datetime.now().strftime("log_%Y-%m-%d_%H-%M-%S.csv")
    with open(SAVE_DIR / name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    return name


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    ser = open_port(port)
    print(f"Listening on {port}. Files are saved to {SAVE_DIR}")
    print("Start a test on the load... (Ctrl+C to quit)")

    header = None      # column names of the current/last test
    rows = []          # readings of the current/last test
    ended = False      # test has ended and is waiting for YES/NO

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()

            if line.startswith("HDR,"):
                header = line[4:].split(",")
                rows = []
                ended = False
                print("\nTest started")

            elif line.startswith("DATA,") and header and not ended:
                rows.append(line[5:].split(","))
                print(f"\r  {len(rows)} rows received", end="", flush=True)

            elif line.startswith("END,") and header:
                _, t, reason = line.split(",", 2)
                ended = True
                print(f"\nTest ended after {t} s ({reason}) - answer on the load's screen")

            elif line == "SAVE":
                if header and rows:
                    name = save_rows(header, rows)
                    print(f"Saved {len(rows)} rows -> test_logs/{name}")
                    time.sleep(0.05)
                    ser.write(f"SAVED,{name}\n".encode())
                else:
                    print("SAVE requested but no test data was received")
                    ser.write(b"SAVED,NO DATA RECEIVED\n")
                header, rows, ended = None, [], False

            elif line == "DISCARD":
                print("Not saved (NO pressed on the load)")
                header, rows, ended = None, [], False

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
