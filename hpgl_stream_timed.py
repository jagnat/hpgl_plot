"""hpgl_stream_timed.py - Stream an HP-GL record file to an HP 7440A over RS-232
using a simple time delay instead of the Enquire/Acknowledge handshake.

The base HP 7440A has a small input buffer and no reliable hardware flow
control. hpgl_stream.py paces sends with the plotter's ENQ/ACK handshake, but
that handshake doesn't work on every Mac/adapter. This version sidesteps it
entirely: send a record, then *wait* long enough for the plotter to have
consumed (plotted) it before sending the next one, so the buffer never fills.

The delay per record is the larger of:
  * the time to transmit the bytes at the given baud, plus
  * a fixed settle time for the pen to actually move (DELAY).
You can bump DELAY up if you still see dropped/garbled output, or down to go
faster once you find the plotter keeps up.

Requires pyserial:  python3 -m pip install pyserial

Usage:
    python3 hpgl_stream_timed.py file.hgl <serial-port> [baud] [delay-seconds]
    python3 hpgl_stream_timed.py file.hgl /dev/tty.usbserial-XXXX 9600 0.5
"""

import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit('pyserial not installed. Run: python3 -m pip install pyserial')

BAUD = 9600
BYTESIZE = 8
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE

# Fixed settle time per record (seconds). Generous so the pen finishes moving
# and the small buffer drains before the next record arrives. Tune as needed.
DELAY = 0.5


def load_records(filename):
    with open(filename) as f:
        return [line.strip() for line in f if line.strip()]


def main():
    if len(sys.argv) < 3:
        sys.exit('Usage: python3 hpgl_stream_timed.py file.hgl '
                 '<serial-port> [baud] [delay-seconds]')

    filename = sys.argv[1]
    port = sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else BAUD
    delay = float(sys.argv[4]) if len(sys.argv) > 4 else DELAY

    records = load_records(filename)
    if not records:
        sys.exit('No records found in ' + filename)

    ser = serial.Serial(port, baudrate=baud, bytesize=BYTESIZE,
                        parity=PARITY, stopbits=STOPBITS, timeout=1)
    try:
        # Put the plotter in the programmed-on state (harmless if already on).
        ser.write(b'\x1b.(')
        ser.flush()
        time.sleep(0.2)

        total = len(records)
        for i, record in enumerate(records, 1):
            data = record.encode('ascii')
            ser.write(data)
            ser.flush()
            # Wait out the transmit time plus a fixed settle delay.
            time.sleep(len(data) * 10.0 / baud + delay)
            print('[{}/{}] {}'.format(i, total, record))
        print('Done. Sent {} records.'.format(total))
    finally:
        ser.close()


if __name__ == '__main__':
    main()
