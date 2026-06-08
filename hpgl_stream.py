"""hpgl_stream.py - Stream an HP-GL record file to an HP 7440A over RS-232.

The base HP 7440A ColorPro (no Graphics Enhancement Cartridge) has a small input
buffer and no reliable hardware flow control, so we cannot just blast the file at
it. Instead we use the plotter's built-in *Enquire/Acknowledge* software
handshake (documented in the HP 7470A/7440A Interfacing and Programming Manual,
device-control instruction ESC.I):

    1. Configure the plotter once:  ESC . R   (reset handshake to defaults)
                                    ESC . I <block>;5;6:
       -> handshake mode 2, block size = BLOCK_SIZE bytes,
          enquiry character = ENQ (5), acknowledgment string = ACK (6).
    2. For each record: send ENQ, wait until the plotter replies ACK (it only
       does so once it has room for a full block), then send the record bytes.

Because each record from svg2hpgl.py is <= MAX_RECORD <= BLOCK_SIZE bytes, every
record is a single safe block. This keeps the tiny buffer from ever overflowing.

Requires pyserial:  python3 -m pip install pyserial

Usage:
    python3 hpgl_stream.py file.hgl /dev/tty.usbserial-XXXX [baud]
    python3 hpgl_stream.py file.hgl COM3 9600
"""

import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit('pyserial not installed. Run: python3 -m pip install pyserial')

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

BAUD = 9600          # 7440A default; set the plotter's rear switches to match.
BYTESIZE = 8
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE

ESC = b'\x1b'
ENQ = b'\x05'        # enquiry character we ask the plotter with
ACK = 0x06           # acknowledgment byte the plotter answers with

# Must be >= the converter's MAX_RECORD (58). The plotter only ACKs an enquiry
# once it has at least this many free bytes, so a full record always fits.
BLOCK_SIZE = 58

ACK_TIMEOUT = 30.0   # seconds to wait for an ACK before giving up
READ_POLL = 0.05     # seconds between ACK polls


def configure_handshake(ser):
    """Put the plotter into Enquire/Acknowledge mode (handshake mode 2)."""
    # ESC.( puts the plotter in the programmed-on state. Harmless if the rear
    # Y/D switch is on D (already on); required if it is on Y.
    ser.write(ESC + b'.(')
    # ESC.R resets all handshake parameters to defaults (hardwire handshake on).
    ser.write(ESC + b'.R')
    # ESC.I<block>;<enq>;<ack>:  -> mode 2: block size, ENQ=5, ACK=6.
    # In mode 2 the plotter sends ONLY the ack string (no output terminator),
    # so each enquiry yields exactly one ACK byte.
    ser.write(ESC + b'.I%d;5;6:' % BLOCK_SIZE)
    ser.flush()
    time.sleep(0.1)
    ser.reset_input_buffer()


def wait_for_ack(ser):
    """Send an enquiry and block until the plotter answers ACK."""
    ser.reset_input_buffer()
    ser.write(ENQ)
    ser.flush()
    deadline = time.time() + ACK_TIMEOUT
    while time.time() < deadline:
        b = ser.read(1)
        if b and b[0] == ACK:
            return True
        if not b:
            time.sleep(READ_POLL)
    return False


def send_record(ser, record):
    """Wait for buffer space, then transmit one HP-GL record."""
    if not wait_for_ack(ser):
        raise TimeoutError('No ACK from plotter (check cabling, baud, power).')
    ser.write(record.encode('ascii'))
    ser.flush()


def load_records(filename):
    with open(filename) as f:
        return [line.strip() for line in f if line.strip()]


def main():
    if len(sys.argv) < 3:
        sys.exit('Usage: python3 hpgl_stream.py file.hgl <serial-port> [baud]')

    filename = sys.argv[1]
    port = sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else BAUD

    records = load_records(filename)
    if not records:
        sys.exit('No records found in ' + filename)

    over = [r for r in records if len(r) > BLOCK_SIZE]
    if over:
        sys.exit('Record exceeds BLOCK_SIZE ({} bytes): {!r}'.format(
            BLOCK_SIZE, over[0]))

    ser = serial.Serial(port, baudrate=baud, bytesize=BYTESIZE,
                        parity=PARITY, stopbits=STOPBITS, timeout=READ_POLL)
    try:
        print('Configuring Enquire/Acknowledge handshake...')
        configure_handshake(ser)

        total = len(records)
        for i, record in enumerate(records, 1):
            send_record(ser, record)
            print('[{}/{}] {}'.format(i, total, record))
        print('Done. Sent {} records.'.format(total))
    finally:
        ser.close()


if __name__ == '__main__':
    main()
