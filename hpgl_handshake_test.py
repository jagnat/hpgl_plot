"""hpgl_handshake_test.py - Diagnose why the Enquire/Acknowledge handshake in
hpgl_stream.py times out on a given Mac/plotter, even though plain drawing and
output commands (OI;/OA;) work.

Since OI;/OA; already return data, both directions of the serial link work, so
the failure is in the handshake *setup*, not the wiring. This script walks that
setup step by step and prints every byte the plotter sends back in hex:

  * Read the error register (ESC.E) BEFORE and AFTER sending the ESC.I config.
    If the AFTER value is non-zero, the plotter rejected the config line -> it
    never entered Enquire/Acknowledge mode, which is why no ACK ever comes.
  * Read the buffer-size register (ESC.L) so we can see whether the plotter can
    ever have BLOCK_SIZE (58) bytes free. If its buffer is smaller than the
    requested block, it will *correctly* never ACK -> shrink BLOCK_SIZE.
  * Send a raw ENQ several times and show the raw reply (expect a single 0x06).

Requires pyserial:  python3 -m pip install pyserial

Usage:
    python3 hpgl_handshake_test.py <serial-port> [baud]
    python3 hpgl_handshake_test.py /dev/tty.usbserial-XXXX 9600
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

ESC = b'\x1b'
ENQ = b'\x05'
ACK = 0x06
BLOCK_SIZE = 58

ACK_TIMEOUT = 5.0    # seconds to wait for an ACK in the live handshake test
READ_POLL = 0.05


def show(label, raw):
    """Print a reply as both hex and printable ASCII."""
    hexs = ' '.join('%02x' % b for b in raw) if raw else '(nothing)'
    text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw)
    print('  {:<30} <- {:<30} {!r}'.format(label, hexs, text))


def ask(ser, label, data, wait=0.3, n=64):
    """Send bytes, wait, read up to n bytes, print what came back."""
    ser.reset_input_buffer()
    ser.write(data)
    ser.flush()
    time.sleep(wait)
    show(label, ser.read(n))


def timed_ack(ser, label):
    """Send ENQ and poll for an ACK exactly like hpgl_stream.wait_for_ack,
    reporting how long it took (or that it timed out)."""
    ser.reset_input_buffer()
    ser.write(ENQ)
    ser.flush()
    start = time.time()
    deadline = start + ACK_TIMEOUT
    got = b''
    while time.time() < deadline:
        b = ser.read(1)
        if b:
            got += b
            if b[0] == ACK:
                show('%s (ACK in %.2fs)' % (label, time.time() - start), got)
                return True
        else:
            time.sleep(READ_POLL)
    show('%s (TIMEOUT after %.1fs)' % (label, ACK_TIMEOUT), got)
    return False


def main():
    if len(sys.argv) < 2:
        sys.exit('Usage: python3 hpgl_handshake_test.py <serial-port> [baud]')
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD

    ser = serial.Serial(port, baudrate=baud, bytesize=BYTESIZE,
                        parity=PARITY, stopbits=STOPBITS, timeout=READ_POLL)
    print('Connected to {} at {} baud.\n'.format(port, baud))
    try:
        print('1. Sanity check (should match your working OI;/OA; test):')
        ask(ser, 'OI; (identify)', b'OI;')
        ask(ser, 'OA; (actual position)', b'OA;')

        print('\n2. Status / buffer registers before any config:')
        ask(ser, 'ESC.E (error register)', ESC + b'.E')
        ask(ser, 'ESC.L (output buffer size)', ESC + b'.L')
        ask(ser, 'ESC.B (buffer space free)', ESC + b'.B')
        ask(ser, 'ESC.O (extended status)', ESC + b'.O')

        print('\n3. Apply Enquire/Acknowledge config, then re-read error reg:')
        ser.write(ESC + b'.(')
        ser.write(ESC + b'.R')
        ser.write(ESC + b'.I%d;5;6:' % BLOCK_SIZE)
        ser.flush()
        time.sleep(0.2)
        ser.reset_input_buffer()
        ask(ser, 'ESC.E (error AFTER config)', ESC + b'.E')

        print('\n4. Live handshake - send ENQ, expect a single 0x06 ACK:')
        for i in range(1, 4):
            timed_ack(ser, 'ENQ attempt %d' % i)

        print('\nInterpretation:')
        print('  - ESC.E AFTER config != 0   -> plotter rejected ESC.I config.')
        print('  - ESC.L buffer size < 58    -> shrink BLOCK_SIZE below that.')
        print('  - ENQ returns 0x06          -> handshake works; bug is elsewhere.')
        print('  - ENQ returns other bytes   -> parity/baud or wrong ACK char.')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
