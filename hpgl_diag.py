"""hpgl_diag.py - Isolate which behavior breaks streaming on a given Mac.

We know (empirically) that on the affected machine:
  * data writes alone draw fine        (serial_passthrough.py, hpgl_stream_timed.py)
  * device-control queries alone work  (hpgl_handshake_test.py)
  * but the moment a streamer INTERLEAVES queries between data writes
    (hpgl_stream.py, hpgl_stream_bufspace.py) the plotter stops moving.

So the fault is one specific behavior the failing scripts add on top of the
working timed script. This tool starts from that working baseline - send each
command, then a fixed delay - and lets you switch ON one suspect behavior at a
time. It draws a big, obvious square so "did it work?" needs no interpretation.

Run the matrix and note which flag first stops the square from drawing:

    # 1. baseline (should draw - confirms the harness itself works)
    python3 hpgl_diag.py /dev/cu.usbserial-XXXX

    # 2. add ESC.R to the preamble
    python3 hpgl_diag.py /dev/cu.usbserial-XXXX --reset-hs

    # 3. add the startup buffer flush
    python3 hpgl_diag.py /dev/cu.usbserial-XXXX --reset-start

    # 4. interleave an ESC.B query between commands (WITH reset_input_buffer,
    #    exactly like hpgl_stream_bufspace.py)
    python3 hpgl_diag.py /dev/cu.usbserial-XXXX --query

    # 5. same interleaved query but WITHOUT reset_input_buffer
    python3 hpgl_diag.py /dev/cu.usbserial-XXXX --query --no-input-reset

The first flag that breaks the square is the culprit. Flags stack, so you can
also combine them to confirm.

Requires pyserial:  python3 -m pip install pyserial

Usage:
    python3 hpgl_diag.py <serial-port> [baud] [flags]
"""

import re
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

# A big, unmistakable square plus a diagonal - impossible to miss if it draws,
# and obviously absent if it doesn't. Coordinates well within the 7440A range.
SQUARE = [
    'IN;',
    'SP1;',
    'PA500,500;',
    'PD7000,500;',
    'PD7000,7000;',
    'PD500,7000;',
    'PD500,500;',
    'PD7000,7000;',   # diagonal across the square
    'PU;',
    'SP0;',
]

DELAY = 0.6          # fixed per-command settle, same idea as hpgl_stream_timed.py
READ_POLL = 0.05


def query_b(ser, do_input_reset):
    """Send ESC.B and read the integer reply - the interleaved query under test.
    `do_input_reset` toggles the reset_input_buffer() call that the real
    streamers make before every query (suspect #2)."""
    if do_input_reset:
        ser.reset_input_buffer()
    ser.write(ESC + b'.B')
    ser.flush()
    deadline = time.time() + 1.0
    buf = b''
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            if buf:
                break
            continue
        if b in (b'\r', b'\n'):
            break
        buf += b
    m = re.search(rb'-?\d+', buf)
    return int(m.group()) if m else None


def main():
    args = sys.argv[1:]
    flags = {a for a in args if a.startswith('--')}
    pos = [a for a in args if not a.startswith('--')]
    if not pos:
        sys.exit('Usage: python3 hpgl_diag.py <serial-port> [baud] [flags]\n'
                 'Flags: --reset-hs --reset-start --query --no-input-reset')

    port = pos[0]
    baud = int(pos[1]) if len(pos) > 1 else BAUD

    use_reset_hs = '--reset-hs' in flags
    use_reset_start = '--reset-start' in flags
    use_query = '--query' in flags
    do_input_reset = '--no-input-reset' not in flags  # default: reset (like streamers)

    print('Port {} @ {} baud'.format(port, baud))
    print('  ESC.R preamble       : {}'.format(use_reset_hs))
    print('  startup buffer flush : {}'.format(use_reset_start))
    print('  interleaved ESC.B    : {}{}'.format(
        use_query, '' if not use_query else
        (' (with reset_input_buffer)' if do_input_reset
         else ' (NO reset_input_buffer)')))
    print()

    ser = serial.Serial(port, baudrate=baud, bytesize=BYTESIZE,
                        parity=PARITY, stopbits=STOPBITS, timeout=READ_POLL)
    try:
        time.sleep(0.5)                     # let DTR/RTS settle after open
        if use_reset_start:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        ser.write(ESC + b'.(')              # programmed-on (timed script does this)
        if use_reset_hs:
            ser.write(ESC + b'.R')          # SUSPECT #1
        ser.flush()
        time.sleep(0.2)

        for i, cmd in enumerate(SQUARE, 1):
            ser.write(cmd.encode('ascii'))
            ser.flush()
            print('[{}/{}] {}'.format(i, len(SQUARE), cmd))
            if use_query:                   # SUSPECT #3 (interleaved query)
                free = query_b(ser, do_input_reset)
                print('        ESC.B -> {}'.format(free))
            time.sleep(DELAY)

        print('\nDone. Did the square + diagonal draw? (y/n)')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
