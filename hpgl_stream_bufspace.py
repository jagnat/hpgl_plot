"""hpgl_stream_bufspace.py - Stream an HP-GL record file to an HP 7440A over
RS-232 using the plotter's *buffer-space* handshake instead of Enquire/Acknowledge.

Why not ENQ/ACK (hpgl_stream.py)? On some Macs the 7440A intermittently refuses
to answer an ENQ even with an empty buffer and no error, so the handshake stalls
or times out. But its *device-control* query ESC.B - "how many bytes are free in
your input buffer?" - is answered immediately and reliably, even while the pen is
mid-draw. So we use that as the flow-control signal:

    before sending a record of N bytes, poll ESC.B until the plotter reports at
    least N bytes free, then send the record.

Because the plotter only frees buffer space as it *finishes executing* each
instruction, a long line (slow to draw) holds the count down until the pen
catches up - so this paces itself exactly to the plot, with no fixed delay to
guess at and no risk of overrunning the tiny buffer.

This avoids ENQ/ACK entirely, so it works on machines where that handshake is
flaky while still giving proper flow control (unlike the fixed-delay
hpgl_stream_timed.py, which overruns on long-drawing records).

Requires pyserial:  python3 -m pip install pyserial

Usage:
    python3 hpgl_stream_bufspace.py [-v] file.hgl <serial-port> [baud]
    python3 hpgl_stream_bufspace.py output.hgl /dev/tty.usbserial-XXXX 9600
"""

import re
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

# How many extra free bytes to require beyond the record length before sending,
# as a guard against off-by-one differences in the plotter's own accounting.
MARGIN = 2

SPACE_TIMEOUT = 30.0  # seconds to wait for buffer space before giving up
SPACE_POLL = 0.03     # seconds between ESC.B polls while waiting for space
READ_POLL = 0.05      # serial read timeout

VERBOSE = False
_START = time.time()


def log(msg):
    if VERBOSE:
        print('  [{:7.3f}] {}'.format(time.time() - _START, msg))


def drain_input(ser):
    """Discard any pending input by READING it.

    We deliberately do NOT use reset_input_buffer() here: on some macOS FTDI
    drivers its underlying tcflush() corrupts the OUTBOUND data path when called
    between plot-data writes, so the plotter receives garbage and never draws
    (confirmed with hpgl_diag.py: the interleaved query breaks streaming only
    when it flushes the input buffer). A plain read clears stale bytes without
    touching tcflush, so it is safe to interleave with data."""
    n = ser.in_waiting
    if n:
        ser.read(n)


def query_int(ser, cmd, timeout=1.0):
    """Send a device-control query (e.g. b'.B') and return its integer reply.

    The plotter answers these immediately - even mid-draw - with ASCII digits
    terminated by CR. A stray byte is tolerated: we extract the first integer we
    find. Returns None if nothing parseable comes back in `timeout` seconds."""
    drain_input(ser)
    ser.write(ESC + cmd)
    ser.flush()
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            if buf:
                break          # had data, then a gap -> reply is complete
            continue
        if b in (b'\r', b'\n'):
            break
        buf += b
    m = re.search(rb'-?\d+', buf)
    return int(m.group()) if m else None


def wait_for_space(ser, need):
    """Poll ESC.B until the plotter reports at least `need` free bytes.

    Returns the free count once satisfied, or None on timeout. Each poll is a
    device-control query, which also keeps the plotter's parser responsive."""
    deadline = time.time() + SPACE_TIMEOUT
    last = None
    while time.time() < deadline:
        free = query_int(ser, b'.B')
        if free is not None:
            last = free
            if free >= need:
                return free
        time.sleep(SPACE_POLL)
    log('TIMEOUT waiting for {} free bytes (last reported {})'.format(need, last))
    return None


def send_record(ser, record, index, total):
    """Wait for enough buffer space, then transmit one HP-GL record."""
    data = record.encode('ascii')
    need = len(data) + MARGIN
    t0 = time.time()
    free = wait_for_space(ser, need)
    if free is None:
        sys.exit('No buffer space for {} bytes after {:.0f}s on record {}/{}: '
                 '{!r}\n  Check cabling, baud, power.'.format(
                     need, SPACE_TIMEOUT, index, total, record))
    log('record {}/{}: {} free after {:.3f}s, writing {} bytes: {!r}'.format(
        index, total, free, time.time() - t0, len(data), record))
    ser.write(data)
    ser.flush()


def load_records(filename):
    with open(filename) as f:
        return [line.strip() for line in f if line.strip()]


def main():
    global VERBOSE
    raw_args = sys.argv[1:]
    args = [a for a in raw_args if a not in ('-v', '--verbose')]
    VERBOSE = len(args) != len(raw_args)

    if len(args) < 2:
        sys.exit('Usage: python3 hpgl_stream_bufspace.py [-v] file.hgl '
                 '<serial-port> [baud]')

    filename = args[0]
    port = args[1]
    baud = int(args[2]) if len(args) > 2 else BAUD

    records = load_records(filename)
    if not records:
        sys.exit('No records found in ' + filename)

    ser = serial.Serial(port, baudrate=baud, bytesize=BYTESIZE,
                        parity=PARITY, stopbits=STOPBITS, timeout=READ_POLL)
    try:
        # Let the post-open DTR/RTS toggle settle, then put the plotter in the
        # programmed-on state and reset handshake params to defaults. We do NOT
        # configure ENQ/ACK mode - we rely solely on the ESC.B query, which works
        # without any handshake setup. Note: we never call reset_input_buffer()/
        # reset_output_buffer() - tcflush corrupts outbound data on some FTDI
        # drivers (see drain_input). We drain by reading instead.
        time.sleep(0.5)
        ser.write(ESC + b'.(')      # programmed-on (harmless if already on)
        ser.write(ESC + b'.R')      # reset handshake parameters to defaults
        ser.flush()
        time.sleep(0.2)
        drain_input(ser)

        bufsz = query_int(ser, b'.L')
        if bufsz is None:
            sys.exit('Plotter did not answer ESC.L (buffer size). '
                     'Check baud/parity switches, cabling, and power.')
        print('Plotter input buffer: {} bytes.'.format(bufsz))

        over = [r for r in records if len(r) + MARGIN > bufsz]
        if over:
            sys.exit('Record too big for {}-byte buffer: {!r}'.format(
                bufsz, over[0]))

        total = len(records)
        for i, record in enumerate(records, 1):
            send_record(ser, record, i, total)
            print('[{}/{}] {}'.format(i, total, record))
        print('Done. Sent {} records.'.format(total))
    finally:
        ser.close()


if __name__ == '__main__':
    main()
