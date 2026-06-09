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
# The 7440A only honors an enquiry when its parser is idle between instructions.
# An ENQ that arrives while it is still parsing the record we just sent is
# silently DROPPED (not queued), so a single ENQ can hang forever. This is a
# timing race - which machine wins depends on CPU/driver speed, so it can lurk
# unseen on one Mac and stall constantly on a faster one. Re-sending the
# enquiry at this interval recovers it the moment the parser goes idle.
RE_ENQ = 0.4         # seconds between re-enquiries while waiting for an ACK

# Verbose diagnostic logging (enable with -v). Logs each record, ACK timing,
# and - when an ACK stalls - the plotter's own registers, so a hang is
# explained (e.g. free space stuck below BLOCK_SIZE = buffer deadlock) instead
# of just sitting silent.
VERBOSE = False
STALL_PROBE = 3.0    # seconds without an ACK before we start probing registers
_START = time.time()


def log(msg):
    if VERBOSE:
        print('  [{:7.3f}] {}'.format(time.time() - _START, msg))


def read_register(ser, cmd):
    """Send an immediate device-control query (e.g. b'.B') and return its ASCII
    reply. Device-control instructions are answered right away, even mid-stream.
    A stray ACK byte (0x06) from a pending ENQ is stripped if it shows up."""
    ser.reset_input_buffer()
    ser.write(ESC + cmd)
    ser.flush()
    time.sleep(0.15)
    raw = ser.read(64)
    return raw.replace(b'\x06', b'').decode('ascii', 'replace').strip() or '-'


def probe(ser):
    """One-line snapshot of the plotter's diagnostic registers."""
    return 'err(.E)={} free(.B)={} bufsz(.L)={} status(.O)={}'.format(
        read_register(ser, b'.E'), read_register(ser, b'.B'),
        read_register(ser, b'.L'), read_register(ser, b'.O'))


def configure_handshake(ser):
    """Put the plotter into Enquire/Acknowledge mode (handshake mode 2).

    Opening the serial port toggles DTR/RTS, which the plotter sees as a line
    disturbance; it drops the first byte or two while it settles. If the
    ESC.I config line below lands in that window it is silently lost, the
    plotter never enters ENQ/ACK mode, and every later enquiry times out. So we
    wait for the line to settle, then send the config and *verify* the plotter
    actually answers an ENQ before we start streaming - retrying if it doesn't.
    """
    time.sleep(0.5)               # let the post-open DTR/RTS toggle settle
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    for _ in range(3):
        # ESC.( puts the plotter in the programmed-on state. Harmless if the
        # rear Y/D switch is on D (already on); required if it is on Y.
        ser.write(ESC + b'.(')
        # ESC.R resets all handshake parameters to defaults.
        ser.write(ESC + b'.R')
        # ESC.I<block>;<enq>;<ack>:  -> mode 2: block size, ENQ=5, ACK=6.
        # In mode 2 the plotter sends ONLY the ack string (no output
        # terminator), so each enquiry yields exactly one ACK byte.
        ser.write(ESC + b'.I%d;5;6:' % BLOCK_SIZE)
        ser.flush()
        time.sleep(0.2)
        ser.reset_input_buffer()

        # Verify the config took: a configured plotter ACKs an ENQ in ~20ms.
        ser.write(ENQ)
        ser.flush()
        deadline = time.time() + 2.0
        while time.time() < deadline:
            b = ser.read(1)
            if b and b[0] == ACK:
                return
    raise TimeoutError('Plotter never ACKed after configuring the handshake. '
                       'Check baud/parity switches, cabling, and power.')


def wait_for_ack(ser):
    """Send an enquiry and block until the plotter answers ACK.

    With -v, if no ACK arrives within STALL_PROBE seconds we interrogate the
    plotter's registers and re-send the ENQ, so a stall is diagnosed live: if
    free(.B) is stuck below BLOCK_SIZE the buffer never drains enough to ACK;
    if free(.B) >= BLOCK_SIZE yet still no ACK, the plotter has room but isn't
    answering (fell out of ENQ/ACK mode, or the adapter is dropping the ACK).
    """
    ser.reset_input_buffer()
    ser.write(ENQ)
    ser.flush()
    deadline = time.time() + ACK_TIMEOUT
    next_enq = time.time() + RE_ENQ
    next_probe = time.time() + STALL_PROBE
    while time.time() < deadline:
        b = ser.read(1)
        if b:
            if b[0] == ACK:
                return True
            log('unexpected byte while waiting for ACK: 0x{:02x}'.format(b[0]))
            continue
        now = time.time()
        if VERBOSE and now >= next_probe:
            log('STALL: no ACK after {:.0f}s -> {}'.format(STALL_PROBE, probe(ser)))
            next_probe = now + STALL_PROBE
        if now >= next_enq:
            # Re-ask: the previous ENQ was likely dropped mid-parse. We don't
            # flush input here, so a late ACK to an earlier ENQ is still caught.
            ser.write(ENQ)
            ser.flush()
            next_enq = now + RE_ENQ
        time.sleep(READ_POLL)
    return False


def send_record(ser, record, index=None, total=None):
    """Wait for buffer space, then transmit one HP-GL record."""
    data = record.encode('ascii')
    log('want ACK for record {}/{} ({} bytes): {!r}'.format(
        index, total, len(data), record))
    t0 = time.time()
    if not wait_for_ack(ser):
        sys.exit('No ACK from plotter after {:.0f}s on record {}/{}: {!r}\n'
                 '  Final register read: {}\n'
                 '  Check cabling, baud, power.'.format(
                     ACK_TIMEOUT, index, total, record, probe(ser)))
    log('ACK in {:.3f}s; writing {} bytes'.format(time.time() - t0, len(data)))
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
        sys.exit('Usage: python3 hpgl_stream.py [-v] file.hgl <serial-port> [baud]')

    filename = args[0]
    port = args[1]
    baud = int(args[2]) if len(args) > 2 else BAUD

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
            send_record(ser, record, i, total)
            print('[{}/{}] {}'.format(i, total, record))
        print('Done. Sent {} records.'.format(total))
    finally:
        ser.close()


if __name__ == '__main__':
    main()
