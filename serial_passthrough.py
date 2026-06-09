"""serial_passthrough.py - Quick interactive test: type lines, send them to the
plotter over RS-232. No handshake, no flow control - just a raw passthrough so
you can poke commands at the HP 7440A by hand (e.g. IN; SP1; PA1000,1000;).

Whatever you type is sent verbatim (a trailing newline is added). Anything the
plotter sends back is printed. Ctrl-D (EOF) or Ctrl-C exits.

Requires pyserial:  python3 -m pip install pyserial

Usage:
    python3 serial_passthrough.py <serial-port> [baud]
    python3 serial_passthrough.py /dev/tty.usbserial-XXXX 9600
"""

import sys

try:
    import serial  # pyserial
except ImportError:
    sys.exit('pyserial not installed. Run: python3 -m pip install pyserial')

BAUD = 9600
BYTESIZE = 8
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
READ_TIMEOUT = 0.1   # seconds; short so we can poll for replies between lines


def main():
    if len(sys.argv) < 2:
        sys.exit('Usage: python3 serial_passthrough.py <serial-port> [baud]')

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD

    ser = serial.Serial(port, baudrate=baud, bytesize=BYTESIZE,
                        parity=PARITY, stopbits=STOPBITS, timeout=READ_TIMEOUT)
    print('Connected to {} at {} baud. Type commands; Ctrl-D to quit.'.format(
        port, baud))
    try:
        while True:
            try:
                line = input('> ')
            except EOFError:
                print()
                break
            ser.write((line + '\n').encode('ascii'))
            ser.flush()
            reply = ser.read(256)
            if reply:
                print('<-', reply)
    except KeyboardInterrupt:
        print()
    finally:
        ser.close()


if __name__ == '__main__':
    main()
