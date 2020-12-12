import serial, time, sys
from xmodem import XMODEM
import serials_find


class PassthroughEnabled(Exception):
    pass


def dbg_print(line=''):
    sys.stdout.write(line + '\n')
    sys.stdout.flush()


def bf_passthrough_init(port, requestedBaudrate, half_duplex=False):
    sys.stdout.flush()
    dbg_print("======== PASSTHROUGH INIT ========")
    dbg_print("  Trying to initialize %s @ %s" % (port, requestedBaudrate))

    s = serial.Serial(port=port, baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=1, xonxoff=0, rtscts=0)
    s.reset_input_buffer()

    cnt = s.write("#".encode('utf-8'))
    s.flush()
    if half_duplex:
        s.read(cnt)
    start = s.read_until("#".encode('utf-8'))
    if not start:
        raise PassthroughEnabled("No CLI available. Already in passthrough mode?")
    try:
        start = start.decode('utf-8')
    except UnicodeDecodeError:
        raise PassthroughEnabled("Invalid response!")

    inChars = ""
    SerialRXindex = ""

    dbg_print("\nAttempting to detect FC UART configuration...")

    s.timeout = 5
    s.reset_input_buffer()
    cnt = s.write("serial\r\n".encode('utf-8'))
    s.flush()
    if half_duplex:
        s.read(cnt)

    while True:
        try:
            line = s.readline().decode('utf-8')
        except UnicodeDecodeError:
            continue
        line = line.strip()
        #print("FC: '%s'" % line)
        if not line or line is "#":
            break

        if line.startswith("serial"):
            line = line.strip()
            dbg_print("  '%s'" % line)

            # Searching: 'serial <index> 64 ...'
            config = line.split()
            try:
                if config[2] == "64":
                    dbg_print("    ** Serial RX config detected: '%s'" % line)
                    SerialRXindex = config[1]
                    break
            except IndexError:
                pass

    dbg_print()

    if not SerialRXindex:
        dbg_print("Failed to make contact with FC, possibly already in passthrough mode?")
        dbg_print("If the next step fails please reboot FC")
        dbg_print()
        raise PassthroughEnabled("FC connection failed");

    cmd = "serialpassthrough %s %s" % (SerialRXindex, requestedBaudrate, )

    dbg_print("Setting serial passthrough...")
    dbg_print("  CMD: '%s'" % cmd)
    s.write((cmd + '\n').encode('utf-8'))
    time.sleep(1)

    s.flush()
    s.close()

    dbg_print("======== PASSTHROUGH DONE ========")


if __name__ == '__main__':
    try:
        requestedBaudrate = int(sys.argv[1])
    except:
        requestedBaudrate = 420000
    port = serials_find.get_serial_port()
    bf_passthrough_init(port, requestedBaudrate)
