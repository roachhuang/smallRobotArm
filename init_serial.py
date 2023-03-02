import serial.tools.list_ports
import serial


def init_ser():
    """
    Initializes a serial connection to an Arduino board.

    Returns:
        A `serial.Serial` object representing the serial connection, or `None` if no suitable
        serial port is found or if the connection fails.
    """
    ports = list(serial.tools.list_ports.comports())
    ser = None

    if not ports:
        print("No serial ports found.")
        return None

    # Skip over COM1 port, if available
    ports = [p for p in ports if "COM1" not in p.name]

    for port in ports:
        try:
            ser = serial.Serial(port.name, 115200, timeout=.1)
            break
        except (OSError, serial.SerialException):
            print(f"Could not connect to serial port {port.name}")

    if not ser:
        print("Failed to connect to any serial port.")
        return None

    print(f"Connected to serial port {ser.name}")
    return ser
