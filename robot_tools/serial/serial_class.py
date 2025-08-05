"""Serial Communication Module

This module provides serial communication functionality for the small robot arm,
handling the connection to the Arduino controller and implementing a reliable
communication protocol with acknowledgment.

Classes:
    SerialPort: Manages serial communication with the Arduino
"""

import serial.tools.list_ports
from serial import Serial, SerialException, SerialTimeoutException
import logging
from threading import Thread, Event, Lock
import robot_tools.misc.helpers as hlp

class SerialPort:
    """Serial port manager for Arduino communication.
    
    This class handles the serial connection to the Arduino, including automatic port
    detection, command sending with acknowledgment, and response handling through
    a dedicated receiver thread.
    
    Attributes:
        ser: Serial connection object
        _event_run: Event to control the receiver thread
        _event_ok2send: Event for flow control (wait for acknowledgment)
        lock: Thread lock for thread-safe operations
        t: Receiver thread
    """
    
    def __del__(self):
        # Ensure resources are cleaned up if disconnect was not called explicitly
        self.disconnect()
        
    def __init__(self):
        self.ser = None
        self._event_run = Event()
        self._event_ok2send = Event()
        self.lock = Lock()
        self.t = None  # Ensure thread is tracked properly
        logging.basicConfig(
            level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
        )

    def connect(self) -> bool:
        """Connects to the first available serial port that matches expected descriptions."""
        port = self._get_serial_port()
        if port:
            try:
                self.ser = serial.Serial(port, baudrate=115200, timeout=1)
                print(f"[INFO] Connected to serial port {self.ser.name}")
                self._event_ok2send.set()
                self._event_run.set()
                # Ensure a new thread is created only if one is not running
                if not self.t or not self.t.is_alive():
                    self.t = Thread(target=self._ReceiveThread, daemon=True)
                    self.t.start()
                return True
            except serial.SerialException as e:
                logging.error(f"Serial connection error: {e}")
                return False  # Connection failed
            except OSError as e:
                logging.error(f"OS error during serial connection: {e}")
                return False  # Connection failed
        else:
            print("[Error] Could not find a suitable serial port.")
            return False

    def disconnect(self):
        """Safely closes the serial connection and stops the receiving thread."""
        self._event_run.clear()
        # check if thread is running before joining.
        if self.ser and self.t and self.t.is_alive():
            self.t.join(timeout=2)  # ensure thread stops within 2 sec.
            self.t = None
        # check if serial port is open before closing.
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("[INFO] Serial port closed successfully.")
            except serial.SerialException as e:
                logging.error(f"Error closing serial port: {e}")

        self.t = None  # Reset thread reference
        # self.event_run = False
        # self.t.join()
        # self.ser.close()

    @property
    def event_run(self):
        """Get the state of the run event.
        
        Returns:
            bool: True if the receiver thread should be running
        """
        return self._event_run.is_set()

    @event_run.setter
    def event_run(self, state):
        """Set the state of the run event.
        
        Args:
            state (bool): True to set the event, False to clear it
        """
        if state:
            self._event_run.set()
        else:
            self._event_run.clear()

    def _get_serial_port(self):
        """Automatically detect and return the first available suitable serial port.
        
        Looks for ports with 'USB' in the description or 'COM' in the device name.
        
        Returns:
            str: Device path of the detected serial port, or None if not found
        """
        """Automatically detects and returns the 1st available serial port."""
        try:
            ports = list(serial.tools.list_ports.comports())
            for port in ports:
                if "USB" in port.description or "COM" in port.device:
                    return port.device
        except Exception as e:
            logging.error(f"Error getting serial ports: {e}")
        return None
    
    @hlp.timer
    def send2Arduino(self, cmd: dict) -> None:
        """
        constrain the speed of sending a CMD to arduino in about 10~20ms. avoid faster than it!!! 
        send robot cmd to arduino
        Args:
            ser (_type_): _description_
            header (str): cmd type
            j (float): theta in deg for 6 axes
            bWaitAck (bool): wait for ack from arduino or not
        """
        if self.ser and self.ser.is_open:
            msg = "{}{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
                cmd["header"], *cmd["joint_angle"]
            )
            try:
                # self.i = 0
                # self._event_ok2send.set()
                # while (self.i < 1): # retry if buffer is full
                #     if self._event_ok2send:
                #         self.ser.write(msg.encode("utf-8"))
                #         self._event_ok2send.clear()
                        # time.sleep(0.05)
                
                # logging.debug(f"Sent command: {msg.strip()}")
                if cmd["ack"]:
                    self.ser.write(msg.encode("utf-8"))                           
                    self._event_ok2send.clear()
                    # print("[DEBUG] Waiting for ack...")
                    # timeout must be >= 15
                    if not self._event_ok2send.wait(timeout=20):  # timeout added.
                        logging.error("Timeout waiting for Arduino acknowledgement.")
                       
            except serial.SerialTimeoutException:
                logging.error("Serial write timeout.")
            except serial.SerialException as e:
                logging.error(f"Serial write error: {e}")
        else:
            logging.error("Serial port not connected. Can't send cmd")

    def _ReceiveThread(self):
        """
        Listens for responses from Arduino and sets acknowledgment event if needed.
        input string is retrieved as a byte string, which works
        differently than a standard string. The decode() method is to convert
        the string from a byte string to a standard string.
        """
        while self.event_run:
            if self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode("utf-8").rstrip()
                    if line:
                        # print(f"[DEBUG] received: {line}")
                        if line == "ack":   #ack is received
                            self._event_ok2send.set()
                        elif line == 'buffer_full':
                            self._event_ok2send.clear()
                        #     logging.debug(
                        #         f"Received from Arduino: {line}"
                        #     )  # add debug line.
                except serial.SerialTimeoutException:
                    pass  # timeout is normal.
                except serial.SerialException as e:
                    logging.error(f"Serial read error: {e}")
                    break  # exit thread on error.
                except UnicodeDecodeError:
                    logging.error("Unicode decoding error from serial data.")
                    break  # exit thread on error.
            else:
                break  # exit thread if serial port is closed.
