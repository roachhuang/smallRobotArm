# pip3 install pyserial
import serial.tools.list_ports

import serial
# import platform
import logging
from threading import Thread, Event

class SerialPort():
    def __init__(self):                
       self.ser = None
       self._event_run = Event()
       self._event_ok2send = Event()
       
    def connect(self):
        port = self._get_serial_port()
        if port:
            self.ser = serial.Serial(port, baudrate=115200, timeout=1)
            print(f"Connected to serial port {self.ser.name}")            
            self._event_ok2send.set()            
            self._event_run.set()
            self.t = Thread(target=self._ReceiveThread, args=[])
            self.t.start()            
        else:
            print("Could not find a suitable serial port.")

    def disconnect(self):
        self.event_run = False
        self.t.join()
        self.ser.close()

    @property
    def event_run(self):
        return self._event_run.is_set()    

    @event_run.setter
    def event_run(self, state):
        if state == True:
            self._event_run.set()
        else:
            self._event_run.clear()

    def _get_serial_port(self):
        """Automatically detects and returns the serial port."""
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if 'USB' in port.description:
                return port.device
        return None

    def send2Arduino(self, cmd: dict) -> None:
    # def send2Arduino(self, header: str, j, bWaitAck: bool):
        """send robot cmd to arduino

        Args:
            ser (_type_): _description_
            header (str): cmd type
            j (float): theta in deg for 6 axes
            bWaitAck (bool): wait for ack from arduino or not
        """
        # msg = '{}{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(cmd['header'], *j)
        msg = '{}{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(cmd['header'], *cmd['joint_angle'])
        self.ser.write(msg.encode('utf-8'))
        self._event_ok2send.clear()
        # print(msg)
        if cmd['ack'] is True:
            # wait till the event is set in rcvThread.
            self._event_ok2send.wait()            
        # while self._event_ack.is_set() and bWaitAck is True:
            # pass

    def _ReceiveThread(self):
        """
        input string is retrieved as a byte string, which works
        differently than a standard string. The decode() method is to convert
        the string from a byte string to a standard string.
        """
        while self.event_run == True:            
            line = self.ser.readline().decode('utf-8')
            if len(line) > 0:
                # get rid of the end of characters /n/r
                string = line.rstrip()
                # logging.warning(string)
                # print(string)
                if string == 'ack':
                    # receive ack frm arduion meaning it is free now
                    logging.debug("Received ack, setting event.")
                    self._event_ok2send.set()

'''
import serial.tools.list_ports
import serial
from threading import Thread, Event

class SerialPortConnection:
    def __init__(self, port_name):
        self.ser = None
        self.port_name = port_name
        
    def connect(self):
        self.ser = serial.Serial(self.port_name, baudrate=115200, timeout=1)
        print(f"Connected to serial port {self.ser.name}")
        
    def disconnect(self):
        self.ser.close()

class SerialPortSender:
    def __init__(self, ser):
        self.ser = ser
        self._event_ok2send = Event()
        
    def send(self, cmd: dict) -> None:
        msg = '{}{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n'.format(cmd['header'], *cmd['joint_angle'])
        self.ser.write(msg.encode('utf-8'))
        print(msg)
        if cmd['ack'] is True:
            self._event_ok2send.wait()
        
class SerialPortReceiver:
    def __init__(self, ser):
        self.ser = ser
        self._event_ok2send = None
        
    def receive(self):
        line = self.ser.readline().decode('utf-8')
        if len(line) > 0:
            string = line.rstrip()
            if string == 'ack':
                self._event_ok2send.set()

class ArduinoController:
    def __init__(self):
        self._event_run = Event()
        self._event_ok2send = Event()
        
    def connect(self):
        port_name = self._get_serial_port()
        if port_name:
            connection = SerialPortConnection(port_name)
            connection.connect()
            self._event_ok2send.set()
            self._event_run.set()
            sender = SerialPortSender(connection.ser)
            receiver = SerialPortReceiver(connection.ser)
            receiver._event_ok2send = self._event_ok2send
            self.t = Thread(target=self._receive_thread, args=[receiver])
            self.t.start()
            return sender
        else:
            print("Could not find a suitable serial port.")
            return None
        
    def disconnect(self):
        self._event_run.clear()
        self.t.join()
        self.connection.disconnect()
        
    @property
    def event_run(self):
        return self._event_run.is_set()
    
    @event_run.setter
    def event_run(self, state):
        if state == True:
            self._event_run.set()
        else:
            self._event_run.clear()
            
    def _get_serial_port(self):
        """Automatically detects and returns the serial port."""
        ports = list(serial.tools.list_ports.comports

'''