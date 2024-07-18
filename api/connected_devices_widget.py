from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QListWidget, QWidget
import sys, glob, serial

import cv2
import numpy as np
from matplotlib import pyplot as plt


class ConnectedDevicesScreen(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Connected Devices")
        self.layout = QVBoxLayout()

        # List to display connected devices
        self.connected_devices_list = QListWidget()
        connected_devices_groupbox = QGroupBox("Connected Devices")
        connected_devices_layout = QVBoxLayout()
        connected_devices_layout.addWidget(self.connected_devices_list)
        
        # Button to disconnect device
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self.disconnect_device)
        connected_devices_layout.addWidget(self.disconnect_button)
        
        connected_devices_groupbox.setLayout(connected_devices_layout)
        self.layout.addWidget(connected_devices_groupbox)

        # List to display available devices
        self.available_devices_list = QListWidget()
        available_devices_groupbox = QGroupBox("Available Devices")
        available_devices_layout = QVBoxLayout()
        available_devices_layout.addWidget(self.available_devices_list)
        
        # Buttons to connect and update device
        buttons_layout = QHBoxLayout()
        
        # Button connect
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_device)
        buttons_layout.addWidget(self.connect_button)
        
        self.refresh_button = QPushButton("Refresh")
        self.refresh_button.clicked.connect(self.refresh_available_devices)
        buttons_layout.addWidget(self.refresh_button)
        
        available_devices_layout.addLayout(buttons_layout)
        available_devices_groupbox.setLayout(available_devices_layout)
        self.layout.addWidget(available_devices_groupbox)        
        
        self.setLayout(self.layout)
        
        # Update connected devices list
        # self.refresh_available_devices()
        
    
    def serial_ports(self):
        """ Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def refresh_available_devices(self):
        self.available_devices_list.clear()
        for port in self.serial_ports():
            print(port)
            self.available_devices_list.addItem(port)
        
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.available_devices_list.addItem(f"Camera {i}")
            cap.release()
        
    def connect_device(self):
        # Logic to connect the selected device
        pass

    def disconnect_device(self):
        # temporarily use the disconnect button to read from camera
        cap = cv2.VideoCapture(0)
        win_name = 'Camera Preview'
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

        while cv2.waitKey(1) != 27: # Escape
            ret, frame = cap.read()
            if not ret: 
                break
            cv2.imshow(win_name, frame)

        cap.release()
        cv2.destroyWindow(win_name)
        pass
    
    
