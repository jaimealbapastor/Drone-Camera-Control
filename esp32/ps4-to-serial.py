import serial
import serial.tools.list_ports
import time
import sys
import struct
from ps4joystick import PS4Joystick


# Serial port configuration
def get_available_ports():
    ports = serial.tools.list_ports.comports()
    print("PORTS: ", [port.name for port in ports])

    for port in ports:
        try:
            ser = serial.Serial(port.device, 115200)
            ser.close()
            print(port.device, "selected")
            return port.device
        except serial.serialutil.SerialException as e:
            print(f"Could not open {port.device}")

    return None


esp32_port = get_available_ports()

if esp32_port is None:
    print("ESP32 not found. Please check the connection.")
    exit()

ser = serial.Serial(esp32_port, 115200)


def send_data_to_serial(data):
    try:
        packed_data = bytearray()
        packed_data.extend(b"\r")
        
        for value in data:
            if value < 0:
                value = 0
            if value > 65535:
                value = 65535
            packed_data.extend(struct.pack("<H", value))
            
        packed_data.extend(b"\n")
        ser.write(packed_data)
        
    except struct.error as e:
        print(e, "\nValue: ", value)


# PS4 joystick configuration
ps4 = PS4Joystick()

if not ps4.init():
    ps4.quit()
    sys.exit()

ps4.calibrate()

# Script
def linear_correction(values:list, a, b):
    return [int(a*val+b) for val in values]

def stop_script():
    print("\nStopping script...")
    ps4.quit()
    ser.close()
    sys.exit()


last_time = time.time()
while True:
    if ps4.check_stop_button():
        stop_script()

    if time.time() - last_time > 0.09:
        last_time = time.time()

        try:
            joystick_data = ps4.get_important_joystick_data_for_drone_control()
            print("\r", joystick_data, end="")
            corrected_data = linear_correction(joystick_data, 1.60035, -1400.82872)
            send_data_to_serial(corrected_data)

        except PS4Joystick.error_t as e:
            print("\nError reading joystick: ", e)
            break

        except serial.serialutil.SerialException as e:
            print("\nError sending data: ", e)
            break
            

ps4.quit()
ser.close()
