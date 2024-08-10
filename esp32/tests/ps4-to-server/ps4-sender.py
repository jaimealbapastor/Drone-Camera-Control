import pygame
import requests
import time
import sys

ESP32_SERVER_URL = "http://192.168.1.130/controller"

PS4_CONTROLLER = {
    "axis": ["yaw", "thrust", "roll", "pitch", "L2", "R2"],
    "buttons": [
        "X",
        "O",
        "S",
        "T",
        "share",
        "ps",
        "options",
        "L3",
        "R3",
        "L1",
        "R1",
        "up",
        "down",
        "left",
        "right",
        "trackpad",
    ],
}

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick connected")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

if joystick.get_name() != "PS4 Controller":
    print("Controller not supported:", joystick.get_name())
    print("Only PS4 Controller supported")
    sys.exit()


def get_joystick_data():

    pygame.event.pump()

    data = {
        "axes": [joystick.get_axis(i) for i in range(joystick.get_numaxes())],
        "buttons": [joystick.get_button(i) for i in range(joystick.get_numbuttons())],
    }
    return data


# Axis index in joystick
YAW_I = 0
THRUST_I = 1
ROLL_I = 2
PITCH_I = 3

TRANSMITTED_AXIS = [ROLL_I, PITCH_I, THRUST_I, YAW_I]  # Order in which they are sent
AXIS_CENTERS = {THRUST_I: 1000, YAW_I: 1500, ROLL_I: 1500, PITCH_I: 1500}
AXIS_OFFSETS = {THRUST_I: 0, YAW_I: 0, ROLL_I: 0, PITCH_I: 0}
AXIS_INV = {THRUST_I: -1, YAW_I: 1, ROLL_I: 1, PITCH_I: 1}

# Buttons index in joystick
X_I = 0
O_I = 1
S_I = 2
T_I = 3
TRANSMITTED_BUTTONS = [X_I, O_I, S_I, T_I]  # Order in which they are sent


def axisPS4toFC(index):
    return int(
        AXIS_INV[index] * joystick.get_axis(index) * 500
        + AXIS_CENTERS[index]
        - AXIS_OFFSETS[index]
    )


def buttonPS4toFC(index):
    return int(joystick.get_button(index) * 1000 + 1000)


def get_important_joystick_data_for_drone_control():
    pygame.event.pump()

    data = [axisPS4toFC(axis) for axis in TRANSMITTED_AXIS]
    data += [buttonPS4toFC(button) for button in TRANSMITTED_BUTTONS]

    return data


def calibrate():
    global AXIS_OFFSETS
    print("Calibrating offsets...")

    start_time = time.time()
    while time.time() - start_time < 3:
        data = get_important_joystick_data_for_drone_control()

    for i, joystick_i in enumerate(TRANSMITTED_AXIS):
        AXIS_OFFSETS[joystick_i] = data[i] - AXIS_CENTERS[joystick_i]

    print(AXIS_OFFSETS)
    print("Done")


def send_data_to_server(data):
    try:
        response = requests.post(ESP32_SERVER_URL, json=data)
        response.raise_for_status()  # Raise an exception for bad status codes
        # print("Data sent:", data)
        # print("Server response:", response.text)
    except requests.exceptions.RequestException as e:
        print("Error sending data:", e)


def stop_script():
    print("Stopping script...")
    pygame.quit()
    sys.exit()


calibrate()
print(AXIS_OFFSETS)

last_time = time.time()

while True:

    pygame.event.pump()
    if joystick.get_button(5):
        stop_script()

    if time.time() - last_time > 0.05:
        last_time = time.time()

        try:
            joystick_data = get_important_joystick_data_for_drone_control()
            print(joystick_data)
            send_data_to_server(joystick_data)
        except pygame.error as e:
            print("Error reading joystick:", e)
            break
