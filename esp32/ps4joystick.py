import time
import pygame

PS4_CONTROLLER = {
    "axis": [
        "L3_horizontal",
        "L3_vertical",
        "R3_horizontal",
        "R3_vertical",
        "L2",
        "R2",
    ],
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


class PS4Joystick:

    error_t = pygame.error

    # Axis index in joystick
    THRUST_I = 5
    PITCH_I = 1
    ROLL_I = 0
    YAW_I = 2

    TRANSMITTED_AXIS = [
        ROLL_I,
        PITCH_I,
        THRUST_I,
        YAW_I,
    ]  # Order in which they are sent
    AXIS_INV = {THRUST_I: 1, YAW_I: 1, ROLL_I: 1, PITCH_I: -1}
    AXIS_SENSIBILITY = {THRUST_I: 1, YAW_I: 0.7, ROLL_I: 0.7, PITCH_I: 0.7}

    # Buttons index in joystick
    X_I = 0
    O_I = 1
    S_I = 2
    T_I = 3
    L1 = 9
    R1 = 10
    UP = 11
    DOWN = 12
    LEFT = 13
    RIGHT = 14
    TRANSMITTED_BUTTONS = [
        UP,
        RIGHT,
        DOWN,
        LEFT,
        L1,
        R1,
        T_I,
        O_I,
        X_I,
        S_I,
    ]  # Order in which they are sent
    STATE_BUTTONS = [UP, RIGHT, DOWN, LEFT]

    def __init__(self) -> None:
        self.js_ = None
        self.axis_offsets_ = {
            self.THRUST_I: 0,
            self.YAW_I: 0,
            self.ROLL_I: 0,
            self.PITCH_I: 0,
        }

        self.previous_button_states = {button: 0 for button in self.STATE_BUTTONS}
        self.aux_states_ = {button: 0 for button in self.STATE_BUTTONS}

    def init(self) -> bool:
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick connected")
            return False

        self.js_ = pygame.joystick.Joystick(0)
        self.js_.init()

        if self.js_.get_name() != "PS4 Controller":
            print("Controller not supported:", self.js_.get_name())
            print("Only PS4 Controller supported")
            return False
        return True

    def quit(self):
        pygame.quit()

    def calibrate(self, sec=2):
        print("Calibrating offsets... ", end="")

        start_time = time.time()
        while time.time() - start_time < sec:
            data = self.get_important_joystick_data_for_drone_control()

        for i, joystick_i in enumerate(self.TRANSMITTED_AXIS):
            if joystick_i == self.THRUST_I:
                self.axis_offsets_[joystick_i] = data[i] - 1000
            else:
                self.axis_offsets_[joystick_i] = data[i] - 1500

        print(self.axis_offsets_, "Done")

    def get_important_joystick_data_for_drone_control(self):

        pygame.event.pump()

        data = [self.axisPS4toFC(axis) for axis in self.TRANSMITTED_AXIS]
        data += [self.buttonPS4toFC(button) for button in self.TRANSMITTED_BUTTONS]

        return data

    def axisPS4toFC(self, index):
        x = (
            self.AXIS_INV[index]
            * self.js_.get_axis(index)
            * self.AXIS_SENSIBILITY[index]
        )
        return int(x * 500 + 1500 - self.axis_offsets_[index])

    def buttonPS4toFC(self, index):
        val = self.js_.get_button(index)  # can be 0 or 1

        if index in self.STATE_BUTTONS:
            if val == 1 and self.previous_button_states[index] == 0:
                self.aux_states_[index] = -self.aux_states_[index] + 1
                
            self.previous_button_states[index] = val
            val = self.aux_states_[index]
            
        return int(val* 1000 + 1000)

    def get_joystick_data(self):

        pygame.event.pump()

        data = {
            "axes": [self.js_.get_axis(i) for i in range(self.js_.get_numaxes())],
            "buttons": [
                self.js_.get_button(i) for i in range(self.js_.get_numbuttons())
            ],
        }
        return data

    def check_stop_button(self) -> bool:
        pygame.event.pump()
        if self.js_.get_button(5):
            return True
        return False
