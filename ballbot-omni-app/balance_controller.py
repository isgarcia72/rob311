import numpy as np
from pyPS4Controller.controller import Controller
import sys

JOYSTICK_SCALE = 32767

'''
Behavior:
- adjusts Lx attribute with left thumbstick (L3). [left, right] = [-1, 1]
- adjusts Ly attribute with left thubmstick (L3). [down, up] = [-1, 1]
- adjusts Rx attribute with right thumbstick (R3). [left, right] = [-1, 1]
- adjusts Ry attribute with right thubmstick (R3). [down, up] = [-1, 1]
- exits messaging thread on options press
'''
class BalanceController(Controller):
    
    def __init__(self, interface, connecting_using_ds4drv=False, event_definition=None, event_format=None):
        super().__init__(interface, connecting_using_ds4drv, event_definition, event_format)
    
        # initialize member variables to 0
        self.Lx = 0.0
        self.Ly = 0.0
        self.Rx = 0.0
        self.Ry = 0.0

    # LEFT THUMBSTICK BEHAVIOR
    def on_L3_right(self, value):
        # scaled x-axis value
        self.Lx = 1.0 * value/JOYSTICK_SCALE

    def on_L3_left(self, value):
        # scaled x-axis value
        self.Lx = 1.0 * value/JOYSTICK_SCALE

    def on_L3_x_at_rest(self):
        # zeroed center
        self.Lx = 0.0

    def on_L3_up(self, value):
        # inverted and scaled y-axis value
        self.Ly = -1.0 * value/JOYSTICK_SCALE

    def on_L3_down(self, value):
        # inverted and scaled y-axis value
        self.Ly = -1.0 * value/JOYSTICK_SCALE

    def on_L3_y_at_rest(self):
        # zeroed center
        self.Ly = 0.0

    # RIGHT THUMBSTICK BEHAVIOR
    def on_R3_right(self, value):
        # scaled x-axis value
        self.Rx = 1.0 * value/JOYSTICK_SCALE

    def on_R3_left(self, value):
        # scaled x-axis value
        self.Rx = 1.0 * value/JOYSTICK_SCALE

    def on_R3_x_at_rest(self):
        # zeroed center
        self.Rx = 0.0

    def on_R3_up(self, value):
        # inverted and scaled y-axis value
        self.Ry = -1.0 * value/JOYSTICK_SCALE

    def on_R3_down(self, value):
        # inverted and scaled y-axis value
        self.Ry = -1.0 * value/JOYSTICK_SCALE

    def on_R3_y_at_rest(self):
        # zeroed center
        self.Ry = 0.0

    # OPTIONS BEHAVIOR
    def on_options_press(self):
        # exiting thread on options press
        print("Exiting PS4 controller thread.")
        sys.exit()
