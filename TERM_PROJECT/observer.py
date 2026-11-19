"""Lab 09 observer measurement interface (Step 3, Part 1)."""

from math import pi
from ulab import numpy as np


class Observer:
    """Read accumulated wheel displacement for one motor observer."""

    def __init__(self, encoder, ticks_per_wheel_rev):
        """Use the same encoder and scale as the motor velocity controller."""

        if not isinstance(ticks_per_wheel_rev, int) or ticks_per_wheel_rev <= 0:
            raise ValueError("Encoder ticks per wheel revolution must be a positive integer")

        self.encoder = encoder
        self.ticks_per_radian = ticks_per_wheel_rev / (2.0 * pi)

    def get_displacement_rad(self):
        """Return signed wheel displacement from the encoder's zero."""

        # The motor task updates the encoder once before reading this value.
        # The driver already handles 16-bit rollover and applies polarity.
        # Keep the cumulative count; wrapping by one revolution loses distance.
        position_ticks = self.encoder.get_position()
        return position_ticks / self.ticks_per_radian


def week1_numeric_check(Ao, Bo, Co, Do):
    """Return the current output and next state for the lab's fixed check."""

    # Use the selected MATLAB matrices. This check does not access hardware
    # or run as part of the motor control loop.
  

    # State order: velocity (rad/s), displacement (rad), disturbance (rad/s^2).
    state = np.array([[1.0], [0.2], [-0.5]])
    # Input order: limited voltage (V), measured displacement (rad).
    inputs = np.array([[2.0], [0.21]])

    current_output = np.dot(Co, state) + np.dot(Do, inputs)
    next_state = np.dot(Ao, state) + np.dot(Bo, inputs)

    return current_output, next_state
