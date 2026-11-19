"""Discrete wheel observer for Lab 09 Design A at the baseline period."""

from array import array
from math import pi


# Design A: Km = 2.887 (rad/s)/V, tau = 0.22 s. Discretize the
# augmented plant at 10 ms, then place the selected continuous error
# poles after mapping them into discrete time. See the MATLAB source.
OBSERVER_PERIOD_MS = 10
Ao = (
    (0.9555630362682842, -6.493845292982613, 0.009776132020977461),
    (0.009776132020977464, 0.4659332238424003, 0.00004925095538495825),
    (0.0, -8.342993098117972, 1.0),
)
Bo = (
    (0.12828951429346333, 6.493845292982613),
    (0.0006463068554380658, 0.5340667761575997),
    (0.0, 8.342993098117972),
)
Co = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
Do = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0))


class Observer:
    """Track velocity, displacement, and disturbance for one wheel."""

    def __init__(self, encoder, ticks_per_wheel_rev):
        """Use the same encoder and scale as the motor velocity controller."""

        if not isinstance(ticks_per_wheel_rev, int) or ticks_per_wheel_rev <= 0:
            raise ValueError("Encoder ticks per wheel revolution must be a positive integer")

        self.encoder = encoder
        self.ticks_per_radian = ticks_per_wheel_rev / (2.0 * pi)

        # Allocate once per wheel. Only the fixed coefficients are shared.
        self.state = array("f", [0.0, 0.0, 0.0])
        self.next_state = array("f", [0.0, 0.0, 0.0])
        self.inputs = array("f", [0.0, 0.0])

    def get_displacement_rad(self):
        """Return signed wheel displacement from the encoder's zero."""

        # The motor task updates the encoder once before reading this value.
        # The driver already handles 16-bit rollover and applies polarity.
        # Keep the cumulative count; wrapping by one revolution loses distance.
        position_ticks = self.encoder.get_position()
        return position_ticks / self.ticks_per_radian

    def reset(self):
        """Initialize from the current encoder reference at rest."""

        self.state[0] = 0.0
        self.state[1] = self.get_displacement_rad()
        self.state[2] = 0.0
        self.inputs[0] = 0.0
        self.inputs[1] = self.state[1]
        for index in range(3):
            self.next_state[index] = self.state[index]

    def get_velocity_rad_s(self):
        """Read the estimate for the current sample, before advancing."""

        # Co is identity and Do is zero for the selected observer.
        return self.state[0]

    def update(self, applied_voltage_V, displacement_rad):
        """Advance the observer by one 10 ms motor sample."""

        self.inputs[0] = applied_voltage_V
        self.inputs[1] = displacement_rad

        # Compute all rows from the old state. Scalar dot products avoid
        # allocating temporary matrices in the scheduled motor task.
        for index in range(3):
            state_row = Ao[index]
            input_row = Bo[index]
            self.next_state[index] = (
                state_row[0] * self.state[0]
                + state_row[1] * self.state[1]
                + state_row[2] * self.state[2]
                + input_row[0] * self.inputs[0]
                + input_row[1] * self.inputs[1]
            )

        for index in range(3):
            self.state[index] = self.next_state[index]


def week1_numeric_check(Ao, Bo, Co, Do):
    """Return the current output and next state for the lab's fixed check."""

    # Use the selected MATLAB matrices. This check does not access hardware
    # or run as part of the motor control loop.
    from ulab import numpy as np

    # State order: velocity (rad/s), displacement (rad), disturbance (rad/s^2).
    state = np.array([[1.0], [0.2], [-0.5]])
    # Input order: limited voltage (V), measured displacement (rad).
    inputs = np.array([[2.0], [0.21]])

    current_output = np.dot(Co, state) + np.dot(Do, inputs)
    next_state = np.dot(Ao, state) + np.dot(Bo, inputs)

    return current_output, next_state


def verify_design(ticks_per_wheel_rev):
    """Check the selected matrices and runtime arithmetic before enabling trials."""

    from ulab import numpy as np

    def matrix(rows):
        return np.array([list(row) for row in rows])

    # References for the 10 ms release, not the earlier 20 ms board check.
    expected_output = (1.0, 0.2, -0.5)
    expected_next = (1.2721924517745482, 0.21638478801573713,
                     -0.4165700690188203)
    tolerance = 0.00001

    output, next_state = week1_numeric_check(
        matrix(Ao), matrix(Bo), matrix(Co), matrix(Do)
    )

    # Reproduce the same nonzero check using the scheduled update method.
    check = Observer(None, ticks_per_wheel_rev)
    for index in range(3):
        check.state[index] = expected_output[index]
    current_velocity = check.get_velocity_rad_s()
    check.update(2.0, 0.21)

    for index in range(3):
        if not abs(output[index, 0] - expected_output[index]) <= tolerance:
            raise ValueError("Observer current output failed the 10 ms check")
        if (not abs(next_state[index, 0] - expected_next[index]) <= tolerance or
                not abs(check.state[index] - expected_next[index]) <= tolerance):
            raise ValueError("Observer next state failed the 10 ms check")
    if not abs(current_velocity - expected_output[0]) <= tolerance:
        raise ValueError("Observer velocity output failed the 10 ms check")

    print("# LAB09 NUMERIC PASS Ts_ms=10")
    print("# LAB09 NUMERIC current", [output[i, 0] for i in range(3)])
    print("# LAB09 NUMERIC next", [next_state[i, 0] for i in range(3)])
