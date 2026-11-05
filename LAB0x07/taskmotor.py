from time import ticks_us, ticks_diff, ticks_add
from array import array
from math import pi

from ClosedLoop import ClosedLoop

TICKS_PER_WHEEL_REVOLUTION = 1440
TICKS_PER_RADIAN = TICKS_PER_WHEEL_REVOLUTION / (2.0 * pi)
NOMINAL_BATTERY_VOLTAGE = 7.2
SAMPLE_INTERVAL_US = 10_000
SAMPLES_PER_TRIAL = 100
TRIAL_PAUSE_US = 1_000_000

class MotorData:
    """Shared communication data for one motor."""

    def __init__(self):
        self.request = False
        self.busy = False
        self.ready = False

        # Open-loop test PWM command (%).
        self.cycle = 0

        # Closed-loop controller configuration.
        self.kp = 0.0
        self.v_min = -3.0
        self.v_max = 3.0

        # Velocity request and feedback (rad/s).
        self.velocity_setpoint = 0.0
        self.velocity_measured = 0.0

        # Encoder position (ticks).
        self.position_measured = 0

        # Applied motor effort (%).
        self.effort = 0.0
        self.enabled = False

        # Step-response storage.
        self.max_samples = SAMPLES_PER_TRIAL
        self.time_us = array('L', [0] * self.max_samples)
        self.position = array('l', [0] * self.max_samples)
        self.velocity = array('f', [0] * self.max_samples)

        self.count = 0

    def clear(self):
        """Reset the sample buffer."""
        self.count = 0

    def add(self, time_us, position, velocity):
        """Add one sample to the fixed-size buffer."""

        if self.count >= self.max_samples:
            return False

        index = self.count

        self.time_us[index] = time_us
        self.position[index] = position
        self.velocity[index] = velocity

        self.count += 1

        return True

    def request_test(self, effort):
        """Request an open-loop PWM test."""

        if self.busy or self.ready or self.request:
            return False

        effort = float(effort)

        if not -100.0 <= effort <= 100.0:
            raise ValueError("PWM must be between -100 and 100")

        self.cycle = effort
        self.request = True

        return True

    def reset_test(self):
        """Reset test communication flags."""

        self.request = False
        self.busy = False
        self.ready = False
        self.clear()


class TaskMotor:

    S_INIT = 0
    S_WAIT = 1
    S_START_TRIAL = 2
    S_SAMPLE = 3
    S_WAIT_PRINT = 4
    S_PAUSE = 5
    S_FINISH = 6

    MODE_VELOCITY = 0
    MODE_TEST = 1
    MODE_STOPPED = 2

    def __init__(
        self,
        motor,
        encoder,
        shared,
        controller=None
    ):

        self.motor = motor
        self.encoder = encoder
        self.shared = shared

        # Use supplied controller or create one.
        self.controller = (
            controller if controller is not None
            else ClosedLoop()
        )

        self.controller.set_kp(self.shared.kp)
        self.controller.set_lim(
            self.shared.v_min,
            self.shared.v_max
        )

        self.mode = self.MODE_STOPPED
        self.state = self.S_INIT

        self.motor_enabled = False

        self.sample_interval_us = SAMPLE_INTERVAL_US
        self.samples_per_trial = SAMPLES_PER_TRIAL

        self.current_effort = 0.0
        self.sample_count = 0

        self.trial_start_us = 0
        self.next_sample_us = 0
        self.next_trial_us = 0

    def enable_motor(self):
        """Enable motor hardware."""

        if not self.motor_enabled:
            self.motor.enable()
            self.motor_enabled = True

        self.shared.enabled = True

    def disable_motor(self):
        """Set zero effort and disable motor hardware."""

        self.motor.set_effort(0)
        self.motor.disable()

        self.motor_enabled = False

        self.shared.enabled = False
        self.shared.effort = 0.0

    def update_feedback(self):
        """Update encoder measurements."""

        self.encoder.update()

        position = self.encoder.get_position()
        velocity_ticks_s = self.encoder.get_velocity()

        velocity_rad_s = (
            velocity_ticks_s / TICKS_PER_RADIAN
        )

        self.shared.position_measured = position
        self.shared.velocity_measured = velocity_rad_s

        return velocity_ticks_s, velocity_rad_s

    def velocity_control(self):
        """Maintain requested wheel velocity using ClosedLoop."""

        _, measured_velocity = self.update_feedback()

        setpoint = self.shared.velocity_setpoint

        if abs(setpoint) < 0.001:
            self.disable_motor()
            self.controller.reset()
            return

        self.controller.set_vel(setpoint)
        self.controller.set_kp(self.shared.kp)

        self.controller.set_lim(
            self.shared.v_min,
            self.shared.v_max
        )

        voltage = self.controller.update(measured_velocity)

        effort = (
            voltage / NOMINAL_BATTERY_VOLTAGE
        ) * 100.0

        effort = max(-100.0, min(100.0, effort))

        self.enable_motor()
        self.motor.set_effort(effort)

        self.shared.effort = effort

    def start_trial(self):
        """Start an open-loop PWM step-response test."""

        self.sample_count = 0
        self.shared.clear()

        self.current_effort = self.shared.cycle
        self.shared.effort = self.current_effort

        self.controller.reset()

        self.encoder.zero()

        self.enable_motor()
        self.motor.set_effort(self.current_effort)

        self.trial_start_us = ticks_us()
        self.next_sample_us = self.trial_start_us

        self.state = self.S_SAMPLE

    def sample_trial(self):
        """Collect one sample when the sampling interval expires."""

        now = ticks_us()

        if ticks_diff(now, self.next_sample_us) < 0:
            return

        velocity_ticks_s, _ = self.update_feedback()

        elapsed_us = ticks_diff(
            now,
            self.trial_start_us
        )

        self.shared.add(
            elapsed_us,
            self.shared.position_measured,
            velocity_ticks_s
        )

        self.sample_count += 1

        self.next_sample_us = ticks_add(
            self.next_sample_us,
            self.sample_interval_us
        )

        if self.sample_count >= self.samples_per_trial:
            self.disable_motor()

            self.shared.ready = True

            self.state = self.S_WAIT_PRINT

    def taskmotor_gen_fcn(self):
        """Cooperative motor task."""

        while True:

            if self.state == self.S_INIT:

                self.disable_motor()
                self.encoder.zero()

                self.shared.velocity_setpoint = 0.0
                self.shared.velocity_measured = 0.0
                self.shared.position_measured = 0

                self.shared.reset_test()

                self.mode = self.MODE_VELOCITY
                self.state = self.S_WAIT

            elif self.state == self.S_WAIT:

                if self.shared.request:

                    self.disable_motor()
                    self.shared.velocity_setpoint = 0.0

                    self.shared.request = False
                    self.shared.busy = True
                    self.shared.ready = False

                    self.shared.clear()

                    self.mode = self.MODE_TEST
                    self.state = self.S_START_TRIAL

                else:
                    self.mode = self.MODE_VELOCITY
                    self.velocity_control()

            elif self.state == self.S_START_TRIAL:

                self.start_trial()

            elif self.state == self.S_SAMPLE:

                self.sample_trial()

            elif self.state == self.S_WAIT_PRINT:

                if not self.shared.ready:

                    self.next_trial_us = ticks_add(
                        ticks_us(),
                        TRIAL_PAUSE_US
                    )

                    self.state = self.S_PAUSE

            elif self.state == self.S_PAUSE:

                if ticks_diff(
                    ticks_us(),
                    self.next_trial_us
                ) >= 0:

                    self.state = self.S_FINISH

            elif self.state == self.S_FINISH:

                self.disable_motor()

                self.shared.velocity_setpoint = 0.0
                self.shared.reset_test()

                self.controller.reset()

                self.mode = self.MODE_VELOCITY
                self.state = self.S_WAIT

            else:
                raise ValueError("Invalid TaskMotor state")

            yield self.state

    def stop(self):
        """Safely stop motor and clear shared requests."""

        self.disable_motor()

        self.shared.velocity_setpoint = 0.0
        self.shared.reset_test()

        self.controller.reset()

        self.mode = self.MODE_STOPPED
        self.state = self.S_WAIT
