from time import ticks_us, ticks_diff, ticks_add
from array import array
import math
from ClosedLoop import ClosedLoop

TICKS_PER_WHEEL_REVOLUTION = 1440
MOTOR_PERIOD_MS = 10

class MotorData:
    """Shared request, status, and command data for one motor."""

    def __init__(self):
        self.request = False
        self.busy = False
        self.ready = False

        # Manual PWM command, in percent.
        self.cycle = 0

        # Shared closed-loop settings.
        self.kp = 0.0
        self.v_min = -3.0
        self.v_max = 3.0

        # Storage for one 100-sample trial.
        self.time_us = array('L', (0 for _ in range(100)))
        self.position = array('l', (0 for _ in range(100)))
        self.velocity = array('f', (0 for _ in range(100)))

        # Requested wheel velocity, in rad/s.
        self.velocity_setpoint = 0.0
        # Current encoder-derived wheel velocity, in rad/s.
        self.measured_speed_rad_s = 0.0
        self.estimated_speed_rad_s = 0.0
        self.limited_voltage_V = 0.0
        self.capture_callback = None
        self.run_deadline_us = None

        self.count = 0
        self.effort = 0

    def clear(self):
        self.count = 0

    def add(self, time_us, position, velocity):
        index = self.count

        self.time_us[index] = time_us
        self.position[index] = position
        self.velocity[index] = velocity

        self.count = index + 1


class TaskMotor:

    S_INIT = 0
    S_WAIT = 1
    S_START_TRIAL = 2
    S_SAMPLE = 3
    S_WAIT_PRINT = 4
    S_PAUSE = 5
    S_FINISH = 6

    def __init__(self, motor, encoder, shared, controller, observer):
        self.motor = motor
        self.encoder = encoder
        self.shared = shared
        self.controller = controller if controller is not None else ClosedLoop()
        self.observer = observer
        self.controller.set_kp(self.shared.kp)
        self.controller.set_lim(self.shared.v_min, self.shared.v_max)

        # Step-response commands, in percent.
        self.efforts = []

        # Sample every 10 ms for 100 samples = approximately 1 second.
        self.sample_interval_us = 10_000
        self.samples_per_trial = 100

        self.state = self.S_INIT

        self.current_effort = 0
        self.sample_count = 0

        self.trial_start_us = 0
        self.next_sample_us = 0
        self.next_trial_us = 0
        self.closed_loop_enabled = False
        self.timing_active = False
        self.previous_start_us = None
        self.timing_samples = 0
        self.total_gap_us = 0
        self.max_gap_us = 0
        self.max_exec_us = 0

    def reset_run(self):
        """Zero the measured and estimated displacement at the same point."""

        self.encoder.zero()
        self.observer.reset()
        self.controller.reset()
        self.shared.measured_speed_rad_s = 0.0
        self.shared.estimated_speed_rad_s = 0.0

    def start_timing(self):
        """Start fresh timing measurements for this trial."""

        self.previous_start_us = None
        self.timing_samples = 0
        self.total_gap_us = 0
        self.max_gap_us = 0
        self.max_exec_us = 0
        self.timing_active = True

    def record_timing(self, start_us):
        """Measure sample spacing and work done in the motor task."""

        execution_us = ticks_diff(ticks_us(), start_us)
        self.max_exec_us = max(self.max_exec_us, execution_us)
        if self.previous_start_us is not None:
            gap_us = ticks_diff(start_us, self.previous_start_us)
            self.total_gap_us += gap_us
            self.max_gap_us = max(self.max_gap_us, gap_us)
        self.previous_start_us = start_us
        self.timing_samples += 1

    def timing_summary(self):
        """Report timing outside of the repeated control calculation."""

        mean_gap_us = 0
        if self.timing_samples > 1:
            mean_gap_us = self.total_gap_us / (self.timing_samples - 1)
        return {
            "samples": self.timing_samples,
            "mean_gap_us": mean_gap_us,
            "max_gap_us": self.max_gap_us,
            "max_exec_us": self.max_exec_us,
        }

    def velocity_control(self):
        """Use the original feedback and monitor the observer at 10 ms."""

        now_us = ticks_us()
        deadline = self.shared.run_deadline_us
        if deadline is not None and ticks_diff(now_us, deadline) >= 0:
            # Stop even if the user task cannot service the USB link.
            self.stop()
            return

        # Update the encoder once. Read the estimate belonging to this tick.
        self.encoder.update()
        self.shared.measured_speed_rad_s = (
            self.encoder.get_velocity() * 2 * math.pi / TICKS_PER_WHEEL_REVOLUTION
        )
        displacement_rad = self.observer.get_displacement_rad()
        self.shared.estimated_speed_rad_s = self.observer.get_velocity_rad_s()

        # Step 3 keeps the existing P controller on the original signal.
        requested = self.shared.velocity_setpoint
        if requested == 0:
            if self.closed_loop_enabled:
                self.motor.set_effort(0)
                self.motor.disable()
                self.closed_loop_enabled = False
            self.controller.reset()
            applied_voltage_V = 0.0
        else:
            self.controller.set_kp(self.shared.kp)
            self.controller.set_lim(self.shared.v_min, self.shared.v_max)
            self.controller.set_vel(requested)
            voltage_V = self.controller.update(self.shared.measured_speed_rad_s)
            if not self.closed_loop_enabled:
                self.motor.enable()
                self.closed_loop_enabled = True
            # set_voltage returns the signed voltage represented by the duty
            # sent to the driver, after clipping and integer PWM rounding.
            applied_voltage_V = self.motor.set_voltage(voltage_V)
        self.shared.limited_voltage_V = applied_voltage_V

        # Log the current estimate before updating it for the next tick.
        if self.shared.capture_callback is not None:
            self.shared.capture_callback(now_us, self.shared)
        self.observer.update(applied_voltage_V, displacement_rad)

        if self.timing_active:
            self.record_timing(now_us)

    def taskmotor_gen_fcn(self):
        """Generator implementing the nonblocking motor state machine."""

        while True:

            # ---------------------------------------------------------
            # Initialize motor and shared data.
            # ---------------------------------------------------------
            if self.state == self.S_INIT:
                self.motor.set_effort(0)
                self.motor.disable()
                self.closed_loop_enabled = False
                self.reset_run()

                self.shared.request = False
                self.shared.busy = False
                self.shared.ready = False
                self.shared.clear()

                self.state = self.S_WAIT

            # ---------------------------------------------------------
            # Wait for TaskUser to request a test.
            # ---------------------------------------------------------
            elif self.state == self.S_WAIT:
                if self.shared.request:
                    self.motor.set_effort(0)
                    self.motor.disable()
                    self.closed_loop_enabled = False
                    self.shared.request = False
                    self.shared.busy = True
                    self.shared.ready = False
                    self.shared.clear()

                    # Reset the trial sequence.
                    self.efforts = [self.shared.cycle]

                    self.state = self.S_START_TRIAL

                else:
                    self.velocity_control()

            # ---------------------------------------------------------
            # Start the next step-response trial.
            # ---------------------------------------------------------
            elif self.state == self.S_START_TRIAL:

                if len(self.efforts) == 0:
                    self.state = self.S_FINISH

                else:
                    self.current_effort = self.efforts.pop(0)
                    self.sample_count = 0

                    self.shared.clear()
                    self.shared.effort = self.current_effort

                    # Reset encoder so each trial starts at zero.
                    self.encoder.zero()

                    # Apply the step input.
                    self.motor.enable()
                    self.motor.set_effort(self.current_effort)

                    self.trial_start_us = ticks_us()
                    self.next_sample_us = self.trial_start_us

                    self.state = self.S_SAMPLE

            # ---------------------------------------------------------
            # Collect samples without blocking the scheduler.
            # ---------------------------------------------------------
            elif self.state == self.S_SAMPLE:
                now_us = ticks_us()

                if ticks_diff(now_us, self.next_sample_us) >= 0:
                    # This older l/r step test applies the requested PWM
                    # directly; the encoder is measured, not controlled.
                    self.encoder.update()

                    elapsed_us = ticks_diff(
                        now_us,
                        self.trial_start_us
                    )

                    self.shared.add(
                        elapsed_us,
                        self.encoder.get_position(),
                        self.encoder.get_velocity()
                    )

                    self.sample_count += 1

                    self.next_sample_us = ticks_add(
                        self.next_sample_us,
                        self.sample_interval_us
                    )

                    # Trial is complete.
                    if self.sample_count >= self.samples_per_trial:
                        self.motor.set_effort(0)
                        self.motor.disable()

                        # Tell TaskUser that the sample buffer is ready.
                        self.shared.ready = True

                        self.state = self.S_WAIT_PRINT

            # ---------------------------------------------------------
            # Wait for TaskUser to finish printing the current trial.
            # ---------------------------------------------------------
            elif self.state == self.S_WAIT_PRINT:
                if not self.shared.ready:

                    # Give the motor a one-second pause before
                    # beginning the next step-response trial.
                    self.next_trial_us = ticks_add(
                        ticks_us(),
                        1_000_000
                    )

                    self.state = self.S_PAUSE

            # ---------------------------------------------------------
            # Pause between trials.
            # ---------------------------------------------------------
            elif self.state == self.S_PAUSE:
                if ticks_diff(ticks_us(), self.next_trial_us) >= 0:
                    self.state = self.S_START_TRIAL

            # ---------------------------------------------------------
            # All trials are complete.
            #
            # Do NOT set ready here. TaskUser has already printed the
            # final trial. Setting ready=True here would cause it to
            # interpret the empty buffer as another trial.
            # ---------------------------------------------------------
            elif self.state == self.S_FINISH:
                self.motor.set_effort(0)
                self.motor.disable()
                self.closed_loop_enabled = False

                self.shared.busy = False
                self.shared.ready = False
                self.shared.request = False
                self.shared.clear()

                # Open-loop diagnostics use a separate encoder reference.
                self.reset_run()
                self.state = self.S_WAIT

            else:
                raise ValueError("Invalid TaskMotor state")

            # Always yield so cotask can run other tasks.
            yield self.state

    def stop(self):
        """Safely disable this motor during shutdown."""

        self.motor.set_effort(0)
        self.motor.disable()
        self.closed_loop_enabled = False
        self.timing_active = False
        self.shared.velocity_setpoint = 0.0
        self.shared.limited_voltage_V = 0.0
        self.shared.capture_callback = None
        self.shared.run_deadline_us = None

        self.shared.request = False
        self.shared.busy = False
        self.shared.ready = False
        self.shared.clear()

        self.state = self.S_WAIT
