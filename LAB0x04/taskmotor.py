from time import ticks_us, ticks_diff, ticks_add
from array import array


class MotorData:
    """Shared request, status, and command data for one motor."""

    def __init__(self):
        self.request = False
        self.busy = False
        self.ready = False

        # Requested manual duty cycle.
        self.cycle = 0

        # Storage for one 100-sample trial.
        self.time_us = array('L', (0 for _ in range(100)))
        self.position = array('l', (0 for _ in range(100)))
        self.velocity = array('f', (0 for _ in range(100)))

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

    def __init__(self, motor, encoder, shared):
        self.motor = motor
        self.encoder = encoder
        self.shared = shared

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

    def taskmotor_gen_fcn(self):
        """Generator implementing the nonblocking motor state machine."""

        while True:

            # ---------------------------------------------------------
            # Initialize motor and shared data.
            # ---------------------------------------------------------
            if self.state == self.S_INIT:
                self.motor.set_effort(0)
                self.motor.disable()
                self.encoder.zero()

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
                    self.shared.request = False
                    self.shared.busy = True
                    self.shared.ready = False
                    self.shared.clear()

                    # Reset the trial sequence.
                    self.efforts = [self.shared.cycle]

                    self.state = self.S_START_TRIAL

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

                self.shared.busy = False
                self.shared.ready = False
                self.shared.request = False
                self.shared.clear()

                self.state = self.S_WAIT

            else:
                raise ValueError("Invalid TaskMotor state")

            # Always yield so cotask can run other tasks.
            yield self.state

    def stop(self):
        """Safely disable this motor during shutdown."""

        self.motor.set_effort(0)
        self.motor.disable()

        self.shared.request = False
        self.shared.busy = False
        self.shared.ready = False
        self.shared.clear()

        self.state = self.S_WAIT
