from time import ticks_us, ticks_diff, ticks_add
from array import array


class MotorData:
    """request, status, and samples shared with the user task."""

    def __init__(self):
        self.request = False
        self.busy = False
        self.ready = False
        # one trial of 100 samples fits in compact fixed-size buffers
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
    """run five nonblocking step-response trials on one motor."""

    S_INIT = 0
    S_WAIT = 1
    S_START_TRIAL = 2
    S_SAMPLE = 3
    S_FINISH = 4
    S_PAUSE = 5
    S_WAIT_PRINT = 6

    def __init__(self, motor, encoder, shared):
        self.motor = motor
        self.encoder = encoder
        self.shared = shared

        self.eff_list = [20, 40, 60, 80, 100]
        self.sample_interval_us = 10_000
        self.samples_per_trial = 100

        self.state = self.S_INIT
        self.current_effort = 0
        self.sample_count = 0
        self.trial_start_us = 0
        self.next_sample_us = 0
        self.next_trial_us = 0

    def taskmotor_gen_fcn(self):
        while True:
            if self.state == self.S_INIT:
                self.motor.set_effort(0)
                self.motor.disable()
                self.encoder.zero()

                self.shared.busy = False
                self.shared.ready = False
                self.shared.request = False
                self.shared.clear()
                self.state = self.S_WAIT

            elif self.state == self.S_WAIT:
                if self.shared.request:
                    self.shared.request = False
                    self.shared.busy = True
                    self.shared.ready = False
                    self.shared.clear()
                    self.eff_list = [20, 40, 60, 80, 100]
                    self.state = self.S_START_TRIAL

            elif self.state == self.S_START_TRIAL:
                if len(self.eff_list) == 0:
                    self.state = self.S_FINISH
                else:
                    self.current_effort = self.eff_list.pop(0)
                    self.sample_count = 0
                    self.shared.clear()
                    self.shared.effort = self.current_effort
                    self.encoder.zero()

                    self.motor.enable()
                    self.motor.set_effort(self.current_effort)

                    self.trial_start_us = ticks_us()
                    self.next_sample_us = self.trial_start_us
                    self.state = self.S_SAMPLE

            elif self.state == self.S_SAMPLE:
                now_us = ticks_us()
                if ticks_diff(now_us, self.next_sample_us) >= 0:
                    self.encoder.update()
                    elapsed_us = ticks_diff(now_us, self.trial_start_us)

                    self.shared.add(elapsed_us,
                                    self.encoder.get_position(),
                                    self.encoder.get_velocity())

                    self.sample_count += 1
                    self.next_sample_us = ticks_add(
                        self.next_sample_us, self.sample_interval_us
                    )

                    if self.sample_count >= self.samples_per_trial:
                        self.motor.set_effort(0)
                        self.motor.disable()
                        self.shared.ready = True
                        self.state = self.S_WAIT_PRINT

            elif self.state == self.S_WAIT_PRINT:
                # wait for TaskUser to finish printing this trial
                if not self.shared.ready:
                    self.next_trial_us = ticks_add(ticks_us(), 1_000_000)
                    self.state = self.S_PAUSE

            elif self.state == self.S_PAUSE:
                if ticks_diff(ticks_us(), self.next_trial_us) >= 0:
                    self.state = self.S_START_TRIAL

            elif self.state == self.S_FINISH:
                self.motor.set_effort(0)
                self.motor.disable()
                self.shared.busy = False
                self.shared.ready = True
                self.state = self.S_WAIT

            else:
                raise ValueError("Invalid TaskMotor state")

            # return control to cotask on every pass
            yield self.state

    def stop(self):
        """disable this motor during shutdown."""
        self.motor.set_effort(0)
        self.motor.disable()
        self.shared.request = False
        self.shared.busy = False
        self.shared.ready = False
        self.shared.clear()
        self.state = self.S_WAIT
