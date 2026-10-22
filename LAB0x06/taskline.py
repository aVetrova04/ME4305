from time import ticks_ms, ticks_diff


class TaskLine:
    """Outer P controller: line position to wheel-speed requests."""

    def __init__(self, sensor, left_data, right_data, config):
        self.sensor = sensor
        self.left_data = left_data
        self.right_data = right_data
        self.config = config

        self.running = False
        self.base_speed = 0.0       # rad/s
        self.kp_line = 0.0          # (rad/s)/mm
        self.max_correction = 0.0   # rad/s

        self.centroid_mm = None
        self.elapsed_ms = 0
        self.start_ms = 0
        self.log_callback = None
        self.error = None

    def start(self, base_speed, kp_line, max_correction, log_callback=None):
        self.base_speed = abs(float(base_speed))
        self.kp_line = float(kp_line)
        self.max_correction = abs(float(max_correction))
        self.log_callback = log_callback
        self.error = None
        self.centroid_mm = None
        self.start_ms = ticks_ms()
        self.running = True

    def stop(self):
        self.running = False
        self.left_data.velocity_setpoint = 0.0
        self.right_data.velocity_setpoint = 0.0

    def taskline_gen_fcn(self):
        while True:
            if self.running:
                try:
                    centroid = self.sensor.update()
                    self.elapsed_ms = ticks_diff(ticks_ms(), self.start_ms)
                    self.centroid_mm = centroid

                    if centroid is None:
                        # Never use an undefined centroid.
                        self.stop()
                        self.error = "Line centroid undefined"
                    else:
                        correction = self.kp_line * centroid
                        correction = max(
                            -self.max_correction,
                            min(self.max_correction, correction)
                        )

                        # Positive correction: turn left.
                        left_request = -self.base_speed + correction
                        right_request = -self.base_speed - correction

                        # Avoid reversing either wheel during line following.
                        left_request = min(0.0, left_request)
                        right_request = min(0.0, right_request)

                        self.left_data.velocity_setpoint = left_request
                        self.right_data.velocity_setpoint = right_request

                        if callable(self.log_callback):
                            self.log_callback(
                                self.elapsed_ms,
                                centroid,
                                self.base_speed,
                                self.kp_line,
                                correction
                            )

                except Exception as exc:
                    self.error = str(exc)
                    self.stop()

            yield self.running
