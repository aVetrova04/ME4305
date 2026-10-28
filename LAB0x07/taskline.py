from array import array
from time import ticks_ms, ticks_diff


class TaskLine:
    """Outer P controller and optional finite, buffered line data capture."""

    MAX_SAMPLES = 1500        # 20 ms task period gives about 30 seconds
    NO_LINE = 1000000.0      # sent as a blank field in the CSV output

    def __init__(self, sensor, left_data, right_data, config):
        self.sensor = sensor
        self.left_data = left_data
        self.right_data = right_data
        self.config = config

        self.running = False
        self.base_speed = 0.0
        self.kp_line = 0.0
        self.max_correction = 0.0
        self.centroid_mm = None
        self.elapsed_ms = 0
        self.start_ms = 0
        self.error = None

        # Only a requested data capture needs these large buffers.
        self.time_ms = None
        self.centroid_log = None
        self.capture_count = 0
        self.capture_active = False
        self.capture_ready = False
        self.capture_reason = ""

    def start(self, base_speed, kp_line, max_correction, capture=False):
        if capture:
            # Allocate before starting the wheels, not in the timed task loop.
            self.time_ms = array('I', [0] * self.MAX_SAMPLES)
            self.centroid_log = array('f', [0] * self.MAX_SAMPLES)
        else:
            self.time_ms = None
            self.centroid_log = None
        self.base_speed = abs(float(base_speed))
        self.kp_line = float(kp_line)
        self.max_correction = abs(float(max_correction))
        self.error = None
        self.centroid_mm = None
        self.elapsed_ms = 0
        self.capture_count = 0
        self.capture_active = capture
        self.capture_ready = False
        self.capture_reason = ""
        self.start_ms = ticks_ms()
        self.running = True

    def stop(self, reason="operator"):
        self.running = False
        self.left_data.velocity_setpoint = 0.0
        self.right_data.velocity_setpoint = 0.0
        if self.capture_active:
            self.capture_active = False
            self.capture_reason = reason
            self.capture_ready = True

    def taskline_gen_fcn(self):
        while True:
            if self.running:
                try:
                    centroid = self.sensor.update()
                    self.elapsed_ms = ticks_diff(ticks_ms(), self.start_ms)
                    self.centroid_mm = centroid

                    if self.capture_active:
                        index = self.capture_count
                        self.time_ms[index] = self.elapsed_ms
                        if centroid is None:
                            self.centroid_log[index] = self.NO_LINE
                        else:
                            self.centroid_log[index] = centroid
                        self.capture_count = index + 1

                    if centroid is None:
                        self.error = "Line centroid undefined"
                        self.stop("line_lost")
                    else:
                        correction = -self.kp_line * centroid
                        correction = max(-self.max_correction,
                                         min(self.max_correction, correction))

                        # On this robot, negative wheel speed moves the
                        # chassis toward the line sensor.
                        left_request = -self.base_speed + correction
                        right_request = -self.base_speed - correction
                        self.left_data.velocity_setpoint = min(0.0, left_request)
                        self.right_data.velocity_setpoint = min(0.0, right_request)

                        if self.capture_active and self.capture_count >= self.MAX_SAMPLES:
                            self.stop("complete")

                except Exception as exc:
                    self.error = str(exc)
                    self.stop("error")

            yield self.running
