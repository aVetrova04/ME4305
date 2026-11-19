from math import pi, sin
from time import ticks_ms, ticks_diff


class TaskIMU:
    """One automatic lane change; TaskLine steers on visible lines."""

    IDLE = 0
    FOLLOW_START = 1
    DEPART = 2
    TURN_BACK = 3
    FOLLOW_TARGET = 4
    DONE = 5
    STATE_NAMES = ("idle", "follow_start", "depart", "turn_back",
                   "follow_target", "done")

    # Starting values for tuning on the printed track.
    WHEEL_RADIUS_M = 0.035
    LANE_DIRECTION = 1               # +1 is left, -1 is right
    DEPART_AFTER_MS = 800
    TURN_BACK_AT_M = 0.100           # half the 0.200 m line spacing
    MANEUVER_TIMEOUT_MS = 8500
    TARGET_TIMEOUT_MS = 15000
    YAW_RATE_RAD_S = 0.5
    YAW_KP = 2.0
    MAX_YAW_CORRECTION = 1.5
    WHITE_MAX_TOTAL_WEIGHT = 0.20
    WHITE_CONFIRM_SAMPLES = 3
    TARGET_MIN_TOTAL_WEIGHT = 0.50
    TARGET_REACQUIRE_MIN_M = 0.16   # target line is about 0.20 m away
    TARGET_ALIGN_MAX_RAD = 0.35     # face close to the first line's heading
    TARGET_CONFIRM_SAMPLES = 3
    BAR_BLACK_WEIGHT = 0.50
    BAR_MIN_BLACK_SENSORS = 6
    BAR_CONFIRM_SAMPLES = 1

    def __init__(self, imu, line_task, left_data, right_data):
        self.imu = imu
        self.line_task = line_task
        self.left_data = left_data
        self.right_data = right_data
        self.state = self.IDLE
        self.running = False
        self.stop_reason = None
        self.error = None
        self.base_speed = 0.0
        self.kp_line = 0.0
        self.max_line_correction = 0.0
        self.start_ms = 0
        self.turn_start_ms = 0
        self.target_start_ms = 0
        self.previous_ms = 0
        self.heading_start = 0.0
        self.relative_heading_rad = 0.0
        self.y_estimate_m = 0.0
        self.white_count = 0
        self.target_count = 0
        self.bar_count = 0
        self.cleared_start_line = False

    def start(self, base_speed, kp_line, max_correction):
        if self.running or self.line_task.running:
            raise RuntimeError("A steering task is already running")
        self.base_speed = abs(float(base_speed))
        self.kp_line = kp_line
        self.max_line_correction = max_correction
        self.start_ms = ticks_ms()
        self.stop_reason = None
        self.error = None
        self.target_start_ms = 0
        self.y_estimate_m = 0.0
        self.relative_heading_rad = 0.0
        self.white_count = 0
        self.target_count = 0
        self.bar_count = 0
        self.cleared_start_line = False
        self.line_task.start(base_speed, kp_line, max_correction)
        self.state = self.FOLLOW_START
        self.running = True

    def stop(self, reason="operator"):
        if self.line_task.running:
            self.line_task.stop(reason)
        self.left_data.velocity_setpoint = 0.0
        self.right_data.velocity_setpoint = 0.0
        self.running = False
        self.state = self.DONE
        self.stop_reason = reason

    def _request_yaw_rate(self, desired_rate):
        # This is yaw-rate feedback, not an open-loop wheel-speed offset.
        error = desired_rate - self.imu.yaw_rate_rad_s()
        correction = self.YAW_KP * error
        correction = max(-self.MAX_YAW_CORRECTION,
                         min(self.MAX_YAW_CORRECTION, correction))

        # Lab 6's negative wheel requests move this robot forward.
        self.left_data.velocity_setpoint = min(0.0,
                                               -self.base_speed + correction)
        self.right_data.velocity_setpoint = min(0.0,
                                                -self.base_speed - correction)

    def _bar_seen(self):
        weights = self.line_task.sensor.weights
        return (len(weights) == 8 and
                sum(w >= self.BAR_BLACK_WEIGHT for w in weights)
                >= self.BAR_MIN_BLACK_SENSORS)

    def taskimu_gen_fcn(self):
        while True:
            if self.running:
                try:
                    now = ticks_ms()

                    if self.state == self.FOLLOW_START:
                        # TaskLine owns the wheel requests while on the line.
                        if not self.line_task.running:
                            self.stop("start_line_lost")
                        elif ticks_diff(now, self.start_ms) >= self.DEPART_AFTER_MS:
                            self.line_task.stop("handoff")
                            self.heading_start = self.imu.heading_rad()
                            self.y_estimate_m = 0.0
                            self.previous_ms = now
                            self.turn_start_ms = now
                            self.white_count = 0
                            self.target_count = 0
                            self.cleared_start_line = False
                            self.state = self.DEPART
                            self._request_yaw_rate(
                                self.LANE_DIRECTION * self.YAW_RATE_RAD_S
                            )

                    elif self.state == self.DEPART or self.state == self.TURN_BACK:
                        if ticks_diff(now, self.turn_start_ms) > self.MANEUVER_TIMEOUT_MS:
                            self.stop("target_timeout")
                        else:
                            # TaskLine is paused: this task owns the sensor and steering.
                            centroid = self.line_task.sensor.update()
                            weight = self.line_task.sensor.total_weight
                            if weight <= self.WHITE_MAX_TOTAL_WEIGHT:
                                self.white_count += 1
                            else:
                                self.white_count = 0
                            if self.white_count >= self.WHITE_CONFIRM_SAMPLES:
                                self.cleared_start_line = True

                            dt_ms = ticks_diff(now, self.previous_ms)
                            self.previous_ms = now
                            if dt_ms < 0 or dt_ms > 200:
                                self.stop("scheduler_delay")
                            else:
                                heading = self.imu.heading_rad()
                                self.relative_heading_rad = (
                                    (heading - self.heading_start + pi) % (2 * pi)
                                    - pi
                                )

                                # Encoder speeds are negative during physical forward travel.
                                forward_m_s = -self.WHEEL_RADIUS_M * (
                                    self.left_data.measured_speed_rad_s
                                    + self.right_data.measured_speed_rad_s
                                ) / 2
                                self.y_estimate_m += (
                                    forward_m_s * sin(self.relative_heading_rad)
                                    * (dt_ms / 1000.0)
                                )

                                if (self.state == self.DEPART and
                                        self.LANE_DIRECTION * self.y_estimate_m
                                        >= self.TURN_BACK_AT_M):
                                    self.state = self.TURN_BACK

                                if (self.state == self.TURN_BACK and
                                        self.cleared_start_line and
                                        self.LANE_DIRECTION * self.y_estimate_m
                                        >= self.TARGET_REACQUIRE_MIN_M and
                                        abs(self.relative_heading_rad)
                                        <= self.TARGET_ALIGN_MAX_RAD and
                                        centroid is not None and
                                        weight >= self.TARGET_MIN_TOTAL_WEIGHT):
                                    self.target_count += 1
                                else:
                                    self.target_count = 0

                                if (self.state == self.TURN_BACK and
                                        self.target_count >= self.TARGET_CONFIRM_SAMPLES):
                                    self.left_data.velocity_setpoint = 0.0
                                    self.right_data.velocity_setpoint = 0.0
                                    self.line_task.start(
                                        self.base_speed, self.kp_line,
                                        self.max_line_correction
                                    )
                                    self.target_start_ms = now
                                    self.state = self.FOLLOW_TARGET
                                elif self.state == self.DEPART:
                                    self._request_yaw_rate(
                                        self.LANE_DIRECTION * self.YAW_RATE_RAD_S
                                    )
                                elif self.state == self.TURN_BACK:
                                    self._request_yaw_rate(
                                        -self.LANE_DIRECTION * self.YAW_RATE_RAD_S
                                    )

                                # Catch a stop bar even if we have not handed
                                # control back to the line-following task yet.
                                # Requiring the white gap and lateral travel
                                # prevents the starting line from stopping us.
                                if (self.running and
                                        self.state in (self.TURN_BACK,
                                                       self.FOLLOW_TARGET) and
                                        self.cleared_start_line and
                                        self.LANE_DIRECTION * self.y_estimate_m
                                        >= self.TARGET_REACQUIRE_MIN_M and
                                        self._bar_seen()):
                                    self.stop("stop_bar")

                    elif self.state == self.FOLLOW_TARGET:
                        # A normal narrow line reaches only a few sensors.
                        # Allow two imperfect channels on a wide stop bar;
                        # stop on the first match so a short bar is not missed.
                        if self._bar_seen():
                            self.bar_count += 1
                        else:
                            self.bar_count = 0
                        if self.bar_count >= self.BAR_CONFIRM_SAMPLES:
                            self.stop("stop_bar")
                        elif not self.line_task.running:
                            self.stop("target_line_lost")
                        elif ticks_diff(now, self.target_start_ms) > self.TARGET_TIMEOUT_MS:
                            self.stop("stop_bar_timeout")

                except Exception as exc:
                    self.error = str(exc)
                    self.stop("error")

            yield self.state
