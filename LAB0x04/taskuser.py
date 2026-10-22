from MultiChar import MultiChar


class TaskUser:
    """Poll USB input and print test data without blocking motor tasks."""

    S_INIT = 0
    S_WAIT = 1
    S_WAIT_RESULT = 2
    S_PRINT = 3
    S_EXIT = 4
    S_GAIN = 5
    S_VEL = 6
    S_KP_VALUE = 7
    S_LINE_PRINT = 8

    def __init__(self, vcp, left_data, right_data, system, line_task):
        self.message = "Enter command (h, f, c, s, v, k, l, r, or e):"
        self.vcp = vcp
        self.left_data = left_data
        self.right_data = right_data
        self.system = system
        self.multi_char = MultiChar(vcp)

        self.state = self.S_INIT
        self.active_data = None
        self.active_name = None
        self.print_index = 0
        self.output_started = False

        self.vel = 0.0
        self.p_gain = 0.0
        self.i_gain = 0.0

        self.line_task = line_task
        self.kp_line = 0.0
        self.max_correction = 1.0
        self.line_print_index = 0
        self.last_line_error = None

        # Shared controller settings; voltage limits remain fixed.
        for data in (self.left_data, self.right_data):
            data.kp = self.p_gain
            data.v_min = -6.0
            data.v_max = 6.0

        self.gain_type = None

    def write_line(self, text):
        self.vcp.write((text + "\r\n").encode())

    def print_help(self):
        self.write_line("")
        self.write_line(
            "+--------------------------------------------------------------+"
        )
        self.write_line(
            "| ME 4305 Romi Tuning Interface Help Menu                      |"
        )
        self.write_line(
            "+-----+--------------------------------------------------------+"
        )
        self.write_line(
            "| h/H | Print help menu                                        |"
        )
        self.write_line(
            "+-----+--------------------------------------------------------+"
        )
        self.write_line(
            "| f/F | Commence line follow                                   |"
        )
        self.write_line("| c/C | Capture a 30-second line-following run                  |")
        self.write_line("| s/S | Stop line following                                     |")
        self.write_line(
            "+-----+--------------------------------------------------------+"
        )
        self.write_line(
            "| v/V | Enter a velocity (rad/s)                               |"
        )
        self.write_line(
            "| k/K | Set proportional gain                                  |"
        )
        self.write_line(
            "| l/L | Run step-response test on left motor                   |"
        )
        self.write_line(
            "| r/R | Run step-response test on right motor                  |"
        )
        self.write_line(
            "| e/E | Exit program                                           |"
        )
        self.write_line(
            "+-----+--------------------------------------------------------+"
        )
        self.write_line("")

    def taskuser_gen_fcn(self):
        while True:

            if self.state == self.S_INIT:
                self.print_help()
                self.write_line(self.message)
                self.state = self.S_WAIT

            elif self.state == self.S_WAIT:
                if self.line_task.capture_ready:
                    # The wheels stopped before any dataset is printed.
                    self.write_line("# LINE FOLLOWING DATA")
                    self.write_line("# channels=1,2,3,4,5,6,7,8")
                    self.write_line("# sensor_count=8")
                    self.write_line("# calibration_file=line_calibration.json")
                    self.write_line("# adc_reads_per_channel={}".format(
                        self.line_task.sensor.samples))
                    self.write_line("# base_speed_rad_s={}".format(
                        self.line_task.base_speed))
                    self.write_line("# wheel_kp_v_per_rad_s={}".format(self.p_gain))
                    self.write_line("# line_kp_rad_s_per_mm={}".format(
                        self.line_task.kp_line))
                    self.write_line("# max_correction_rad_s={}".format(
                        self.line_task.max_correction))
                    self.write_line("# motor_period_ms=10")
                    self.write_line("# line_period_ms=20")
                    self.write_line("# ui_period_ms=20")
                    self.write_line("# samples={}".format(
                        self.line_task.capture_count))
                    self.write_line("# stop_reason={}".format(
                        self.line_task.capture_reason))
                    self.write_line("time_ms,centroid_mm")
                    self.line_print_index = 0
                    self.state = self.S_LINE_PRINT

                elif self.line_task.error != self.last_line_error:
                    self.last_line_error = self.line_task.error
                    if self.last_line_error is not None:
                        self.write_line("# LINE FOLLOWING STOPPED: " +
                                        self.last_line_error)

                # Read only when input is available.
                elif self.vcp.any():
                    char_in = self.vcp.read(1)

                    if char_in is not None:
                        try:
                            command = char_in.decode()
                        except AttributeError:
                            command = chr(char_in[0])

                        if command in ("\r", "\n"):
                            yield self.state
                            continue

                        self.write_line(">>> " + command)

                        if command in ("h", "H"):
                            self.print_help()
                            self.write_line(self.message)

                        elif command in ("l", "L"):
                            if (
                                not self.left_data.busy
                                and not self.left_data.ready
                                and not self.right_data.busy
                                and not self.right_data.ready
                            ):
                                self.left_data.cycle = self.vel
                                self.active_data = self.left_data
                                self.active_name = "LEFT"
                                self.output_started = False
                                self.active_data.request = True

                                self.write_line(
                                    "Starting LEFT motor step-response."
                                )
                                self.state = self.S_WAIT_RESULT
                            else:
                                self.write_line(
                                    "Test unavailable: wait for current "
                                    "test/output to finish."
                                )

                        elif command in ("r", "R"):
                            if (
                                not self.left_data.busy
                                and not self.left_data.ready
                                and not self.right_data.busy
                                and not self.right_data.ready
                            ):
                                self.right_data.cycle = self.vel
                                self.active_data = self.right_data
                                self.active_name = "RIGHT"
                                self.output_started = False
                                self.active_data.request = True

                                self.write_line(
                                    "Starting RIGHT motor step-response."
                                )
                                self.state = self.S_WAIT_RESULT
                            else:
                                self.write_line(
                                    "Test unavailable: wait for current "
                                    "test/output to finish."
                                )

                        elif command in ("v", "V"):
                            self.multi_char.reset()
                            self.write_line("Enter a velocity (rad/s):")
                            self.state = self.S_VEL

                        elif command in ("k", "K"):
                            self.write_line(
                                "Enter p/P for proportional gain:"
                            )
                            self.state = self.S_GAIN

                        elif command in ("e", "E"):
                            self.write_line("Exit requested.")
                            self.system.exit_requested = True
                            self.state = self.S_EXIT

                        elif command in ("f", "F", "c", "C"):
                            if self.line_task.running:
                                self.write_line("Line following is already running.")
                            elif self.left_data.busy or self.right_data.busy:
                                self.write_line("Wait for the motor test to finish.")
                            elif self.vel <= 0 or self.p_gain <= 0 or self.kp_line <= 0:
                                self.write_line("Set positive wheel speed, wheel Kp, and line gain first.")
                            else:
                                capture = command in ("c", "C")
                                self.line_task.start(
                                    base_speed=self.vel,
                                    kp_line=self.kp_line,
                                    max_correction=self.max_correction,
                                    capture=capture,
                                )
                                self.last_line_error = None
                                if capture:
                                    self.write_line("Line data capture started.")
                                else:
                                    self.write_line("Line following requested.")

                        elif command in ("s", "S"):
                            self.line_task.stop()
                            self.left_data.velocity_setpoint = 0.0
                            self.right_data.velocity_setpoint = 0.0
                            self.write_line("Stop requested.")

                        else:
                            self.write_line(
                                "Invalid command. Enter h, f, c, s, v, k, l, r, or e."
                            )

            elif self.state == self.S_GAIN:
                # Wait cooperatively for the gain type.
                if self.vcp.any():
                    char_in = self.vcp.read(1)

                    if char_in is not None:
                        try:
                            command = char_in.decode()
                        except AttributeError:
                            command = chr(char_in[0])

                        if command in ("\r", "\n"):
                            pass
                        elif command in ("p", "P"):
                            self.gain_type = "p"
                            self.multi_char.reset()
                            self.write_line("Enter proportional gain:")
                            self.state = self.S_KP_VALUE
                        elif command in ("i", "I"):
                            self.write_line(
                                "Integral control is not implemented."
                            )
                            self.state = self.S_WAIT
                        else:
                            self.write_line("Enter p for proportional gain.")

            elif self.state == self.S_KP_VALUE:
                # Nonblocking numeric input.
                if self.multi_char.update():
                    if self.multi_char.was_changed():
                        self.p_gain = self.multi_char.get_value()

                        for data in (self.left_data, self.right_data):
                            data.kp = self.p_gain

                        self.write_line(
                            "Kp set to {}".format(self.p_gain)
                        )
                        self.write_line(self.message)
                        self.state = self.S_WAIT

            elif self.state == self.S_VEL:
                # Nonblocking numeric input.
                if self.multi_char.update():
                    if self.multi_char.was_changed():
                        self.vel = self.multi_char.get_value()
                        self.write_line(
                            "Velocity set to {}".format(self.vel)
                        )
                        self.write_line(self.message)
                        self.state = self.S_WAIT

            elif self.state == self.S_LINE_PRINT:
                if self.line_print_index < self.line_task.capture_count:
                    index = self.line_print_index
                    centroid = self.line_task.centroid_log[index]
                    field = "" if centroid == self.line_task.NO_LINE else str(centroid)
                    self.write_line("{},{}".format(
                        self.line_task.time_ms[index], field))
                    self.line_print_index += 1
                else:
                    if self.line_task.capture_reason == "complete":
                        self.write_line("# LINE DATA COMPLETE")
                    else:
                        self.write_line("# LINE DATA ABORTED: " +
                                        self.line_task.capture_reason)
                    self.line_task.capture_ready = False
                    self.write_line(self.message)
                    self.state = self.S_WAIT

            elif self.state == self.S_WAIT_RESULT:
                if self.active_data.ready:
                    self.print_index = 0

                    if not self.output_started:
                        self.write_line("")
                        self.write_line(
                            "# {} MOTOR STEP RESPONSE".format(
                                self.active_name
                            )
                        )
                        self.write_line(
                            "command_pct,time_us,position_ticks,"
                            "velocity_ticks_per_s"
                        )
                        self.output_started = True

                    self.state = self.S_PRINT

                elif (
                    not self.active_data.busy
                    and not self.active_data.request
                ):
                    self.write_line(
                        "# {} MOTOR TEST COMPLETE".format(
                            self.active_name
                        )
                    )
                    self.write_line(self.message)
                    self.active_data = None
                    self.state = self.S_WAIT

            elif self.state == self.S_PRINT:
                # Send one CSV row per dispatch to preserve cooperation.
                if self.print_index < self.active_data.count:
                    index = self.print_index

                    self.write_line(
                        "{},{},{},{}".format(
                            self.active_data.effort,
                            self.active_data.time_us[index],
                            self.active_data.position[index],
                            self.active_data.velocity[index],
                        )
                    )
                    self.print_index += 1

                else:
                    self.active_data.clear()
                    self.active_data.ready = False
                    self.state = self.S_WAIT_RESULT

            elif self.state == self.S_EXIT:
                pass

            else:
                raise ValueError("Invalid TaskUser state")

            yield self.state
