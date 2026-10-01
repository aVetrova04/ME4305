from MultiChar import MultiChar

class TaskUser:
    """poll usb input and print test data without blocking motor tasks."""

    S_INIT = 0
    S_WAIT = 1
    S_WAIT_RESULT = 2
    S_PRINT = 3
    S_EXIT = 4
    S_DUTY_SELECT = 5
    S_DUTY_CYCLE = 6

    def __init__(self, vcp, left_data, right_data, system):
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

        self.duty_cycle = 0

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
            "| d/D | Enter duty cycle (-100 to 100):                        |"
        )
        self.write_line(
            "| l/L | Run step-response sequence on left motor               |"
        )
        self.write_line(
            "| r/R | Run step-response sequence on right motor              |"
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
                self.write_line("Enter command (h, l, r, d, or e):")
                self.state = self.S_WAIT

            elif self.state == self.S_WAIT:
                # read only when one character is available
                if self.vcp.any():
                    char_in = self.vcp.read(1)
                    if char_in is not None:
                        try:
                            command = char_in.decode()
                        except AttributeError:
                            command = chr(char_in[0])

                        # ignore enter if the terminal sent a newline
                        if command in ("\r", "\n"):
                            yield self.state
                            continue

                        self.write_line(">>> " + command)

                        if command in ("h", "H"):
                            self.print_help()
                            self.write_line("Enter command (h, l, r, or e):")

                        elif command in ("l", "L"):
                            if (not self.left_data.busy and
                                    not self.left_data.ready and
                                    not self.right_data.busy and
                                    not self.right_data.ready):

                                self.left_data.cycle = self.duty_cycle

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
                            if (not self.left_data.busy and
                                    not self.left_data.ready and
                                    not self.right_data.busy and
                                    not self.right_data.ready):

                                self.right_data.cycle = self.duty_cycle

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

                        elif command in ("d", "D"):
                            self.multi_char.reset()

                            self.write_line("Enter duty cycle (-100 to 100):")
                            self.state = self.S_DUTY_CYCLE


                        elif command in ("e", "E"):
                            self.write_line("Exit requested.")
                            self.system.exit_requested = True
                            self.state = self.S_EXIT

                        else:
                            self.write_line(
                                "Invalid command. Enter h, l, r, or e."
                            )



            elif self.state == self.S_DUTY_CYCLE:

                if self.multi_char.update():
                    if self.multi_char.was_changed():
                        self.duty_cycle = self.multi_char.get_value()

                        # Clamp to valid motor command range.
                        if self.duty_cycle > 100:
                            self.duty_cycle = 100
                        elif self.duty_cycle < -100:
                            self.duty_cycle = -100

                        self.write_line("Duty cycle set to {}%".format(self.duty_cycle))

                    self.write_line("Enter command (h, l, r, d, or e):")
                    self.state = self.S_WAIT

            elif self.state == self.S_WAIT_RESULT:
                if self.active_data.ready:
                    self.print_index = 0
                    if not self.output_started:
                        self.write_line("")
                        self.write_line(
                            "# {} MOTOR STEP RESPONSE".format(self.active_name)
                        )
                        self.write_line(
                            "command_pct,time_us,position_ticks,"
                            "velocity_ticks_per_s"
                        )
                        self.output_started = True
                    self.state = self.S_PRINT
                elif (not self.active_data.busy and
                      not self.active_data.request):
                    self.write_line(
                        "# {} MOTOR TEST COMPLETE".format(self.active_name)
                    )
                    self.write_line("Enter command (h, l, r, or e):")
                    self.active_data = None
                    self.state = self.S_WAIT

            elif self.state == self.S_PRINT:
                # send one csv row per dispatch
                if self.print_index < self.active_data.count:
                    index = self.print_index
                    self.write_line("{},{},{},{}".format(
                        self.active_data.effort,
                        self.active_data.time_us[index],
                        self.active_data.position[index],
                        self.active_data.velocity[index]
                    ))
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
