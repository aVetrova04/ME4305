class MultiChar:

    def __init__(self, vcp):
        self.vcp = vcp

        self.value = 0.0
        self.char_buf = []

        self.digits = set("0123456789")
        self.terminators = set("\r\n")

        self.done = False
        self.changed = False

    def reset(self):
        """reset the input processor for a new value."""
        self.char_buf = []
        self.done = False
        self.changed = False

    def update(self):
        """process at most one available character."""

        if self.done:
            return True

        if not self.vcp.any():
            return False

        char_in = self.vcp.read(1)

        if char_in is None:
            return False

        try:
            char_in = char_in.decode()
        except AttributeError:
            char_in = chr(char_in[0])

        # accept digits
        if char_in in self.digits:
            self.vcp.write(char_in.encode())
            self.char_buf.append(char_in)

        # accept a minus sign only as the first character
        elif char_in == "-" and len(self.char_buf) == 0:
            self.vcp.write(char_in.encode())
            self.char_buf.append(char_in)

        # accept no more than one decimal point
        elif char_in == "." and "." not in self.char_buf:
            self.vcp.write(char_in.encode())
            self.char_buf.append(char_in)

        # support delete and backspace
        elif char_in in ("\x7f", "\x08"):
            if len(self.char_buf) > 0:
                self.char_buf.pop()
                self.vcp.write(b"\b \b")

        # finish entry when enter is pressed
        elif char_in in self.terminators:
            self.vcp.write(b"\r\n")

            # these entries are incomplete
            if (
                len(self.char_buf) == 0
                or self.char_buf == ["-"]
                or self.char_buf == ["."]
                or self.char_buf == ["-", "."]
            ):
                self.vcp.write(b"Invalid or incomplete value\r\n")
                self.done = True
                self.changed = False

            else:
                self.value = float("".join(self.char_buf))

                self.vcp.write(
                    ("Value entered: {}\r\n".format(self.value)).encode()
                )

                self.done = True
                self.changed = True

        # ignore unsupported characters
        else:
            pass

        return self.done

    def get_value(self):
        """return the most recently completed numeric value."""
        return self.value

    def is_done(self):
        """return true when input has been terminated."""
        return self.done

    def was_changed(self):
        """return true if a complete numeric value was entered."""
        return self.changed
