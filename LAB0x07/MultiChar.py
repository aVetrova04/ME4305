class MultiChar:

    def __init__(self, vcp):
        self.vcp = vcp

        self.value = 0
        self.char_buf = []

        self.digits = set("0123456789")
        self.terminators = set("\r\n")

        self.done = False
        self.changed = False

    def reset(self):
        """Reset the input processor for a new value."""
        self.char_buf = []
        self.done = False
        self.changed = False

    def update(self):
        """
        Process at most one available character.

        Returns True when the user has finished entering a value.
        The completed value is available in self.value.
        """

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

        if char_in in self.digits:
            self.vcp.write(char_in.encode())
            self.char_buf.append(char_in)

        elif char_in == "-" and len(self.char_buf) == 0:
            self.vcp.write(char_in.encode())
            self.char_buf.append(char_in)

        elif char_in == "\x7f":
            if len(self.char_buf) > 0:
                self.vcp.write(char_in.encode())
                self.char_buf.pop()

        elif char_in in self.terminators:

            # Empty input: leave value unchanged.
            if len(self.char_buf) == 0:
                self.vcp.write(b"\r\n")
                self.vcp.write(b"Value not changed\r\n")

                self.done = True
                self.changed = False

            # A lone '-' is not a valid integer.
            elif self.char_buf == ["-"]:
                pass

            # Valid integer.
            else:
                self.vcp.write(b"\r\n")

                self.value = float("".join(self.char_buf))

                self.vcp.write(
                    ("Value set to {}\r\n".format(self.value)).encode()
                )

                self.done = True
                self.changed = True

        else:
            pass

        return self.done

    def get_value(self):
        """Return the most recently completed integer value."""
        return self.value

    def is_done(self):
        """Return True when input has been terminated."""
        return self.done

    def was_changed(self):
        """Return True if the user entered a new value."""
        return self.changed