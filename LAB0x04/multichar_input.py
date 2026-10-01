from pyb import USB_VCP, UART

# A serial port object to use for reading characters
ser = USB_VCP()
# ser = UART(1, 115_200)

# A scalar variable where the completed integer is to be stored. The
# value is left unchanged if the user presses Enter without typing a
# number.
value: int = 0

# A character buffer used to store incoming characters as they're
# received by the command processor
char_buf: list = []

# A set used to quickly check if a character entered by the user is
# a numerical digit.
digits: set = set(map(str, range(10)))

# A set used to quickly check if a character entered by the user is
# a terminator (a carriage return or newline)
term: set = {"\r", "\n"}

# A flag used to track whether or not the command processing is
# still active.
done = False

# If the command is not done being processed, wait for characters
# and then process them individually.
while not done:

    # Only read and process characters if they're available. This
    # avoids blocking on the serial read, although this surrounding
    # loop still runs until the complete number has been entered.
    if ser.any():

        # Each input character is processed individually, so only one
        # needs to be read from the serial port. The character coming
        # from the serial port is of type bytes, so it is decoded as a
        # string.
        char_in = ser.read(1).decode()

        # Digits are simply appended to the incoming character
        # buffer and echoed back to the UI.
        if char_in in digits:
            ser.write(char_in)
            char_buf.append(char_in)

        # Dashes are used for negative values but are only valid if
        # they're the first character in the buffer. Valid dashes
        # are echoed.
        elif char_in == "-" and len(char_buf) == 0:
            ser.write(char_in)
            char_buf.append(char_in)

        # If a "rubout" character is received, as would come from
        # a serial monitor like PuTTY, and there is at least one
        # character in the character buffer then the last character
        # in the buffer is removed. Echoing the validated "rubout"
        # character deletes the previous digit in the serial
        # monitor and also moves the cursor a unit to the left.
        elif char_in == "\x7f" and len(char_buf) > 0:
            ser.write(char_in)
            char_buf.pop()

        # If a termination character is received it is interpreted
        # as the end of data entry.
        elif char_in in term:

            # If the buffer is empty then the value is left unchanged.
            # That way the user can choose not to enter a new value
            # even if they've already issued the command to change it.
            if len(char_buf) == 0:
                ser.write("\r\n")
                ser.write("Value not changed\r\n")
                char_buf = []
                done = True

            # If the character buffer is not empty the termination
            # character is interpreted as an end to the user input.
            # However, if the buffer only contains a single dash, the
            # termination key is ignored because no digits were entered.
            elif char_buf != ["-"]:
                ser.write("\r\n")
                value = int("".join(char_buf))
                ser.write(f"Value set to {value}\r\n")
                char_buf = []
                done = True

        # All other characters, including decimal points, are ignored.
        # The flowchart shows the additional logic needed to accept them.
        else:
            pass
