# Standalone Lab 0x06 sensor test for PuTTY. Run with motors disabled and with
# the cooperative scheduler stopped. Delay and printing loops are allowed here.
# Edit main() to construct YOUR driver and bind read_values to its ADC-reading
# method. It must return 13 raw readings or averages using the same acquisition
# method/configuration as calibration and line following. No driver API is fixed.

from pyb import USB_VCP, delay


def make_frame(values, channels, full_scale=4095, width=36):
    if len(values) != 13 or len(channels) != 13:
        raise ValueError("Expected all 13 channel readings")
    lines = ["Romi ADC readings (0-4095); q to quit", "Ch  Reading  Bar"]
    for channel, value in zip(channels, values):
        size = int(width * min(full_scale, max(0, value)) / full_scale)
        lines.append("{:2d}  {:7.1f}  |{}{}|".format(
            channel, value, "#" * size, " " * (width - size)))
    # Cursor home and clear each row; fixed-width bars replace the old frame.
    frame = "\x1b[H"
    for row in lines:
        frame += "\x1b[2K" + row + "\r\n"
    return frame


def plot_adc(read_values, channels, refresh_ms=100):
    vcp = USB_VCP()
    vcp.write("\x1b[2J\x1b[H")  # Clear screen once; home cursor on each refresh.
    try:
        while True:
            if vcp.any():
                command = vcp.read()
                if command and (b"q" in command or b"Q" in command):
                    break
            vcp.write(make_frame(read_values(), channels))
            delay(refresh_ms)  # Display refresh rate, not ADC sample frequency.
    except KeyboardInterrupt:
        pass
    finally:
        vcp.write("\r\nSensor display stopped.\r\n")


def main():
    # STUDENT ADAPTER: import and construct your driver here with all 13 pins.
    # For example, if your own object is named sensor and its method read_raw:
    # read_values = sensor.read_raw  # Bind the method; do not call it here.
    read_values = None
    channels = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
    # Change this order to match the readings returned by YOUR driver.
    if read_values is None:
        print("Edit main(): construct your driver and set read_values/channel order.")
        return
    plot_adc(read_values, channels)


if __name__ == "__main__":
    main()
