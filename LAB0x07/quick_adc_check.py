"""Standalone eight-channel line sensor display for the Romi USB serial port.

Run from the MicroPython REPL after stopping main.py with Ctrl-C.
This is a wiring check; it does not run the motors or the scheduler.
"""

from pyb import ADC, Pin, USB_VCP, delay


# (labeled sensor lead number, MCU pin name) from NUCLEO PIN LAYOUT-3.pdf.
# W/Y is the wire color. Confirm actual left/right order by moving a line under
# the stationary array before using these numbers as centroid positions.
# If the wiring changes, edit this map only. Keep the pairs in sensor order.
SENSOR_MAP = (
    (1, "C0"),  # W1: CN8 pin 38
    (2, "C1"),  # Y2: CN8 pin 36
    (3, "C2"),  # W3: CN7 pin 35
    (4, "C3"),  # Y4: CN7 pin 37
    (5, "A4"),  # W5: CN8 pin 32
    (6, "C4"),  # Y6: CN10 pin 34
    (7, "B0"),  # W7: CN8 pin 34
    (8, "B1"),  # Y8: CN10 pin 24
)


# Current Lab 0x05 motor and encoder pins from main.py. Update if they move.
OTHER_PINS = ("A0", "A1", "A6", "A7", "C6", "C5",
              "A8", "A9", "B4", "B5")


def make_frame(readings, sensor_map, full_scale=4095, width=32):
    lines = ["Romi line sensor ADC (q to quit)",
             "Sensor  MCU pin  Raw ADC  Bar"]

    for (sensor_number, pin_name), value in zip(sensor_map, readings):
        clipped = min(full_scale, max(0, value))
        size = int(width * clipped / full_scale)
        lines.append("{:>6}  {:>7}  {:>7}  |{}{}|".format(
            sensor_number, pin_name, value,
            "#" * size, " " * (width - size)))

    frame = "\x1b[H"
    for row in lines:
        frame += "\x1b[2K" + row + "\r\n"
    return frame


def make_adcs(sensor_map):
    if len(sensor_map) != 8:
        raise ValueError("Enter exactly eight connected sensors in SENSOR_MAP")

    numbers = []
    pins = []
    adcs = []
    for number, pin_name in sensor_map:
        if not isinstance(number, int) or not 1 <= number <= 13:
            raise ValueError("Set each physical sensor number from 1 to 13")
        if not isinstance(pin_name, str) or not pin_name:
            raise ValueError("Set each MCU pin name, for example 'C0'")
        if number in numbers or pin_name in pins:
            raise ValueError("A sensor number or MCU pin is listed twice")
        if pin_name in OTHER_PINS:
            raise ValueError("{} is already used by the Lab 0x05 motors/encoders".format(pin_name))

        numbers.append(number)
        pins.append(pin_name)
        try:
            adcs.append(ADC(getattr(Pin.cpu, pin_name)))
        except (AttributeError, ValueError) as error:
            raise ValueError("Check pin {}: {}".format(pin_name, error))

    return adcs


def plot_adc(adcs, sensor_map, refresh_ms=100):
    vcp = USB_VCP()
    vcp.write(b"\x1b[2J\x1b[H")

    try:
        while True:
            if vcp.any():
                command = vcp.read()
                if command and (b"q" in command or b"Q" in command):
                    break

            # One ADC conversion per connected sensor on each screen update.
            readings = [adc.read() for adc in adcs]
            vcp.write(make_frame(readings, sensor_map).encode())
            delay(refresh_ms)
    except KeyboardInterrupt:
        pass
    finally:
        vcp.write(b"\r\nSensor display stopped.\r\n")


def main():
    adcs = make_adcs(SENSOR_MAP)
    plot_adc(adcs, SENSOR_MAP)


if __name__ == "__main__":
    main()
