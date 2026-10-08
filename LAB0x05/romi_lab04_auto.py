#!/usr/bin/env python3

import csv
from datetime import datetime
from math import isfinite
from pathlib import Path
from time import monotonic

from matplotlib import pyplot
from serial import Serial, SerialException


# replace this with the Romi port reported by your Mac
SERIAL_PORT = "/dev/cu.usbmodem205D32904E332"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.25

# MultiChar.py accepts signed integers, not decimal values
TESTS = (("left", 25, "l"), ("right", -25, "r"))
NUMERIC_LINE_ENDING = "\r"

CSV_HEADER = "command_pct,time_us,position_ticks,velocity_ticks_per_s"
DUTY_PROMPT = "Enter duty cycle (-100 to 100):"
SAMPLES_PER_TEST = 100
FIRST_RESPONSE_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 8.0


def read_line(ser, deadline, purpose):
    """Return a complete nonempty serial line before the deadline."""
    while monotonic() < deadline:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                return line
    raise TimeoutError("Timed out waiting for " + purpose)


def wait_for_line(ser, expected, purpose, seconds):
    """Skip echoes and status messages until the requested line arrives."""
    deadline = monotonic() + seconds
    while True:
        line = read_line(ser, deadline, purpose)
        if line == expected:
            return
        if line.startswith(("Traceback", "Test unavailable", "Value not changed")):
            raise RuntimeError("Romi reported: " + line)
        if line.startswith("Duty cycle set to "):
            raise ValueError("Unexpected duty confirmation: " + line)
        print("Romi:", line)


def set_duty_cycle(ser, duty):
    """Send d immediately, then enter a signed whole-number duty with CR."""
    if type(duty) is not int or not -100 <= duty <= 100:
        raise ValueError("Configured duty must be an integer from -100 to 100")

    ser.write(b"d")
    wait_for_line(ser, DUTY_PROMPT, "duty prompt", FIRST_RESPONSE_TIMEOUT)

    # MultiChar echoes the characters and finishes when it receives CR.
    ser.write((str(duty) + NUMERIC_LINE_ENDING).encode("ascii"))
    wait_for_line(ser, "Duty cycle set to {}%".format(float(duty)),
                  "duty confirmation", FIRST_RESPONSE_TIMEOUT)


def collect_dataset(ser, motor_name, duty):
    """Collect exactly one marked, headed 100-row response."""
    start_marker = "# {} MOTOR STEP RESPONSE".format(motor_name.upper())
    end_marker = "# {} MOTOR TEST COMPLETE".format(motor_name.upper())

    started = False
    header_seen = False
    rows = []
    deadline = monotonic() + FIRST_RESPONSE_TIMEOUT

    while True:
        line = read_line(ser, deadline, motor_name + " response")
        deadline = monotonic() + BETWEEN_LINES_TIMEOUT

        if line.startswith(("Traceback", "Test unavailable")):
            raise RuntimeError("Romi reported: " + line)

        if not started:
            if line == start_marker:
                started = True
            elif line.startswith("# ") and "MOTOR STEP RESPONSE" in line:
                raise ValueError("Unexpected motor start marker: " + line)
            elif line:
                print("Romi:", line)
            continue

        if not header_seen:
            if line == CSV_HEADER:
                header_seen = True
            elif line == end_marker:
                raise ValueError("Motor test ended before its CSV header")
            elif line.startswith("# ") and "MOTOR STEP RESPONSE" in line:
                raise ValueError("Duplicate motor start marker: " + line)
            continue

        if line == end_marker:
            if len(rows) != SAMPLES_PER_TEST:
                raise ValueError("{} returned {} rows; expected {}".format(
                    motor_name, len(rows), SAMPLES_PER_TEST))
            return rows

        if line.startswith("# ") and "MOTOR TEST COMPLETE" in line:
            raise ValueError("Unexpected motor end marker: " + line)

        # The menu and other messages may share the VCP with the CSV rows.
        if not line or line[0] not in "+-.0123456789":
            continue

        fields = [field.strip() for field in line.split(",")]
        if len(fields) != 4:
            raise ValueError("Malformed CSV row: " + line)
        try:
            values = [float(field) for field in fields]
        except ValueError as error:
            raise ValueError("Nonnumeric CSV row: " + line) from error
        if not all(isfinite(value) for value in values):
            raise ValueError("Nonfinite CSV row: " + line)
        if values[0] != duty:
            raise ValueError("Row duty does not match {}%: {}".format(duty, line))

        rows.append(fields)
        if len(rows) > SAMPLES_PER_TEST:
            raise ValueError("Received more than 100 rows for " + motor_name)


def save_csv(path, rows):
    """Save numeric readings under the exact firmware column labels."""
    with path.open("x", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(CSV_HEADER.split(","))
        writer.writerows(rows)


def save_plot(path, motor_name, duty, rows):
    """Plot encoder position and velocity against time since the step."""
    if path.exists():
        raise FileExistsError("Plot already exists: " + str(path))

    seconds = [float(row[1]) / 1_000_000 for row in rows]
    position = [float(row[2]) for row in rows]
    velocity = [float(row[3]) for row in rows]

    figure, axes = pyplot.subplots(2, 1, sharex=True, figsize=(8, 6))
    axes[0].plot(seconds, position, color="#333333")
    axes[1].plot(seconds, velocity, color="#555555")
    axes[0].set_ylabel("Position (encoder ticks)")
    axes[1].set_ylabel("Velocity (encoder ticks/s)")
    axes[1].set_xlabel("Time after step (s)")
    for axis in axes:
        axis.grid(True)
    figure.suptitle("{} motor open-loop response | {:+d}% duty".format(
        motor_name.title(), duty))
    figure.tight_layout()
    try:
        figure.savefig(path, dpi=200)
    finally:
        pyplot.close(figure)


def main():
    if SERIAL_PORT.endswith("XXXX"):
        raise SystemExit("Set SERIAL_PORT to your Mac's Romi VCP path first")
    if len(TESTS) != 2 or {side for side, _, _ in TESTS} != {"left", "right"}:
        raise ValueError("Configure one left and one right trial in TESTS")

    run_dir = Path("romi_runs") / datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
    run_dir.mkdir(parents=True, exist_ok=False)
    print("Opening", SERIAL_PORT, "at", BAUDRATE, "baud")

    try:
        with Serial(SERIAL_PORT, baudrate=BAUDRATE,
                    timeout=SERIAL_TIMEOUT) as ser:
            try:
                ser.reset_input_buffer()

                for motor_name, duty, command in TESTS:
                    if (motor_name, command) not in (("left", "l"), ("right", "r")):
                        raise ValueError("Motor and command do not match: " + motor_name)

                    print("Starting {} motor at {:+d}%".format(motor_name, duty))
                    set_duty_cycle(ser, duty)
                    ser.write(command.encode("ascii"))  # no newline after l or r
                    rows = collect_dataset(ser, motor_name, duty)

                    duty_tag = "plus" if duty >= 0 else "minus"
                    stem = "{}_{}{}pct".format(motor_name, duty_tag, abs(duty))
                    csv_path = run_dir / (stem + ".csv")
                    plot_path = run_dir / (stem + ".png")
                    save_csv(csv_path, rows)
                    save_plot(plot_path, motor_name, duty, rows)
                    print("Saved {} rows to {} and {}".format(
                        len(rows), csv_path, plot_path))

                ser.write(b"e")  # exit is also a single-character command
                wait_for_line(ser, "Exit requested.",
                              "exit confirmation", FIRST_RESPONSE_TIMEOUT)
                print("Done. Saved results in", run_dir)

            except (Exception, KeyboardInterrupt):
                # The board's main.py stops both motors in its finally block.
                try:
                    ser.write(b"\x03")
                except (SerialException, OSError):
                    pass
                raise

    except SerialException as error:
        raise SystemExit("Serial-port error: " + str(error)) from error


if __name__ == "__main__":
    main()
