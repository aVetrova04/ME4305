#!/usr/bin/env python3
"""Run P-control step tests on Romi and save the responses on the Mac."""

import csv
from datetime import datetime
from math import isclose, isfinite
from pathlib import Path
from time import monotonic

from matplotlib import pyplot as plt
from serial import Serial, SerialException


# change this if the Mac assigns Romi a different VCP port.
PORT = "/dev/cu.usbmodem205D32904E332"
BAUD = 115200
TESTS = [
    ("left", 5.0, 0.2),
    ("left", 5.0, 0.5),
    ("left", 5.0, 1.0),
]

CSV_HEADER = ["time_us", "setpoint_rad_s", "velocity_rad_s",
              "limited_voltage_v", "position_ticks"]
SAMPLES = 100


def read_line(ser, seconds, log):
    """Wait for one serial line, with a timeout for lost connections."""
    end = monotonic() + seconds
    while monotonic() < end:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                log.write(line + "\n")
                return line
    raise TimeoutError("Romi did not send the next expected line")


def wait_for(ser, expected, log, seconds=20):
    """Ignore echoed input and menu text until the expected prompt arrives."""
    end = monotonic() + seconds
    while monotonic() < end:
        line = read_line(ser, max(0.01, end - monotonic()), log)
        if line == expected:
            return
        if (line.startswith(("Invalid", "Traceback", "Test unavailable")) or
                "TEST ABORTED" in line or "TEST FAILED" in line):
            raise RuntimeError("Romi: " + line)
        print("Romi:", line)
    raise TimeoutError("Waiting for " + expected)


def wait_for_value(ser, prefix, requested, log):
    end = monotonic() + 20
    while monotonic() < end:
        line = read_line(ser, max(0.01, end - monotonic()), log)
        if line.startswith(prefix):
            text = line[len(prefix):].split()[0]
            try:
                actual = float(text)
            except ValueError as error:
                raise ValueError("Bad value confirmation: " + line) from error
            if not isclose(actual, requested, rel_tol=1e-6, abs_tol=1e-7):
                raise ValueError("Romi accepted a different setting: " + line)
            return
        if line.startswith(("Invalid", "Traceback", "Test unavailable")):
            raise RuntimeError("Romi: " + line)
        print("Romi:", line)
    raise TimeoutError("Waiting for " + prefix)


def set_trial(ser, setpoint, kp, log):
    """Send single-letter commands first, numeric values with a CR."""
    ser.write(b"v")
    wait_for(ser, "Enter a velocity (rad/s):", log)
    ser.write(("{:g}\r".format(setpoint)).encode("ascii"))
    wait_for_value(ser, "Velocity set to ", setpoint, log)

    ser.write(b"k")
    wait_for(ser, "Enter p/P for proportional gain:", log)
    ser.write(b"p")
    wait_for(ser, "Enter proportional gain:", log)
    ser.write(("{:g}\r".format(kp)).encode("ascii"))
    wait_for_value(ser, "Kp set to ", kp, log)


def collect(ser, motor, setpoint, kp, log):
    """Read metadata and 100 numeric rows through the completion marker."""
    side = motor.upper()
    start = "# {} MOTOR CLOSED-LOOP STEP RESPONSE".format(side)
    finish = "# {} MOTOR TEST COMPLETE".format(side)
    wait_for(ser, start, log, seconds=30)

    notes = []
    settings = {}
    header_found = False
    rows = []
    while True:
        line = read_line(ser, 8, log)
        if "TEST ABORTED" in line or "TEST FAILED" in line:
            raise RuntimeError("Romi: " + line)
        if line == finish:
            break
        if line.startswith("# ") and not header_found:
            notes.append(line)
            if "=" in line:
                key, value = line[2:].split("=", 1)
                settings[key.strip()] = value.strip()
            continue
        if not header_found:
            if line.split(",") != CSV_HEADER:
                raise ValueError("Unexpected CSV header: " + line)
            header_found = True
            continue

        fields = [field.strip() for field in line.split(",")]
        if len(fields) != len(CSV_HEADER):
            raise ValueError("Bad data row: " + line)
        try:
            values = [float(field) for field in fields]
        except ValueError as error:
            raise ValueError("Nonnumeric data row: " + line) from error
        if not all(isfinite(value) for value in values):
            raise ValueError("Nonfinite data row: " + line)
        rows.append(fields)
        if len(rows) > SAMPLES:
            raise ValueError("Romi sent more than 100 rows")

    if not header_found or len(rows) != SAMPLES:
        raise ValueError("Incomplete response: {} of {} rows".format(len(rows), SAMPLES))
    required = ("motor", "control", "setpoint_rad_s", "kp_v_per_rad_s",
                "motor_period_ms", "ui_period_ms", "duration_ms",
                "battery_voltage_v", "voltage_min_v", "voltage_max_v", "units")
    if any(key not in settings for key in required):
        raise ValueError("The firmware header is missing a test setting")
    if settings["motor"] != side or settings["control"] != "P":
        raise ValueError("Romi returned the wrong motor or controller")
    if not isclose(float(settings["setpoint_rad_s"]), setpoint, abs_tol=1e-6):
        raise ValueError("Wrong setpoint in firmware header")
    if not isclose(float(settings["kp_v_per_rad_s"]), kp, abs_tol=1e-6):
        raise ValueError("Wrong Kp in firmware header")
    return notes, settings, rows


def save_run(folder, number, motor, setpoint, kp, notes, settings, rows):
    base = folder / "trial_{:02d}_{}_P".format(number, motor)
    with base.with_suffix(".txt").open("x", encoding="utf-8") as out:
        out.write("\n".join(notes) + "\n")
    with base.with_suffix(".csv").open("x", newline="", encoding="utf-8") as out:
        writer = csv.writer(out)
        writer.writerow(CSV_HEADER)
        writer.writerows(rows)

    t = [float(row[0]) / 1_000_000 for row in rows]
    speed = [float(row[2]) for row in rows]
    voltage = [float(row[3]) for row in rows]
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    axes[0].plot(t, [setpoint] * len(t), "k--", label="Setpoint")
    axes[0].plot(t, speed, color="0.4", label="Measured")
    axes[0].set_ylabel("Wheel velocity (rad/s)")
    axes[0].legend()
    axes[1].plot(t, voltage, color="0.35")
    axes[1].set_ylabel("Limited voltage (V)")
    axes[1].set_xlabel("Time after step (s)")
    for axis in axes:
        axis.grid(True)
    fig.suptitle("{} motor | P | setpoint {:g} rad/s | Kp {:g} V/(rad/s)\n"
                 "motor period {} ms".format(motor.title(), setpoint, kp,
                                              settings["motor_period_ms"]))
    fig.tight_layout()
    try:
        fig.savefig(base.with_suffix(".png"), dpi=200)
    finally:
        plt.close(fig)
    print("Saved 100 rows and plot:", base)


def main():
    if not TESTS:
        raise SystemExit("Add at least one test before running")
    if any(motor not in ("left", "right") or not isfinite(speed) or
           abs(speed) > 50 or not isfinite(kp) or not 0 <= kp <= 20
           for motor, speed, kp in TESTS):
        raise ValueError("A test has an invalid motor, velocity, or Kp")

    folder = Path("romi_lab05_runs") / datetime.now().strftime(
        "%Y-%m-%d_%H-%M-%S_%f")
    folder.mkdir(parents=True, exist_ok=False)
    print("Opening", PORT, "at", BAUD, "baud")

    try:
        with (folder / "serial.log").open("x", encoding="utf-8", buffering=1) as log:
            with Serial(PORT, baudrate=BAUD, timeout=0.25) as ser:
                try:
                    ser.reset_input_buffer()
                    for number, (motor, speed, kp) in enumerate(TESTS, 1):
                        print("Trial {}: {} motor, {:g} rad/s, Kp {:g}".format(
                            number, motor, speed, kp))
                        set_trial(ser, speed, kp, log)
                        ser.write(motor[0].encode("ascii"))
                        notes, settings, rows = collect(ser, motor, speed, kp, log)
                        save_run(folder, number, motor, speed, kp,
                                 notes, settings, rows)
                    ser.write(b"e")
                    wait_for(ser, "Exit requested.", log)
                    print("Done. Results:", folder)
                except BaseException:
                    try:
                        ser.write(b"\x03")
                    except (SerialException, OSError):
                        pass
                    print("Stopped. Check", folder / "serial.log")
                    raise
    except SerialException as error:
        raise SystemExit("Serial error: " + str(error)) from error


if __name__ == "__main__":
    main()
