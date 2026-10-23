#!/usr/bin/env python3
"""Collect one timed Lab 0x06 line run from Romi and save its data and plot."""

import csv
from contextlib import ExitStack
from datetime import datetime
from io import StringIO
from math import isclose
from pathlib import Path
from time import monotonic, sleep

import matplotlib.pyplot as plt
import serial


# Set these to the values verified on the robot before collecting final data.
PORT = "/dev/cu.usbmodem205D32904E332"
BAUD = 115200
BASE_SPEED = 5.0                 # rad/s, magnitude; robot uses negative speed
WHEEL_KP = 1.0                   # V/(rad/s), retune on the actual track
LINE_KP = 0.10                  # intended value; actual value is saved from Romi
READY_PROMPT = "Enter command (h, f, c, s, v, k, l, r, or e):"


def read_line(ser, log, timeout):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                log.write(line + "\n")
                if not line[0].isdigit():
                    print("Romi:", line)
                return line
    raise TimeoutError("Timed out waiting for a response from Romi")


def wait_for(ser, log, wanted, timeout=10):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        line = read_line(ser, log, max(0.25, deadline - monotonic()))
        if line == wanted:
            return
        if line.startswith(("Traceback", "RuntimeError:", "Invalid command",
                            "# LINE DATA ABORTED")):
            raise RuntimeError(line)
    raise TimeoutError("Expected: " + wanted)


def reconnect(ser, ports, log):
    """Reopen USB serial if Romi's soft reset briefly disconnects it."""
    ser.close()
    end = monotonic() + 10
    while monotonic() < end:
        try:
            new_ser = ports.enter_context(serial.Serial(PORT, BAUD, timeout=0.25))
            log.write("# USB serial reconnected\n")
            return new_ser
        except (serial.SerialException, OSError):
            sleep(0.25)
    raise RuntimeError("Romi's serial port did not reconnect; check PORT")


def restart_and_wait_for_menu(ser, ports, log):
    """Start Romi with a fresh heap; recover if boot skips main.py."""
    ser.reset_input_buffer()
    print("Sending Ctrl-C to stop Romi")
    ser.write(b"\x03")                    # Ctrl-C = ASCII 3

    end = monotonic() + 8
    while monotonic() < end:
        try:
            line = read_line(ser, log, 0.6)
        except TimeoutError:
            continue
        except (serial.SerialException, OSError):
            ser = reconnect(ser, ports, log)
            continue
        if line.endswith(">>>"):
            break
    else:
        raise RuntimeError("No >>> prompt after Ctrl-C; stopped before resetting")

    # Reboot clears old imports, arrays and scheduler tasks from RAM.
    # This board sometimes returns to >>> with a name-too-long boot error;
    # when that happens, start main() ourselves in the new session.
    print("Soft rebooting Romi to clear memory")
    ser.write(b"\x04")
    boot_error = ""
    launched = False
    sent_help = False
    calibration_step = 0
    start = monotonic()
    while monotonic() - start < 30:
        try:
            line = read_line(ser, log, 0.6)
        except TimeoutError:
            # After launching main(), ask a running UI to repeat its menu.
            if launched and not sent_help and monotonic() - start > 5:
                try:
                    ser.write(b"h")
                except (serial.SerialException, OSError):
                    ser = reconnect(ser, ports, log)
                    continue
                sent_help = True
            continue
        except (serial.SerialException, OSError):
            ser = reconnect(ser, ports, log)
            continue

        if line == READY_PROMPT:
            if calibration_step == 2:
                input("Place Romi on the line, then press Return to run: ")
            print("Romi's command menu is ready")
            return ser
        if "Place ALL eight sensors over black" in line and calibration_step == 0:
            input("Put all eight sensors over BLACK, then press Return here: ")
            ser.write(b"b")
            calibration_step = 1
            start = monotonic()
            continue
        if "Place ALL eight sensors over white" in line and calibration_step == 1:
            input("Put all eight sensors over WHITE, then press Return here: ")
            ser.write(b"w")
            calibration_step = 2
            start = monotonic()
            continue
        if "Error:" in line or line.startswith("Traceback"):
            boot_error = line
        if line == ">>>":
            if launched:
                raise RuntimeError("main.main() exited before displaying the menu:\n"
                                   + log.getvalue()[-3000:])
            if boot_error:
                print("Boot did not launch main.py:", boot_error)
            print("Starting main.main() automatically")
            ser.write(b"import main; main.main()\r")
            launched = True
            boot_error = ""

    raise RuntimeError("Romi never displayed the c-command menu after launch:\n"
                       + log.getvalue()[-3000:])


def set_value(ser, log, command, prompt, value, confirmation):
    if command:
        ser.write(command.encode("ascii"))  # Single-character UI command.
    wait_for(ser, log, prompt)
    ser.write(("{:g}\r".format(value)).encode("ascii"))
    deadline = monotonic() + 10
    while monotonic() < deadline:
        line = read_line(ser, log, max(0.25, deadline - monotonic()))
        if line.startswith(confirmation):
            accepted = float(line[len(confirmation):].strip())
            if not isclose(accepted, value, abs_tol=1e-6):
                raise ValueError("Romi accepted {} rather than {}".format(
                    accepted, value))
            return
    raise TimeoutError("No confirmation of " + command)


def collect(ser, log):
    # Thirty seconds of driving, then about thirty seconds of buffered printout.
    wait_for(ser, log, "# LINE FOLLOWING DATA", timeout=50)
    metadata = ["# LINE FOLLOWING DATA"]
    settings = {}
    rows = []
    saw_columns = False

    while True:
        line = read_line(ser, log, timeout=8)
        if line.startswith("# LINE DATA ABORTED"):
            raise RuntimeError("Run was interrupted: " + line)
        if line == "# LINE DATA COMPLETE":
            break
        if not saw_columns:
            if line.startswith("# "):
                metadata.append(line)
                if "=" in line:
                    key, value = line[2:].split("=", 1)
                    settings[key] = value
                continue
            if line != "time_ms,centroid_mm":
                raise ValueError("Unexpected CSV header: " + line)
            saw_columns = True
            continue

        fields = line.split(",")
        if len(fields) != 2:
            raise ValueError("Unexpected data row: " + line)
        time_ms = int(fields[0])
        centroid = fields[1]
        if centroid:
            float(centroid)
        rows.append((time_ms, centroid))

    if not saw_columns or settings.get("stop_reason") != "complete":
        raise ValueError("Capture did not finish normally")
    if len(rows) != int(settings["samples"]) or len(rows) != 1500:
        raise ValueError("Expected 1500 rows, got {}".format(len(rows)))
    for key, expected in (("base_speed_rad_s", BASE_SPEED),
                          ("wheel_kp_v_per_rad_s", WHEEL_KP)):
        if not isclose(float(settings[key]), expected, abs_tol=1e-6):
            raise ValueError("Mismatched {} in Romi dataset".format(key))
    actual_line_kp = float(settings["line_kp_rad_s_per_mm"])
    if not isclose(actual_line_kp, LINE_KP, abs_tol=1e-6):
        print("Note: Romi used line Kp {}, expected {}; saved Romi's actual value."
              .format(actual_line_kp, LINE_KP))
    return metadata, settings, rows


def save(folder, metadata, settings, rows):
    folder.mkdir(parents=True, exist_ok=False)
    path = folder / "line_following.csv"
    with path.open("x", newline="", encoding="utf-8") as out:
        for line in metadata:
            out.write(line + "\n")
        writer = csv.writer(out)
        writer.writerow(("time_ms", "centroid_mm"))
        writer.writerows(rows)

    valid = [(t / 1000, float(c)) for t, c in rows if c]
    if len(valid) < 10:
        raise ValueError("Fewer than ten valid line measurements")

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot([t for t, _ in valid], [c for _, c in valid],
            color="0.2", linewidth=1, label="Measured centroid")
    ax.axhline(0, color="0.5", linestyle="--", label="Array center")
    ax.set(xlabel="Time (s)", ylabel="Line centroid (mm)",
           title="Lab 0x06 | base {} rad/s | wheel Kp {} | line Kp {}".format(
               settings["base_speed_rad_s"],
               settings["wheel_kp_v_per_rad_s"],
               settings["line_kp_rad_s_per_mm"]))
    ax.legend()
    ax.grid(True, alpha=0.4)
    fig.tight_layout()
    fig.savefig(folder / "line_following.png", dpi=180)
    plt.close(fig)
    print("Saved {} samples to {}".format(len(rows), path))


def main():
    run_folder = Path("romi_lab06_runs") / datetime.now().strftime(
        "%Y-%m-%d_%H-%M-%S_%f")
    print("Opening", PORT, "at", BAUD, "baud")
    with StringIO() as log:
        with ExitStack() as ports:
            ser = ports.enter_context(serial.Serial(PORT, BAUD, timeout=0.25))
            firmware_ready = False
            try:
                # The serial port must be free of PuTTY or another terminal.
                ser = restart_and_wait_for_menu(ser, ports, log)
                firmware_ready = True
                ser.write(b"k")
                wait_for(ser, log, "Enter p/P for proportional gain:")
                ser.write(b"p")
                set_value(ser, log, "", "Enter proportional gain:",
                          WHEEL_KP, "Kp set to ")
                set_value(ser, log, "v", "Enter a velocity (rad/s):",
                          BASE_SPEED, "Velocity set to ")
                # The line gain comes from user_task.kp_line in Romi's main.py.
                # This UI has no g command. The data header verifies the value.
                ser.write(b"c")
                wait_for(ser, log, "Line data capture started.")
                print("Capturing for about 30 seconds, then transferring the data...")
                metadata, settings, rows = collect(ser, log)
                save(run_folder, metadata, settings, rows)
                ser.write(b"e")
                wait_for(ser, log, "Exit requested.")
                print("Complete. Results:", run_folder)
            except BaseException:
                if firmware_ready:
                    try:
                        ser.write(b"s")
                    except (serial.SerialException, OSError):
                        pass
                print("Stopped before a complete dataset was saved.")
                raise


if __name__ == "__main__":
    main()
