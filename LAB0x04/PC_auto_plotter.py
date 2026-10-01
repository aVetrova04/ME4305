"""Collect one serial dataset, save it as CSV, and create a plot.

This is starter code rather than a complete Lab 0x04 solution. It shows a
robust pattern for one command and one dataset. Students will need to adapt the
command sequence, expected messages, and test loop to match their firmware.
"""

from datetime import datetime
from time import monotonic

from matplotlib import pyplot
from serial import Serial, SerialException


# Serial-port settings. Change SERIAL_PORT to the VCP assigned by the computer.
SERIAL_PORT = "COM6"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.25

# This example requests one left-motor dataset using the required Lab 0x04 menu.
# Single-character commands are sent as soon as they are chosen; they do not
# need a line ending. Students will extend this into the duty-cycle and
# two-motor command sequence. A multicharacter numeric value does need the line
# ending expected by the firmware so that it knows when entry is complete.
START_COMMAND = "l"
NUMERIC_LINE_ENDING = "\r\n"

# The firmware should print this marker on its own line after the last data row.
# Using an explicit marker is more reliable than assuming that a quiet serial
# port means the dataset is complete.
END_MARKER = "End of data"

# Stop waiting and report an error instead of hanging forever if the firmware
# does not respond or stops transmitting partway through a dataset.
#
# monotonic() returns a steadily increasing time in seconds. Its starting value
# is arbitrary, so it is not used as a date or time of day. Subtracting an old
# reading from a new reading gives the elapsed time without being affected if
# the computer's clock is adjusted while the program is running.
FIRST_RESPONSE_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 5.0

# The run name becomes part of each output filename. Students may replace this
# with a motor name, duty cycle, or another useful test identifier.
RUN_NAME = "serial_response"


def cleaned_fields(line):
    """Return stripped comma-separated fields after removing a comment."""
    content = line.split("#", 1)[0].strip()
    if not content:
        return []

    fields = []
    for field in content.split(","):
        fields.append(field.strip())

    return fields


def collect_dataset(ser):
    """Read one headed numeric dataset, ending at END_MARKER.

    Status and prompt lines before the CSV header are displayed and ignored.
    After the header is found, malformed rows are reported and skipped rather
    than terminating the whole collection.
    """
    headers = None
    columns = None
    serial_line_number = 0
    # Save one reading from the monotonic clock. Later readings are compared
    # with this one to determine how many seconds have elapsed.
    waiting_since = monotonic()

    while True:
        raw_line = ser.readline()

        # readline() returns b"" when its short serial timeout expires. Keep
        # polling until the longer application timeout has also expired.
        if not raw_line:
            timeout = (FIRST_RESPONSE_TIMEOUT if headers is None
                       else BETWEEN_LINES_TIMEOUT)
            if monotonic() - waiting_since >= timeout:
                if headers is None:
                    raise TimeoutError("No CSV header was received.")
                raise TimeoutError("Serial data stopped before the end marker.")
            continue

        waiting_since = monotonic()
        serial_line_number += 1
        line = raw_line.decode("utf-8", errors="replace").strip()

        if line == END_MARKER:
            break

        fields = cleaned_fields(line)
        if not fields:
            continue

        # The first comma-separated line is treated as the CSV header. Lines
        # printed before it may contain prompts, acknowledgements, or metadata.
        if headers is None:
            if len(fields) < 2:
                print(f"Serial message: {line}")
                continue

            headers = fields
            columns = []
            for header in headers:
                columns.append([])
            print(f"CSV header received: {', '.join(headers)}")
            continue

        # Ignore extra fields, as in HW 0, but reject rows that do not contain
        # enough values for every named column.
        if len(fields) < len(headers):
            print(f"Rejected serial line {serial_line_number}: "
                  f"expected {len(headers)} values, received {len(fields)}")
            continue

        try:
            values = []
            for field in fields[:len(headers)]:
                values.append(float(field))
        except ValueError:
            print(f"Rejected serial line {serial_line_number}: "
                  "one or more fields are not numeric")
            continue

        for column, value in zip(columns, values):
            column.append(value)

    if headers is None:
        raise ValueError("The end marker arrived before a CSV header.")
    if not columns[0]:
        raise ValueError("The dataset did not contain any valid numeric rows.")

    return headers, columns


def save_csv(filename, headers, columns):
    """Save the collected columns using the received header labels."""
    with open(filename, "w", encoding="utf-8", newline="") as csv_file:
        csv_file.write(",".join(headers) + "\n")
        for row in zip(*columns):
            formatted_values = []
            for value in row:
                formatted_values.append(f"{value:.12g}")
            csv_file.write(",".join(formatted_values) + "\n")


def save_plot(filename, headers, columns):
    """Plot every response column against the first collected column."""
    figure, axes = pyplot.subplots()

    for index in range(1, len(headers)):
        axes.plot(columns[0], columns[index], label=headers[index])

    axes.set_xlabel(headers[0])
    if len(headers) == 2:
        axes.set_ylabel(headers[1])
    else:
        axes.set_ylabel("Measured response")
        axes.legend()

    axes.set_title(RUN_NAME.replace("_", " ").title())
    axes.grid(True)
    figure.tight_layout()
    figure.savefig(filename, dpi=200)
    pyplot.close(figure)


def main():
    """Collect, save, and plot one response from the configured serial port."""
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    data_filename = f"{timestamp}_{RUN_NAME}.csv"
    plot_filename = f"{timestamp}_{RUN_NAME}.png"

    print(f"Opening {SERIAL_PORT} at {BAUDRATE} baud")
    try:
        # The context manager closes the port even if collection raises an
        # exception or the user interrupts the program.
        with Serial(SERIAL_PORT,
                    baudrate=BAUDRATE,
                    timeout=SERIAL_TIMEOUT) as ser:
            print("Discarding any unread serial data")
            ser.reset_input_buffer()

            print(f"Sending command: {START_COMMAND!r}")
            ser.write(START_COMMAND.encode("utf-8"))

            print("Waiting for a CSV header and data")
            headers, columns = collect_dataset(ser)

    except SerialException as error:
        raise SystemExit(f"Serial-port error: {error}") from error

    save_csv(data_filename, headers, columns)
    save_plot(plot_filename, headers, columns)

    print(f"Saved {len(columns[0])} valid data rows to {data_filename}")
    print(f"Saved plot to {plot_filename}")


if __name__ == "__main__":
    main()
