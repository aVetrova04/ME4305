#!/usr/bin/env python3
"""Collect Lab 09 Step 5 P responses using observer velocity feedback."""
import argparse
import csv
from contextlib import ExitStack
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
from statistics import mean, pstdev
from time import monotonic, monotonic_ns, sleep
import zipfile

PORT = '/dev/cu.usbmodem205D32904E332'
BAUD = 115200
COLUMNS = ['time_s', 'setpoint_rad_s', 'feedback_rad_s',
           'estimated_rad_s', 'limited_voltage_V']
STEP4_BUILD = 'lab09-step4-10ms-gc1'
STEP5_BUILD = 'lab09-step5-10ms-observer-p'
SCRIPT_FOLDER = Path(__file__).resolve().parent
MOTOR_PERIOD_US = 10000
MAX_SAMPLE_GAP_US = 15000

# keep these conditions the same for both tests and later comparisons
SURFACE = 'lab track surface, clear straight section'
LOADING = 'Romi only; no added weight; connected USB cable with slack'
TEAMMATES = ['Logan Ribarich', 'Arina Vetrova']
MEASURED_BATTERY_V = 7.65  # supplied meter reading; not measured while driving


class Reader:
    # keep partial lines between reads, including a repl prompt without a newline
    def __init__(self, ser, log):
        self.ser, self.log = ser, log
        self.pending = bytearray()

    def read_line(self, timeout=10):
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            if b'\n' in self.pending:
                raw, _, rest = self.pending.partition(b'\n')
                self.pending = bytearray(rest)
                line = raw.decode('utf-8', 'replace').strip()
            elif self.pending and re.fullmatch(rb'(?:\s*>>>)+\s*', self.pending):
                self.pending.clear()
                line = '>>>'
            else:
                self.pending.extend(self.ser.read(max(1, min(self.ser.in_waiting, 1024))))
                if len(self.pending) > 8192:
                    raise ValueError('serial line too long')
                continue
            if not line:
                continue
            self.log.write(line + '\n')
            self.log.flush()
            if not (line[0].isdigit() or line[0] == '-'):
                print('Romi:', line)
            return line
        raise TimeoutError('Romi did not send the expected response')


def wait_for(reader, prefix, timeout=10):
    end = monotonic() + timeout
    while monotonic() < end:
        line = reader.read_line(max(.01, end - monotonic()))
        if line.startswith(prefix):
            return line
        if line.startswith(('# LAB09 ERROR', 'Traceback', 'MemoryError', 'ValueError')):
            raise RuntimeError(line)
    raise TimeoutError('Expected ' + prefix)


def wait_for_repl(reader, timeout=10):
    # The old main.py may print a traceback while Ctrl-C returns to REPL.
    # It is safe to continue once MicroPython itself shows the prompt.
    end = monotonic() + timeout
    while monotonic() < end:
        line = reader.read_line(max(.01, end - monotonic()))
        if line.startswith('>>>'):
            return
    raise TimeoutError('No MicroPython REPL prompt after Ctrl-C')


def restart_and_wait_for_menu(reader, ports, port, serial_module):
    reader.ser.reset_input_buffer()
    reader.pending.clear()
    for attempt in range(2):
        if attempt:
            print('Romi is still at the REPL. Retrying the soft reboot once.')
        confirm_repl(reader)
        print('REPL confirmed. Sending Ctrl-D to start the saved firmware.')
        reader.ser.write(b'\x04')
        build = wait_for_boot(reader, ports, port, serial_module)
        if build is not None:
            return build
    raise RuntimeError('Romi stayed at the REPL after two reset attempts. '
                       'Start it in PuTTY and try --attach; see serial.log.')


def confirm_repl(reader):
    # ctrl-c clears any text typed at the prompt, including an old stop command
    for _ in range(2):
        reader.ser.write(b'\x03')
        try:
            wait_for_repl(reader, 4)
            nonce = str(monotonic_ns())
            marker = 'LAB09 REPL ' + nonce
            # the echo contains two strings; only the evaluated line is the marker
            reader.ser.write(("print('LAB09 REPL ' + '{}')\r".format(nonce)).encode('ascii'))
            end = monotonic() + 4
            while monotonic() < end:
                if reader.read_line(max(.01, end - monotonic())) == marker:
                    return
        except TimeoutError:
            pass
    raise TimeoutError('Could not confirm the MicroPython REPL; see serial.log')


def wait_for_boot(reader, ports, port, serial_module):
    end = monotonic() + 20
    last_prompt = None
    probe_at = monotonic() + 3
    probed = False
    traceback_lines = []
    while monotonic() < end:
        try:
            line = reader.read_line(.5)
        except TimeoutError:
            if last_prompt is not None and monotonic() - last_prompt >= 2:
                if traceback_lines:
                    raise RuntimeError('Firmware startup traceback:\n' + '\n'.join(traceback_lines))
                return None
            if last_prompt is None and not probed and monotonic() >= probe_at:
                # h only requests the ready marker when the Lab 09 task is idle
                reader.ser.write(b'h')
                probed = True
            continue
        except (serial_module.SerialException, OSError):
            reopen_port(reader, ports, port, serial_module)
            last_prompt = None
            probe_at = monotonic() + 1
            probed = False
            continue
        if line.startswith('# LAB09 READY '):
            return line.split()[-1]
        if line.startswith('Traceback'):
            traceback_lines = []
        if line.startswith('Traceback') or traceback_lines:
            traceback_lines.append(line)
        if re.match(r'^\w+(?:Error|Exception)(?::|$)', line):
            raise RuntimeError('Firmware startup failed: ' + line)
        if line.startswith('>>>'):
            # a delayed prompt alone does not mean the firmware exited
            last_prompt = monotonic()
            continue
        last_prompt = None
        if 'Enter command (' in line:
            raise RuntimeError('Old UI is running. Install the Step 3 observer firmware first.')
    detail = '\n' + '\n'.join(traceback_lines) if traceback_lines else ''
    raise TimeoutError('No Lab 09 ready marker after Ctrl-D; see serial.log.' + detail)


def reopen_port(reader, ports, port, serial_module):
    reader.ser.close()
    end = monotonic() + 10
    while True:
        try:
            reader.ser = ports.enter_context(serial_module.Serial(
                port, BAUD, timeout=.2, write_timeout=2))
            reader.pending.clear()
            return
        except (serial_module.SerialException, OSError):
            if monotonic() >= end:
                raise
            sleep(.25)


def attach_to_menu(reader):
    reader.ser.reset_input_buffer()
    print('Checking for the running Lab 09 firmware without rebooting.')
    reader.ser.write(b'h')
    try:
        return wait_for(reader, '# LAB09 READY ', 5).split()[-1]
    except TimeoutError:
        raise TimeoutError('No Lab 09 ready reply. In PuTTY, start the firmware and '
                           'wait for # LAB09 READY, then close PuTTY and rerun --attach.') from None


def set_trial(reader, config):
    # one-character commands have no ending; the multi-character value uses CR
    reader.ser.write(b'd')
    wait_for(reader, '# LAB09 CONFIGURE')
    reader.ser.write((json.dumps(config, separators=(',', ':')) + '\r').encode('ascii'))
    line = wait_for(reader, '# LAB09 CONFIG ')
    accepted = json.loads(line[len('# LAB09 CONFIG '):])
    if accepted != config:
        raise ValueError('Romi accepted different settings')


def collect(reader, config, expected_reason="complete"):
    wait_for(reader, '# LAB09 BEGIN', config['duration_ms'] / 1000 + 15)
    metadata = None
    rows = []
    header_seen = False
    deadline = monotonic() + 30
    while monotonic() < deadline:
        line = reader.read_line(max(.01, deadline - monotonic()))
        if line.startswith('# LAB09 END '):
            reason = line[len('# LAB09 END '):]
            break
        if line.startswith('# {'):
            if metadata is not None:
                raise ValueError('duplicate metadata')
            metadata = json.loads(line[2:])
        elif line == ','.join(COLUMNS):
            if header_seen:
                raise ValueError('duplicate header')
            header_seen = True
        elif header_seen and not line.startswith('#'):
            fields = line.split(',')
            if len(fields) != len(COLUMNS):
                raise ValueError('malformed data row: ' + line)
            values = tuple(float(x) for x in fields)
            if not all(math.isfinite(x) for x in values):
                raise ValueError('nonfinite response data')
            rows.append(values)
        elif line.startswith(('# LAB09 ERROR', 'Traceback')):
            raise RuntimeError(line)
    else:
        raise TimeoutError('missing end-of-dataset marker')
    if not header_seen or metadata is None:
        raise ValueError('missing header or metadata')
    if reason != expected_reason or metadata.get('stop_reason') != expected_reason:
        raise RuntimeError('MCU stopped the test: ' + reason + '; partial output is in serial.log')
    for key, expected in config.items():
        if metadata.get(key) != expected:
            raise ValueError('dataset setting mismatch: ' + key)
    if metadata.get('samples') != len(rows) or len(rows) < 10:
        raise ValueError('incomplete sample count')
    if metadata.get('motor_period_ms') != 10 or metadata.get('controller') != 'P':
        raise ValueError('unexpected baseline controller or period')
    if metadata.get('controller_period_ms') != 10:
        raise ValueError('controller period changed; comparisons require 10 ms')
    if metadata.get('observer_enabled') is not True:
        raise ValueError('observer is not confirmed running on both motors')
    if metadata.get('controller_feedback') != 'observer':
        raise ValueError('Step 5 requires observer velocity as P feedback')
    if metadata.get('observer_period_ms') != 10:
        raise ValueError('observer period does not match the 10 ms design')
    if (metadata.get('feedback') != 'encoder finite difference' or
            metadata.get('velocity_window') != 'one encoder update; measured dt' or
            metadata.get('counts_per_rev') != 1440):
        raise ValueError('original encoder velocity no longer matches Step 1')
    if metadata.get('line_correction') is not False or metadata.get('heading_correction') is not False:
        raise ValueError('steering correction must be disabled')
    if rows[0][0] < 0 or any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise ValueError('timestamps are not increasing')
    refs = [r[1] for r in rows]
    if refs[0] != 0 or not any(math.isclose(x, config['speed'], abs_tol=1e-5) for x in refs):
        raise ValueError('rest/step segment missing')
    first_step = next(i for i, x in enumerate(refs) if x != 0)
    if first_step < 2 or any(not math.isclose(x, config['speed'], abs_tol=1e-5) for x in refs[first_step:]):
        raise ValueError('invalid step setpoint')
    if any(not config['vmin'] - 1e-5 <= r[4] <= config['vmax'] + 1e-5 for r in rows):
        raise ValueError('voltage exceeds configured limits')
    total = (config['rest_ms'] + config['duration_ms']) / 1000
    if expected_reason == 'complete' and (rows[-1][0] < total - .1 or rows[-1][0] > total + .1):
        raise ValueError('recording duration differs from the finite test')
    metadata['max_sample_gap_s'] = max(b[0] - a[0] for a, b in zip(rows, rows[1:]))
    return metadata, rows


def save(folder, metadata, rows, conditions, needs_review=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    name = 'step5_observer_feedback_{}_{:g}rad_s'.format(
        metadata['motor'], metadata['speed'])
    if needs_review:
        name += '_review'
    path = folder / (name + '.csv')
    with path.open('x', newline='', encoding='utf-8') as out:
        out.write('# ' + json.dumps({'firmware': metadata, 'conditions': conditions}) + '\n')
        writer = csv.writer(out)
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    fig, (ax, voltage_ax) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    t, ref, velocity, estimate, voltage = zip(*rows)
    ax.step(t, ref, where='post', color='0.45', linestyle='--', label='setpoint')
    ax.plot(t, velocity, color='black', linewidth=.9, label='encoder velocity (recorded)')
    ax.plot(t, estimate, color='0.3', linewidth=1.2,
            linestyle='-.', label='observer estimate (P feedback)')
    ax.set(ylabel='Wheel velocity (rad/s)',
           title='Lab 9 Step 5 | observer P feedback | {} | {:g} rad/s'.format(
               metadata['motor'], metadata['speed']))
    ax.legend()
    voltage_ax.step(t, voltage, where='post', color='black', linewidth=.9, label='limited voltage')
    voltage_ax.axhline(metadata['vmax'], color='.6', linestyle=':', label='voltage limits')
    voltage_ax.axhline(metadata['vmin'], color='.6', linestyle=':')
    voltage_ax.set(xlabel='Time from capture start (s)', ylabel='Motor command (V)')
    voltage_ax.legend()
    for a in (ax, voltage_ax):
        a.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(folder / (name + '.png'), dpi=180)
    plt.close(fig)
    print('Saved', len(rows), 'rows and plot:', name)


def load_step4_reference(speed):
    name = 'step4_original_feedback_left_{:g}rad_s.csv'.format(speed)
    path = SCRIPT_FOLDER / 'step4_reference' / name
    with path.open(encoding='utf-8', newline='') as reference:
        saved = json.loads(reference.readline()[2:])
        reader = csv.reader(reference)
        if next(reader) != COLUMNS:
            raise ValueError('Unexpected Step 4 CSV columns: ' + name)
        rows = [tuple(float(x) for x in line) for line in reader]
    meta = saved['firmware']
    conditions = saved['conditions']
    if (meta['build'] != STEP4_BUILD or meta['controller_feedback'] != 'original' or
            meta['mode'] != 'loaded' or meta['stop_reason'] != 'complete' or
            meta['motor'] != 'left' or meta['speed'] != speed or
            len(rows) != meta['samples'] or
            conditions['surface'] != SURFACE or conditions['loading'] != LOADING):
        raise ValueError('Step 4 reference does not match the saved checkpoint: ' + name)
    if (not all(len(r) == len(COLUMNS) and all(math.isfinite(x) for x in r) for r in rows)
            or any(b[0] <= a[0] for a, b in zip(rows, rows[1:]))
            or not rows or rows[0][1] != 0 or not any(r[1] == speed for r in rows)):
        raise ValueError('Invalid Step 4 samples: ' + name)
    return meta, rows, path


def save_comparison(folder, reference, metadata, rows):
    import matplotlib.pyplot as plt

    before, old_rows, source = reference
    matched = ('motor', 'speed', 'kp_left', 'kp_right', 'rest_ms', 'duration_ms',
               'controller', 'vmin', 'vmax', 'motor_period_ms', 'controller_period_ms',
               'observer_period_ms', 'observer_design', 'feedback', 'counts_per_rev',
               'velocity_window', 'line_correction', 'heading_correction')
    for key in matched:
        if before[key] != metadata[key]:
            raise ValueError('Step 4/5 comparison setting differs: ' + key)

    def trace(data):
        # Align the first logged nonzero setpoint in each capture.
        origin = next(row[0] for row in data if row[1] != 0)
        return ([row[0] - origin for row in data], *zip(*(row[1:] for row in data)))

    ta, ra, ea, oa, va = trace(old_rows)
    tb, rb, eb, ob, vb = trace(rows)
    figure, (before_ax, after_ax, voltage_ax) = plt.subplots(
        3, 1, figsize=(9, 8), sharex=True)
    figure.suptitle('Lab 9 | left wheel | {:g} rad/s | Step 4 vs Step 5'.format(metadata['speed']))
    for ax, t, ref, encoder, estimate, label in (
            (before_ax, ta, ra, ea, oa, 'Step 4: encoder velocity controls P'),
            (after_ax, tb, rb, eb, ob, 'Step 5: observer velocity controls P')):
        ax.step(t, ref, where='post', color='0.5', linestyle='--', label='setpoint')
        ax.plot(t, encoder, color='black', linewidth=.85, label='encoder velocity')
        ax.plot(t, estimate, color='.35', linestyle='-.', label='observer estimate')
        ax.set(ylabel='Velocity (rad/s)', title=label)
        ax.legend(loc='lower right')
    # Use one velocity scale so changes in tracking are visible at a glance.
    ymin = min(before_ax.get_ylim()[0], after_ax.get_ylim()[0])
    ymax = max(before_ax.get_ylim()[1], after_ax.get_ylim()[1])
    before_ax.set_ylim(ymin, ymax)
    after_ax.set_ylim(ymin, ymax)
    voltage_ax.step(ta, va, where='post', color='.5', linestyle='--', label='Step 4')
    voltage_ax.step(tb, vb, where='post', color='black', linewidth=1, label='Step 5')
    voltage_ax.axhline(metadata['vmax'], color='.65', linestyle=':', label='voltage limits')
    voltage_ax.axhline(metadata['vmin'], color='.65', linestyle=':')
    voltage_ax.set(xlabel='Time from recorded velocity step (s)', ylabel='Motor command (V)')
    voltage_ax.legend(loc='lower right')
    for ax in (before_ax, after_ax, voltage_ax):
        ax.grid(alpha=.25)
    if before['battery'] != metadata['battery']:
        voltage_ax.set_title('Battery references: Step 4 {:g} V; Step 5 {:g} V'.format(
            before['battery'], metadata['battery']))
    figure.tight_layout()
    name = 'step5_vs_step4_left_{:g}rad_s.png'.format(metadata['speed'])
    figure.savefig(folder / name, dpi=180)
    plt.close(figure)

    def late_metrics(data, meta):
        tail = [r for r in data if r[1] != 0 and r[0] >= data[-1][0] - .5]
        encoder = [r[2] for r in tail]
        volts = [r[4] for r in tail]
        return {'mean_encoder_rad_s': mean(encoder),
                'encoder_std_rad_s': pstdev(encoder),
                'setpoint_minus_mean_encoder_rad_s': meta['speed'] - mean(encoder),
                'mean_voltage_V': mean(volts), 'voltage_std_V': pstdev(volts),
                'battery_reference_V': meta['battery']}

    a, b = late_metrics(old_rows, before), late_metrics(rows, metadata)
    print('Step 4 -> 5, last 0.5 s: mean encoder {:.2f} -> {:.2f} rad/s; '
          'voltage variation {:.2f} -> {:.2f} V (std dev).'.format(
              a['mean_encoder_rad_s'], b['mean_encoder_rad_s'],
              a['voltage_std_V'], b['voltage_std_V']))
    print('Saved comparison:', name)
    return {'plot': name, 'reference_file': source.name,
            'reference_build': before['build'],
            'step4_original_feedback': a, 'step5_observer_feedback': b}


def snapshot(source, folder):
    source = Path(source)
    build = json.loads((source / 'lab09_build.json').read_text())['build']
    hashes = {}
    with zipfile.ZipFile(folder / 'firmware_checkpoint.zip', 'x', zipfile.ZIP_DEFLATED) as out:
        for p in sorted(source.iterdir()):
            if p.is_file() and p.suffix in ('.py', '.mpy', '.json', '.txt'):
                out.write(p, p.name)
                hashes[p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    if 'cotask.py' not in hashes:
        print('NOTE: cotask.py is missing from the uploaded source archive.')
        print('Copy the board\'s working cotask.py into', source,
              'to complete the source checkpoint before final submission.')
    return build, hashes


def read_profile(reader):
    """Collect the course scheduler profile while both wheels are stopped."""

    reader.ser.write(b'p')
    wait_for(reader, '# LAB09 PROFILE BEGIN')
    lines = []
    deadline = monotonic() + 15
    while monotonic() < deadline:
        line = reader.read_line(max(.01, deadline - monotonic()))
        if line == '# LAB09 PROFILE END':
            return lines
        lines.append(line)
    raise TimeoutError('Romi did not finish the scheduler profile')


def check_trial_timing(metadata, rows):
    """Check actual sample intervals and work, without adding unrelated maxima."""

    expected = round((metadata['rest_ms'] + metadata['duration_ms']) * 1000
                     / MOTOR_PERIOD_US)
    issues = []
    gc_info = metadata.get('scheduled_gc')
    if gc_info is not None:
        print('Scheduled cleanup: {} runs; max pause {:.3f} ms'.format(
            gc_info['runs'], gc_info['max_pause_us'] / 1000))
    for name in ('left', 'right'):
        timing = metadata['timing_' + name]
        samples = timing['samples']
        mean_gap_us = timing['mean_gap_us']
        max_gap_us = timing['max_gap_us']
        max_exec_us = timing['max_exec_us']
        print('{}: {} samples; mean interval {:.3f} ms; max interval {:.3f} ms; '
              'max work {:.3f} ms'.format(name.capitalize(), samples,
                  mean_gap_us / 1000, max_gap_us / 1000, max_exec_us / 1000))
        if (abs(samples - expected) > 2 or
                not 9000 <= mean_gap_us <= 11000 or
                not mean_gap_us <= max_gap_us < MAX_SAMPLE_GAP_US):
            issues.append(name + ' motor sample count or interval is outside the 10 ms check')
        if not 0 <= max_exec_us < MOTOR_PERIOD_US:
            issues.append(name + ' motor reported work at or above one 10 ms period')

    # Timestamps also check the selected wheel independently of its summary.
    logged_max_us = round(1e6 * max(b[0] - a[0] for a, b in zip(rows, rows[1:])))
    if logged_max_us >= MAX_SAMPLE_GAP_US:
        issues.append('Selected-wheel log has a sample gap of 15 ms or longer')
    return issues


def tracking_summary(metadata, rows):
    """Report useful checks for the Step 4 plot and subsequent comparisons."""

    rest = [row for row in rows if row[1] == 0]
    end_time = rows[-1][0]
    late = [row for row in rows if row[1] != 0 and row[0] >= end_time - .5]
    original_mean = sum(row[2] for row in late) / len(late)
    estimate_mean = sum(row[3] for row in late) / len(late)
    result = {
        'rest_original_peak_rad_s': max(abs(row[2]) for row in rest),
        'rest_estimate_peak_rad_s': max(abs(row[3]) for row in rest),
        'late_original_mean_rad_s': original_mean,
        'late_estimate_mean_rad_s': estimate_mean,
        'late_signs_match_setpoint': (
            metadata['speed'] * original_mean > 0 and
            metadata['speed'] * estimate_mean > 0
        ),
    }
    print('Rest peak original/estimated: {:.2f}/{:.2f} rad/s'.format(
        result['rest_original_peak_rad_s'], result['rest_estimate_peak_rad_s']))
    print('Last 0.5 s mean original/estimated: {:.2f}/{:.2f} rad/s'.format(
        original_mean, estimate_mean))
    return result


def confirmed(prompt):
    return input(prompt + ' [y/yes]: ').strip().lower() in ('y', 'yes')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default=PORT)
    p.add_argument('--firmware', default=str(SCRIPT_FOLDER / 'firmware'),
                   help='local source checkpoint; defaults to firmware/ next to this script')
    p.add_argument('--motor', choices=('left', 'right'), default='left')
    p.add_argument('--low', type=float, default=2)
    p.add_argument('--high', type=float, default=12)
    p.add_argument('--kp-left', type=float, default=2, help='saved Step 1 left P gain')
    p.add_argument('--kp-right', type=float, default=2, help='saved Step 1 right P gain')
    p.add_argument('--battery', type=float, default=MEASURED_BATTERY_V,
                   help='measured battery reference in volts; default is the supplied 7.65 V')
    p.add_argument('--vmin', type=float, default=-6)
    p.add_argument('--vmax', type=float, default=6)
    p.add_argument('--duration', type=float, default=1.5, help='step duration in seconds, excluding 0.5 s rest')
    p.add_argument('--bench-only', action='store_true',
                   help='keep both wheels raised; check signs, abort and timing, then exit')
    p.add_argument('--attach', action='store_true',
                   help='connect to the running Lab 09 firmware without resetting it')
    a = p.parse_args()
    values = (a.low, a.high, a.kp_left, a.kp_right, a.battery, a.vmin, a.vmax, a.duration)
    if not all(math.isfinite(x) for x in values):
        p.error('settings must be finite')
    if not 1 <= a.low <= 2 or not 10 <= a.high <= 15:
        p.error('low must be 1-2 rad/s and high 10-15 rad/s')
    if not .5 <= a.duration <= 2.5 or not 0 < a.battery <= 12:
        p.error('duration must be 0.5-2.5 s and battery 0-12 V')
    if not -a.battery <= a.vmin < 0 < a.vmax <= a.battery:
        p.error('voltage limits must fit the battery reference')
    if not all(0 <= k <= 20 for k in (a.kp_left, a.kp_right)):
        p.error('Kp must be between 0 and 20')
    if (a.motor != 'left' or (a.low, a.high) != (2, 12) or
            (a.kp_left, a.kp_right) != (2, 2) or
            (a.vmin, a.vmax) != (-6, 6) or a.duration != 1.5):
        p.error('Step 5 must match Step 4: left wheel, 2/12 rad/s, '
                'Kp 2/2, +/-6 V, 0.5 s rest + 1.5 s motion')
    source = Path(a.firmware).expanduser().resolve()
    build_path = source / 'lab09_build.json'
    if not build_path.is_file():
        p.error('Cannot find {}. Keep the firmware folder from the Step 5 package '
                'next to this script, or pass --firmware with the path to it.'
                .format(build_path))
    if json.loads(build_path.read_text())['build'] != STEP5_BUILD:
        p.error('Use the Step 5 source checkpoint shipped with this script.')
    a.firmware = str(source)
    import serial
    import matplotlib  # find missing dependencies before connecting to the robot
    references = {} if a.bench_only else {
        speed: load_step4_reference(speed) for speed in (a.low, a.high)}
    run_root = 'romi_lab09_step5_bench' if a.bench_only else 'romi_lab09_step5_runs'
    folder = SCRIPT_FOLDER / run_root / datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    build, hashes = snapshot(a.firmware, folder)
    print('Step 5 bench checks' if a.bench_only else 'Step 5 bench and track collection')
    print('Each P loop uses its own observer velocity; encoder velocity is still recorded.')
    print('Expected 10 ms motor/observer period, 1440 ticks/rev, and +/-6 V limits.')
    print('Saved baseline P gains: left={:g}, right={:g}.'.format(
        a.kp_left, a.kp_right))
    print('Battery reference: {:g} V.'.format(a.battery))
    print('Both motors will receive equal FORWARD setpoints; line and IMU steering stay off.')
    print('Each test has 0.5 s at rest followed by', a.duration, 's commanded motion.')
    conditions = {
        'run_type': 'step5_bench' if a.bench_only else 'step5_observer_feedback',
        'surface': SURFACE,
        'loading': LOADING,
        'battery_condition': 'meter reading supplied before tests; not monitored while moving',
        'battery_measured_V': a.battery,
        'teammates': TEAMMATES,
        'teammate_confirmations': 'startup procedure recorded in Step 1 baseline',
        'controller_tuning': 'observer velocity feedback, original P law and Kp=2 per wheel',
    }
    print('Test conditions:', SURFACE, '|', LOADING)
    print('If battery voltage or test conditions have changed, update the settings before driving.')
    run_info = {'build': build, 'arguments': vars(a), 'conditions': conditions,
                'source_sha256': hashes, 'missing_source_files':
                [] if 'cotask.py' in hashes else ['cotask.py'],
                'complete': False, 'completed_speeds': [], 'tracking_runs': []}
    if references:
        run_info['step4_reference_sha256'] = {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for _, _, path in references.values()}
    config_path = folder / ('step5_bench_config.json' if a.bench_only else 'step5_config.json')
    config_path.write_text(json.dumps(run_info, indent=2))
    with (folder / 'serial.log').open('x', encoding='utf-8') as log, ExitStack() as ports:
        reader = None
        app_ready = False
        try:
            print('Opening', a.port, 'at', BAUD, 'baud')
            ser = ports.enter_context(serial.Serial(a.port, BAUD, timeout=.2, write_timeout=2))
            reader = Reader(ser, log)
            found = (attach_to_menu(reader) if a.attach else
                     restart_and_wait_for_menu(reader, ports, a.port, serial))
            if found != build:
                raise ValueError('Romi build {} differs from local checkpoint {}'.format(found, build))
            app_ready = True
            print('\nTEST 1 - STARTUP AND DIRECTION (wheels raised)')
            if not confirmed('Keep BOTH wheels raised. Did both stay still at startup?'):
                raise RuntimeError('Physical startup check failed')
            run_info['startup_check'] = 'operator-confirmed'
            checks = []
            for motor, speed in (('left', 5), ('left', -5), ('right', 5), ('right', -5)):
                c = dict(build=build, motor=motor, speed=speed, mode='check',
                         kp_left=a.kp_left, kp_right=a.kp_right, vmin=a.vmin,
                         vmax=a.vmax, battery=a.battery, duration_ms=700, rest_ms=500)
                set_trial(reader, c)
                input('Raised check: {} at {:+g} rad/s. Press Return: '.format(motor, speed))
                reader.ser.write(b'b')
                meta, check_rows = collect(reader, c)
                moving = [row for row in check_rows if row[1] != 0]
                settled = moving[len(moving) // 2:]
                original_mean = sum(row[2] for row in settled) / len(settled)
                estimate_mean = sum(row[3] for row in settled) / len(settled)
                if original_mean * speed <= 0:
                    raise RuntimeError('Raised check: encoder sign or motion failed for ' + motor)
                if estimate_mean * speed <= 0:
                    raise RuntimeError('Raised check: observer velocity sign failed for ' + motor)
                print('Late-run original/estimated mean: {:.2f}/{:.2f} rad/s'.format(
                    original_mean, estimate_mean))
                if not confirmed('Did ONLY that wheel turn {} and then stop?'.format(
                        'forward' if speed > 0 else 'backward')):
                    raise RuntimeError('Physical direction/stop check failed')
                checks.append({'motor': motor, 'speed': speed, 'result': 'operator and both signals confirmed',
                               'original_mean_rad_s': original_mean, 'estimated_mean_rad_s': estimate_mean})
            # exercise the same stop command used on a PC abort while still raised
            print('\nTEST 2 - OPERATOR STOP (wheels raised)')
            c['speed'] = 5
            c['duration_ms'] = 1500
            set_trial(reader, c)
            input('Raised stop-command check on right wheel. Press Return: ')
            reader.ser.write(b'b')
            sleep(.85)
            reader.ser.write(b's')
            collect(reader, c, expected_reason='operator_abort')
            if not confirmed('Did s stop the wheel promptly?'):
                raise RuntimeError('Operator stop check failed')
            run_info['raised_checks'] = checks
            run_info['operator_stop_check'] = 'operator-confirmed'
            print('\nTEST 3 - BOTH MOTORS AND TIMING (wheels raised)')
            c.update(motor='left', speed=5, mode='both')
            set_trial(reader, c)
            input('Keep BOTH wheels raised and still. Press Return for the timing check: ')
            reader.ser.write(b'b')
            metadata, rows = collect(reader, c)
            if not confirmed('Did BOTH wheels turn forward and then stop without sustained surging?'):
                raise RuntimeError('Both-wheel bench check failed')
            run_info['combined_timing'] = {
                'left': metadata['timing_left'], 'right': metadata['timing_right']}
            if 'scheduled_gc' in metadata:
                run_info['scheduled_gc'] = metadata['scheduled_gc']
            issues = check_trial_timing(metadata, rows)
            run_info['bench_timing_issues'] = issues
            config_path.write_text(json.dumps(run_info, indent=2))
            if issues:
                raise RuntimeError('; '.join(issues))
            if a.bench_only:
                profile = read_profile(reader)
                (folder / 'scheduler_profile.txt').write_text('\n'.join(profile) + '\n')
                if not profile or any(line.startswith('# LAB09 PROFILE UNAVAILABLE') for line in profile):
                    raise RuntimeError('Course scheduler profile unavailable; see serial.log')
                reader.ser.write(b'e')
                wait_for(reader, '# LAB09 EXIT')
                run_info['complete'] = True
                config_path.write_text(json.dumps(run_info, indent=2))
                print('Bench collection complete:', folder)
                print('Review scheduler_profile.txt and confirm 10 ms before driving on the track.')
                return

            for number, speed in enumerate((a.low, a.high), start=4):
                print('\nTEST {} - LOADED RESPONSE: {:g} rad/s'.format(number, speed))
                config = dict(build=build, motor=a.motor, speed=speed, mode='loaded',
                              kp_left=a.kp_left, kp_right=a.kp_right,
                              vmin=a.vmin, vmax=a.vmax, battery=a.battery,
                              duration_ms=round(a.duration * 1000), rest_ms=500)
                set_trial(reader, config)
                input('Place Romi AT REST on the lab track, clear straight path, cable slack. Press Return for {:g} rad/s: '.format(speed))
                reader.ser.write(b'b')
                metadata, rows = collect(reader, config)
                issues = check_trial_timing(metadata, rows)
                summary = tracking_summary(metadata, rows)
                if not summary['late_signs_match_setpoint']:
                    issues.append('Late encoder and observer velocities do not both agree with the forward command')
                run_info['tracking_runs'].append({
                    'setpoint_rad_s': speed, 'timing_left': metadata['timing_left'],
                    'timing_right': metadata['timing_right'], 'summary': summary,
                    'review_issues': issues,
                })
                if 'scheduled_gc' in metadata:
                    run_info['tracking_runs'][-1]['scheduled_gc'] = metadata['scheduled_gc']
                save(folder, metadata, rows, conditions, needs_review=bool(issues))
                if issues:
                    raise RuntimeError('Trial saved for review: ' + '; '.join(issues))
                run_info['tracking_runs'][-1]['comparison'] = save_comparison(
                    folder, references[speed], metadata, rows)
                run_info['completed_speeds'].append(speed)
                config_path.write_text(json.dumps(run_info, indent=2))
            profile = read_profile(reader)
            (folder / 'scheduler_profile.txt').write_text('\n'.join(profile) + '\n')
            if not profile or any(line.startswith('# LAB09 PROFILE UNAVAILABLE') for line in profile):
                raise RuntimeError('Course scheduler profile unavailable; see serial.log')
            reader.ser.write(b'e')
            wait_for(reader, '# LAB09 EXIT')
            run_info['complete'] = True
            config_path.write_text(json.dumps(run_info, indent=2))
            print('Collection complete:', folder)
            print('Retain these setpoints, gains, conditions and checkpoint for the later comparisons.')
        except BaseException as error:
            if app_ready:
                try:
                    reader.ser.write(b's\x03')
                except (serial.SerialException, OSError):
                    pass
            run_info['error'] = str(error)
            config_path.write_text(json.dumps(run_info, indent=2))
            print('Stopped. Completed trials remain saved; see', folder / 'serial.log')
            raise


if __name__ == '__main__':
    main()
