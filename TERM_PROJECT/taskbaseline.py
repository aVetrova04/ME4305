"""Finite Lab 09 trials and serial output for the motor tasks."""
from array import array
from time import ticks_us, ticks_diff, ticks_add
import json
import math
import gc

from observer import OBSERVER_PERIOD_MS
from taskmotor import MOTOR_PERIOD_MS, TICKS_PER_WHEEL_REVOLUTION

BUILD_ID = "lab09-step3-10ms-a"
COLUMNS = (
    "time_s,setpoint_rad_s,feedback_rad_s,estimated_rad_s,limited_voltage_V"
)


class Lab09UI:
    def __init__(self, vcp, left_data, right_data, system,
                 left_task, right_task, scheduler_profile):
        self.vcp = vcp
        self.data = (left_data, right_data)
        self.tasks = (left_task, right_task)
        self.system = system
        self.scheduler_profile = scheduler_profile
        self.config = None
        self.state = "init"
        self.buffer = bytearray()
        self.count = 0
        self.index = 0
        self.reason = "idle"
        self.times = array('L', [0] * 320)
        self.refs = array('f', [0] * 320)
        self.speeds = array('f', [0] * 320)
        self.estimates = array('f', [0] * 320)
        self.volts = array('f', [0] * 320)
        self.stop()

    def write(self, text):
        self.vcp.write((text + "\r\n").encode())

    def stop(self):
        for task in self.tasks:
            task.stop()

    def configure(self, text):
        c = json.loads(text)
        for key in ("speed", "kp_left", "kp_right", "vmin", "vmax", "battery"):
            if not math.isfinite(float(c[key])):
                raise ValueError("nonfinite " + key)
        if not 0 < abs(c['speed']) <= 15 or c['motor'] not in ('left', 'right'):
            raise ValueError("speed or motor")
        if c.get('mode') not in ('loaded', 'check', 'both'):
            raise ValueError('mode')
        if c['mode'] == 'loaded' and c['speed'] <= 0:
            raise ValueError('loaded tests must be forward')
        if c['mode'] in ('check', 'both') and abs(c['speed']) > 5:
            raise ValueError('check speed')
        if not 0 <= c['kp_left'] <= 20 or not 0 <= c['kp_right'] <= 20:
            raise ValueError("gain")
        if not 0 < c['battery'] <= 12 or not -c['battery'] <= c['vmin'] < 0 < c['vmax'] <= c['battery']:
            raise ValueError("voltage limits")
        if not 500 <= c['duration_ms'] <= 2500 or c['rest_ms'] != 500:
            raise ValueError("duration or rest interval")
        if c.get('build') != BUILD_ID:
            raise ValueError("checkpoint does not match")
        self.stop()
        for i, (task, data) in enumerate(zip(self.tasks, self.data)):
            data.kp = c['kp_left'] if i == 0 else c['kp_right']
            data.v_min, data.v_max = c['vmin'], c['vmax']
            task.motor.battery_voltage = c['battery']
        self.config = c

    def begin(self):
        if self.config is None:
            raise ValueError("configure first")
        self.stop()
        for task in self.tasks:
            task.reset_run()
        gc.collect()
        self.count = 0
        self.index = 0
        self.reason = "running"
        self.start_us = ticks_us()
        # ticks_add handles the micropython clock rollover
        self.deadline = ticks_add(self.start_us, 1000 * (
            self.config['rest_ms'] + self.config['duration_ms']))
        selected = 0 if self.config['motor'] == 'left' else 1
        self.data[selected].capture_callback = self.record
        for data in self.data:
            data.run_deadline_us = self.deadline
        for task in self.tasks:
            task.start_timing()
        self.state = "rest"

    def record(self, now, data):
        if self.state not in ('rest', 'run'):
            return
        if self.count >= len(self.times):
            self.finish("buffer_full")
            return
        i = self.count
        self.times[i] = ticks_diff(now, self.start_us)
        self.refs[i] = data.velocity_setpoint
        self.speeds[i] = data.measured_speed_rad_s
        self.estimates[i] = data.estimated_speed_rad_s
        self.volts[i] = data.limited_voltage_V
        self.count += 1

    def finish(self, reason):
        self.stop()
        self.reason = reason
        self.state = "header"

    def report_profile(self):
        """Print the scheduler profile while both motors are stopped."""

        self.write("# LAB09 PROFILE BEGIN")
        try:
            if self.scheduler_profile is None:
                raise RuntimeError("Scheduler has no task_list.profile()")
            profile = self.scheduler_profile()
            if profile is not None:
                self.write(str(profile))
        except (TypeError, AttributeError, RuntimeError) as error:
            self.write("# LAB09 PROFILE UNAVAILABLE " + str(error))
        self.write("# LAB09 PROFILE END")

    def taskuser_gen_fcn(self):
        while True:
            now = ticks_us()
            if self.state == 'init':
                self.write('# LAB09 READY ' + BUILD_ID)
                self.state = 'idle'
            elif self.state in ('rest', 'run'):
                if ticks_diff(now, self.deadline) >= 0:
                    self.finish('complete')
                elif self.state == 'rest' and ticks_diff(now, self.start_us) >= 500000:
                    if any(abs(d.measured_speed_rad_s) > 0.3 for d in self.data):
                        self.finish('not_at_rest')
                    else:
                        for i, d in enumerate(self.data):
                            selected = i == (0 if self.config['motor'] == 'left' else 1)
                            d.velocity_setpoint = self.config['speed'] if (
                                self.config['mode'] in ('loaded', 'both') or selected
                            ) else 0
                        self.state = 'run'
            elif self.state == 'header':
                meta = dict(self.config)
                meta.update(
                    samples=self.count, stop_reason=self.reason, controller='P',
                    motor_period_ms=MOTOR_PERIOD_MS, controller_period_ms=MOTOR_PERIOD_MS,
                    observer_period_ms=OBSERVER_PERIOD_MS, observer_enabled=True,
                    controller_feedback='original', observer_design='A',
                    velocity_window='one encoder update; measured dt',
                    counts_per_rev=TICKS_PER_WHEEL_REVOLUTION,
                    feedback='encoder finite difference',
                    voltage='post-limit duty-equivalent volts',
                    heading_correction=False, line_correction=False,
                    timing_left=self.tasks[0].timing_summary(),
                    timing_right=self.tasks[1].timing_summary(),
                )
                self.write('# LAB09 BEGIN')
                self.write('# ' + json.dumps(meta))
                self.write(COLUMNS)
                self.state = 'print'
            elif self.state == 'print':
                if self.index < self.count:
                    i = self.index
                    self.write('{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}'.format(
                        self.times[i] / 1000000, self.refs[i], self.speeds[i],
                        self.estimates[i], self.volts[i]))
                    self.index += 1
                else:
                    self.write('# LAB09 END ' + self.reason)
                    self.state = 'idle'

            # bounded input work; nothing blocks while a wheel is running
            for _ in range(32):
                if not self.vcp.any():
                    break
                ch = self.vcp.read(1)
                if self.state == 'config':
                    if ch in (b'\r', b'\n'):
                        if not self.buffer:
                            continue
                        try:
                            self.configure(self.buffer.decode())
                            self.write('# LAB09 CONFIG ' + json.dumps(self.config))
                        except (ValueError, KeyError, TypeError) as error:
                            self.config = None
                            self.write('# LAB09 ERROR ' + str(error))
                        self.buffer = bytearray()
                        self.state = 'idle'
                    elif len(self.buffer) < 512:
                        self.buffer.extend(ch)
                    else:
                        self.buffer = bytearray()
                        self.config = None
                        self.state = 'idle'
                        self.write('# LAB09 ERROR config too long')
                    continue
                if not ch:
                    break
                ch = ch.lower()
                if ch == b's':
                    if self.state in ('rest', 'run'):
                        self.finish('operator_abort')
                    else:
                        self.stop()
                        self.write('# LAB09 STOPPED')
                elif ch == b'e':
                    self.stop()
                    self.system.exit_requested = True
                    self.write('# LAB09 EXIT')
                elif ch == b'h' and self.state == 'idle':
                    self.write('# LAB09 READY ' + BUILD_ID)
                elif ch == b'p' and self.state == 'idle':
                    self.report_profile()
                elif ch == b'd' and self.state == 'idle':
                    self.buffer = bytearray()
                    self.state = 'config'
                    self.write('# LAB09 CONFIGURE')
                elif ch == b'b' and self.state == 'idle':
                    try:
                        self.begin()
                    except (ValueError, MemoryError) as error:
                        self.stop()
                        self.state = 'idle'
                        self.write('# LAB09 ERROR ' + str(error))
            yield self.state
