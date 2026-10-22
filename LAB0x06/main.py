from pyb import ADC, Pin, Timer, USB_VCP, delay
from time import ticks_ms, ticks_diff

from MotorDriver import MotorDriver
from EncoderDriver import EncoderDriver
from taskmotor import MotorData, TaskMotor
from taskuser import TaskUser
from line_sensor import LineSensor, CHANNELS
from line_calibration import save_calibration, load_calibration
from taskline import TaskLine
import cotask


# the eight sensor pins, ordered from printed sensor 1 through sensor 8
SENSOR_PINS = (
    Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.C2, Pin.cpu.C3,
    Pin.cpu.A4, Pin.cpu.C4, Pin.cpu.B0, Pin.cpu.B1,
)
CALIBRATION_FILE = "line_calibration.json"
MIN_CALIBRATION_SPAN = 10


class SystemData:
    def __init__(self):
        self.exit_requested = False


def wait_for_key(vcp, key, message):
    # repeat the prompt so it is visible even if PuTTY opens after startup
    last_prompt = None
    while True:
        now = ticks_ms()
        if last_prompt is None or ticks_diff(now, last_prompt) >= 3000:
            vcp.write((message + "\r\n").encode())
            last_prompt = now
        if vcp.any():
            incoming = vcp.read(1)
            if incoming in (key, key.upper()):
                return
        delay(20)


def capture_average(adcs):
    # average the same eight ADC channels that LineSensor will use
    averages = []
    for adc in adcs:
        total = 0
        for _ in range(50):
            total += adc.read()
        averages.append(total / 50)
    return averages


def get_calibration():
    try:
        calibration = load_calibration(
            CALIBRATION_FILE, CHANNELS, MIN_CALIBRATION_SPAN
        )
        print("Loaded eight-sensor calibration from", CALIBRATION_FILE)
        return calibration
    except (OSError, ValueError, KeyError, TypeError) as error:
        print("No valid calibration file:", error)

    # motors were disabled before this function was called
    vcp = USB_VCP()
    adcs = [ADC(pin) for pin in SENSOR_PINS]

    wait_for_key(vcp, b"b", "Place ALL eight sensors over black; press b.")
    delay(300)
    black = capture_average(adcs)
    print("Black ADC:", black)

    wait_for_key(vcp, b"w", "Place ALL eight sensors over white; press w.")
    delay(300)
    white = capture_average(adcs)
    print("White ADC:", white)

    calibration = {"channels": list(CHANNELS), "black": black, "white": white}
    save_calibration(
        CALIBRATION_FILE, calibration, CHANNELS, MIN_CALIBRATION_SPAN
    )
    print("Saved eight-sensor calibration to", CALIBRATION_FILE)
    return calibration


def main():
    pwm_tim = Timer(2, freq=20_000)
    left_motor = MotorDriver(Pin.cpu.A0, Pin.cpu.A6, Pin.cpu.C6, pwm_tim, 1)
    right_motor = MotorDriver(Pin.cpu.A1, Pin.cpu.A7, Pin.cpu.C5, pwm_tim, 2)
    left_enc = EncoderDriver(1, Pin.cpu.A8, Pin.cpu.A9)
    right_enc = EncoderDriver(3, Pin.cpu.B4, Pin.cpu.B5)

    left_motor.set_effort(0)
    left_motor.disable()
    right_motor.set_effort(0)
    right_motor.disable()

    calibration = get_calibration()
    sensor = LineSensor(
        SENSOR_PINS, calibration["black"], calibration["white"],
        min_span=MIN_CALIBRATION_SPAN, samples=2,
    )

    left_data = MotorData()
    right_data = MotorData()
    system = SystemData()

    # the fourth TaskMotor argument is present in the uploaded taskmotor.py;
    # that file creates its own controller and does not use the argument
    left_task = TaskMotor(left_motor, left_enc, left_data, None)
    right_task = TaskMotor(right_motor, right_enc, right_data, None)
    line_task = TaskLine(sensor, left_data, right_data, config={})
    user_task = TaskUser(USB_VCP(), left_data, right_data, system, line_task)

    # starting line gain only; tune after the motor task can follow setpoints
    user_task.kp_line = 0.02
    user_task.max_correction = 0.5

    cotask.task_list.append(cotask.Task(
        left_task.taskmotor_gen_fcn(), name="Left_Motor",
        priority=5, period=10, profile=True,
    ))
    cotask.task_list.append(cotask.Task(
        right_task.taskmotor_gen_fcn(), name="Right_Motor",
        priority=5, period=10, profile=True,
    ))
    cotask.task_list.append(cotask.Task(
        user_task.taskuser_gen_fcn(), name="User",
        priority=1, period=20, profile=True,
    ))
    cotask.task_list.append(cotask.Task(
        line_task.taskline_gen_fcn(), name="Line",
        priority=2, period=20, profile=True,
    ))

    try:
        while not system.exit_requested:
            cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        print("Stopped by Ctrl-C")
    finally:
        line_task.stop()
        left_task.stop()
        right_task.stop()


if __name__ == "__main__":
    main()
