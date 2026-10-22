from pyb import Pin, Timer, USB_VCP, ADC

from MotorDriver import MotorDriver
from EncoderDriver import EncoderDriver
from taskmotor import MotorData, TaskMotor
from taskuser import TaskUser
from ClosedLoop import ClosedLoop
import cotask

from line_sensor import LineSensor, CHANNELS
from line_calibration import save_calibration, load_calibration
from taskline import TaskLine

class SystemData:
    def __init__(self):
        self.exit_requested = False


def main():
    # both pwm channels use timer 2
    pwm_tim = Timer(2, freq=20_000)

    left_motor = MotorDriver(Pin.cpu.A0, Pin.cpu.A6, Pin.cpu.C6,
                             pwm_tim, 1)
    right_motor = MotorDriver(Pin.cpu.A1, Pin.cpu.A7, Pin.cpu.C5,
                              pwm_tim, 2)

    left_enc = EncoderDriver(1, Pin.cpu.A8, Pin.cpu.A9)
    right_enc = EncoderDriver(3, Pin.cpu.B4, Pin.cpu.B5)

    left_motor.set_effort(0)
    left_motor.disable()
    right_motor.set_effort(0)
    right_motor.disable()

    left_data = MotorData()
    right_data = MotorData()
    system = SystemData()

    left_task = TaskMotor(left_motor, left_enc, left_data)
    right_task = TaskMotor(right_motor, right_enc, right_data)
    user_task = TaskUser(USB_VCP(), left_data, right_data, system)

    # TODO: Replace with the eight verified ADC-capable pins.
    SENSOR_PINS = [
        Pin.cpu.PC0, Pin.cpu.PC1, Pin.cpu.PC2, Pin.cpu.PC3,
        Pin.cpu.PA4, Pin.cpu.PC4, Pin.cpu.PB0, Pin.cpu.PB1
    ]

    MIN_CALIBRATION_SPAN = 10

    sensor = LineSensor(
        SENSOR_PINS,
        calibration["black"],
        calibration["white"],
        min_span=MIN_CALIBRATION_SPAN,
        samples=2,
    )

    line_task = TaskLine(sensor, left_data, right_data, config={})

    # motor tasks run every 10 ms; the ui runs every 20 ms
    cotask.task_list.append(cotask.Task(
        left_task.taskmotor_gen_fcn(), name="Left_Motor",
        priority=5, period=10, profile=True
    ))
    cotask.task_list.append(cotask.Task(
        right_task.taskmotor_gen_fcn(), name="Right_Motor",
        priority=5, period=10, profile=True
    ))

    # lowest priority of ui
    cotask.task_list.append(cotask.Task(
        user_task.taskuser_gen_fcn(), name="User",
        priority=1, period=20, profile=True
    ))

    # mid-priority data collection
    cotask.task_list.append(cotask.Task(
        line_task.taskline_gen_fcn(), name="Line",
        priority=2, period=20, profile=True,
    ))

    try:
        while not system.exit_requested:
            cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        pass
    finally:
        # stop both motors on exit, ctrl-c, or an error
        left_task.stop()
        right_task.stop()


if __name__ == "__main__":
    main()
