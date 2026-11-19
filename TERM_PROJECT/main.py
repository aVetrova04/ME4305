from pyb import Pin, Timer, USB_VCP
import gc

from MotorDriver import MotorDriver
from EncoderDriver import EncoderDriver
from observer import Observer, OBSERVER_PERIOD_MS, verify_design
from taskmotor import MotorData, TaskMotor, MOTOR_PERIOD_MS, TICKS_PER_WHEEL_REVOLUTION
from taskbaseline import Lab09UI
import cotask


class SystemData:
    def __init__(self):
        self.exit_requested = False


def main():
    # Keep the drivers asleep through the one-step numerical check.
    Pin(Pin.cpu.C6, Pin.OUT_PP, value=0)
    Pin(Pin.cpu.C7, Pin.OUT_PP, value=0)
    if MOTOR_PERIOD_MS != OBSERVER_PERIOD_MS:
        raise ValueError("Motor task period and observer design must match")
    verify_design(TICKS_PER_WHEEL_REVOLUTION)
    gc.collect()

    pwm_tim = Timer(2, freq=20_000)
    left_motor = MotorDriver(Pin.cpu.A15, Pin.cpu.C10, Pin.cpu.C6,
                             pwm_tim, 1, polarity=-1)
    right_motor = MotorDriver(Pin.cpu.B3, Pin.cpu.C11, Pin.cpu.C7,
                              pwm_tim, 2, polarity=-1)
    left_motor.set_effort(0)
    right_motor.set_effort(0)
    left_motor.disable()
    right_motor.disable()

    # Same physical encoder pins as the supplied rewired checkpoint.
    left_enc = EncoderDriver(1, Pin.cpu.A8, Pin.cpu.A9, polarity=1)
    right_enc = EncoderDriver(3, Pin.cpu.B4, Pin.cpu.B5, polarity=-1)
    left_data = MotorData()
    right_data = MotorData()
    system = SystemData()
    left_observer = Observer(left_enc, TICKS_PER_WHEEL_REVOLUTION)
    right_observer = Observer(right_enc, TICKS_PER_WHEEL_REVOLUTION)
    left_task = TaskMotor(left_motor, left_enc, left_data, None, left_observer)
    right_task = TaskMotor(right_motor, right_enc, right_data, None, right_observer)
    left_task.reset_run()
    right_task.reset_run()

    # No line, IMU, or Dictator steering runs in this straight motor test.
    user_task = Lab09UI(USB_VCP(), left_data, right_data, system,
                        left_task, right_task, getattr(cotask.task_list, "profile", None))

    cotask.task_list.append(cotask.Task(
        left_task.taskmotor_gen_fcn(), name="Left_Motor",
        priority=5, period=MOTOR_PERIOD_MS, profile=True))
    cotask.task_list.append(cotask.Task(
        right_task.taskmotor_gen_fcn(), name="Right_Motor",
        priority=5, period=MOTOR_PERIOD_MS, profile=True))
    cotask.task_list.append(cotask.Task(
        user_task.taskuser_gen_fcn(), name="User",
        priority=1, period=10, profile=True))

    try:
        while not system.exit_requested:
            cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        print("Stopped by Ctrl-C")
    finally:
        left_task.stop()
        right_task.stop()


if __name__ == "__main__":
    main()
