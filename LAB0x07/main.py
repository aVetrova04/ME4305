from pyb import Pin, Timer, USB_VCP, ADC, I2C

from MotorDriver import MotorDriver
from EncoderDriver import EncoderDriver
from taskmotor import MotorData, TaskMotor
from taskuser import TaskUser
from ClosedLoop import ClosedLoop
import cotask

from line_sensor import LineSensor, CHANNELS
from line_calibration import save_calibration, load_calibration
from taskline import TaskLine
from IMU_calibration import initialize_imu


# I2C bus for the BNO055.
# Confirm that I2C(1) uses the SDA/SCL pins connected to your harness.
i2c = I2C(1, I2C.MASTER, baudrate=100000)


class SystemData:
    def __init__(self):
        self.exit_requested = False


def main():
    # Both PWM channels use timer 2.
    pwm_tim = Timer(2, freq=20_000)

    left_motor = MotorDriver(
        Pin.cpu.A0, Pin.cpu.A6, Pin.cpu.C6, pwm_tim, 1
    )
    right_motor = MotorDriver(
        Pin.cpu.A1, Pin.cpu.A7, Pin.cpu.C5, pwm_tim, 2
    )

    left_enc = EncoderDriver(1, Pin.cpu.A8, Pin.cpu.A9)
    right_enc = EncoderDriver(3, Pin.cpu.B4, Pin.cpu.B5)

    # SAFETY: Disable both motors before sensor initialization/calibration.
    left_motor.set_effort(0)
    left_motor.disable()
    right_motor.set_effort(0)
    right_motor.disable()

    # ---------------------------------------------------------
    # IMU startup initialization and calibration
    # ---------------------------------------------------------
    # If a valid calibration profile exists, initialize_imu()
    # restores it. Otherwise, it waits for gyro and accelerometer
    # calibration, then saves a new profile to this JSON file.
    #
    # Keep Romi stationary for gyro calibration and follow the
    # required axis-aligned orientations for accelerometer calibration.
    # This runs before the scheduler starts.

    print("Initializing BNO055 IMU...")

    try:
        imu = initialize_imu(
            i2c,
            filename="imu_calibration.json",
        )

        print("IMU initialized.")
        print("Calibration status:", imu.calibration_status())
        print("Heading (rad):", imu.heading_rad())
        print("Yaw rate (rad/s):", imu.yaw_rate_rad_s())

    except Exception as error:
        # Keep motors disabled if IMU startup fails.
        left_motor.set_effort(0)
        left_motor.disable()
        right_motor.set_effort(0)
        right_motor.disable()
        print("IMU initialization failed:", error)
        raise

    # ---------------------------------------------------------
    # Line sensor startup calibration
    # ---------------------------------------------------------

    SENSOR_PINS = [
        Pin.cpu.PC0, Pin.cpu.PC1, Pin.cpu.PC2, Pin.cpu.PC3,
        Pin.cpu.PA4, Pin.cpu.PC4, Pin.cpu.PB0, Pin.cpu.PB1
    ]

    MIN_CALIBRATION_SPAN = 10

    # Load the existing eight-channel QTR calibration.
    # This file is separate from imu_calibration.json.
    calibration = load_calibration(
        "line_calibration.json",
        CHANNELS,
        MIN_CALIBRATION_SPAN,
    )

    sensor = LineSensor(
        SENSOR_PINS,
        calibration["black"],
        calibration["white"],
        min_span=MIN_CALIBRATION_SPAN,
        samples=2,
    )

    # ---------------------------------------------------------
    # Tasks and shared data
    # ---------------------------------------------------------

    left_data = MotorData()
    right_data = MotorData()
    system = SystemData()

    left_task = TaskMotor(left_motor, left_enc, left_data)
    right_task = TaskMotor(right_motor, right_enc, right_data)
    user_task = TaskUser(
        USB_VCP(), left_data, right_data, system
    )

    # The IMU instance is available for later integration into
    # TaskLine, e.g. heading_rad() and yaw_rate_rad_s().
    line_task = TaskLine(
        sensor, left_data, right_data, config={}
    )

    # Motor tasks run every 10 ms.
    cotask.task_list.append(cotask.Task(
        left_task.taskmotor_gen_fcn(),
        name="Left_Motor",
        priority=5,
        period=10,
        profile=True,
    ))

    cotask.task_list.append(cotask.Task(
        right_task.taskmotor_gen_fcn(),
        name="Right_Motor",
        priority=5,
        period=10,
        profile=True,
    ))

    # Lowest priority: user interface.
    cotask.task_list.append(cotask.Task(
        user_task.taskuser_gen_fcn(),
        name="User",
        priority=1,
        period=20,
        profile=True,
    ))

    # Line-following task.
    cotask.task_list.append(cotask.Task(
        line_task.taskline_gen_fcn(),
        name="Line",
        priority=2,
        period=20,
        profile=True,
    ))

    # ---------------------------------------------------------
    # Cooperative scheduler
    # ---------------------------------------------------------

    try:
        while not system.exit_requested:
            cotask.task_list.pri_sched()

    except KeyboardInterrupt:
        pass

    finally:
        # Stop both motors on exit, Ctrl-C, or a scheduler error.
        left_task.stop()
        right_task.stop()


if __name__ == "__main__":
    main()
