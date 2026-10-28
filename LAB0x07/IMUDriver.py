# check out interrupts later to maybe integrate the line sensor with this
# TODO: FIRST TIME EVER
    # Enter IMU Mode and let it run auto calibration
    # Do the thingy and read CALIB_STAT
    # (SYS)(GYR)(ACC)(MAG) 2 bits each: 00=uncalibrated, 11=calibrated
    # If all good, save the calibration profile (radius and offsets)

# TODO: ON STARTUP
    # [CONFIG_MODE]: xxxx0000b on startup

    # CHECK IF DEVICE IS ALIVE
    # Set bit SELF_TEST in SYS_TRIGGER register
    # Wait 400ms
    # Read SYS_ERROR register (0x3A)
        # 0 --> no error
        # 3 --> error
            # Read ST_RESULT (0x36) --> 1=success, 0=fail
            # xxxx(MCU)(GYR)(MAG)(ACC)

    # LOAD CALIBRATION PROFILE
    # From saved calibration profile:
    #Set ACCELEROMETER radius
        # ACC_RADIUS_(MSB/LSB)
    #Sensor calibration offsets (6-bit/sensor MSB&LSB of each axis)
        # Write calibration value
        # Ex register name: ACC_OFFSET_X_LSB

# TODO: Operating mode configuration - IMU MODE (packaged accel and gyro data)
    # Write to [OPR_MODE]: xxxx1000b

# TODO: Define axes for IMU
    # Write to AXIS_MAP_CONFIG and AXIS_MAP_SIGN
        # [AXIS_MAP_CONFIG]: xx100100 (CHECK - this is the default)
        # [AXIS_MAP_SIGN]: xxxxx110 (SET - remap sign of x and y)
    # Use table and graphic in datasheet AND Windows convention
    # Windows PITCH increases from turning clockwise (-180--180)
    # YAW increases from turning clockwise (0--360)
# ** Sensor power modes can be configured

# TODO: Configure units for IMU (fusion) mode
    # Write to [UNIT_SEL]: 0xxxx000b [WINDOWS, E: (deg), (deg/s), (m/s^2)]

# TODO: Read sensor data
    # [ACC/GYR]_DATA_[X/Y/Z]_[MSB/LSB] - accelerometer/gyroscope (deg/s)
    # EUL_[HEADING/ROLL/PITCH]_[MSB/LSB] - Euler angles
    # GRV_DATA_[X/Y/Z] - gravity angle

# TODO: Use the IMU for orientation and angular rate, and the encoders for forward-speed estimation

from micropython import const
from struct import calcsize, unpack_from
from pyb import delay
from math import pi


class BNO055:
    DEV_ADDR = const(0x28)

    # BNO055 register addresses
    CHIP_ID = const(0x00)
    PAGE_ID = const(0x07)

    GYR_DATA_Z = (const(0x18), b"<h")
    EUL_HEADING_MSB = (const(0x1B), b"<B")
    EUL_HEADING_LSB = (const(0x1A), b"<B")
    EUL_HEADING = (const(0x1A), b"<h")
    EUL_DATA_ALL = (const(0x1A), b"<hhh")

    CALIB_STAT = const(0x35)
    ST_RESULT = const(0x36)
    SYS_STATUS = const(0x39)
    SYS_ERROR = const(0x3A)
    UNIT_SEL = const(0x3B)
    OPR_MODE = const(0x3D)
    PWR_MODE = const(0x3E)
    SYS_TRIGGER = const(0x3F)
    AXIS_MAP_CONFIG = const(0x41)
    AXIS_MAP_SIGN = const(0x42)

    CALIB_START = const(0x55)
    CALIB_LENGTH = const(22)

    CONFIG_MODE = const(0x00)
    IMUPLUS_MODE = const(0x08)
    NORMAL_POWER = const(0x00)

    COUNTS_PER_DEGREE = 16.0
    DEG_TO_RAD = pi / 180.0

    # Fixed mounting configuration from the original TODO:
    # AXIS_MAP_CONFIG = 0b00100100 (0x24)
    # AXIS_MAP_SIGN   = 0b00000110 (0x06), flip sensor X and Y.
    DEFAULT_AXIS_MAP_CONFIG = const(0x24)
    DEFAULT_AXIS_MAP_SIGN = const(0x06)

    def __init__(
        self,
        i2c,
        address=DEV_ADDR,
        axis_map_config=DEFAULT_AXIS_MAP_CONFIG,
        axis_map_sign=DEFAULT_AXIS_MAP_SIGN,
        heading_sign=1,
        yaw_rate_sign=1,
    ):
        self._i2c = i2c
        self.DEV_ADDR = address
        self._buf = bytearray(22)

        # Mounting and sign conventions.
        self.axis_map_config = axis_map_config
        self.axis_map_sign = axis_map_sign
        self.heading_sign = 1 if heading_sign >= 0 else -1
        self.yaw_rate_sign = 1 if yaw_rate_sign >= 0 else -1

        self.initialized = False

    def _read_reg(self, reg):
        length = calcsize(reg[1])
        buf = memoryview(self._buf)[:length]
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0])
        return unpack_from(reg[1], buf)

    def read_bytes(self, register, length):
        """Read consecutive registers into a list of bytes."""
        buf = memoryview(self._buf)[:length]
        self._i2c.mem_read(buf, self.DEV_ADDR, register)
        return list(buf)

    def write_reg(self, register, value):
        """Write one byte to a register."""
        self._i2c.mem_write(
            bytes((value & 0xFF,)), self.DEV_ADDR, register
        )

    def read_u8(self, register):
        return self._read_reg((register, b"<B"))[0]

    def read_i16(self, register):
        """Read a signed little-endian 16-bit value."""
        return self._read_reg((register, b"<h"))[0]

    def check_device(self):
        chip_id = self.read_u8(self.CHIP_ID)
        if chip_id != 0xA0:
            raise OSError(
                "BNO055 not found; CHIP_ID=0x%02X" % chip_id
            )
        return True

    def set_mode(self, mode):
        """Change operating mode and wait for the transition."""
        self.write_reg(self.OPR_MODE, mode)

        if mode == self.CONFIG_MODE:
            delay(25)
        else:
            delay(10)

    def begin(self):
        """Configure the sensor and enter IMUPLUS mode."""
        self.check_device()

        self.set_mode(self.CONFIG_MODE)
        self.write_reg(self.PAGE_ID, 0x00)
        self.write_reg(self.PWR_MODE, self.NORMAL_POWER)
        delay(10)

        # Degree-based Euler angles and degrees/second gyro rates.
        self.write_reg(self.UNIT_SEL, 0x00)

        # Apply the mounting configuration from the TODO comments.
        self.write_reg(
            self.AXIS_MAP_CONFIG, self.axis_map_config
        )
        self.write_reg(
            self.AXIS_MAP_SIGN, self.axis_map_sign
        )

        self.set_mode(self.IMUPLUS_MODE)
        delay(20)

        self.initialized = True
        return True

    def euler(self):
        """Return heading, roll, pitch in degrees."""
        head, roll, pitch = self._read_reg(self.EUL_DATA_ALL)

        return (
            self.heading_sign * head / self.COUNTS_PER_DEGREE,
            roll / self.COUNTS_PER_DEGREE,
            pitch / self.COUNTS_PER_DEGREE,
        )

    def heading_deg(self):
        """Heading in degrees with chassis sign conversion."""
        raw = self.read_i16(self.EUL_HEADING[0])
        return self.heading_sign * raw / self.COUNTS_PER_DEGREE

    def heading_rad(self):
        return self.heading_deg() * self.DEG_TO_RAD

    def yaw_rate_deg_s(self):
        """Mapped Z-axis gyro rate in degrees/second."""
        raw = self.read_i16(self.GYR_DATA_Z[0])
        return self.yaw_rate_sign * raw / self.COUNTS_PER_DEGREE

    def yaw_rate_rad_s(self):
        return self.yaw_rate_deg_s() * self.DEG_TO_RAD

    def calibration_status(self):
        """Return SYS, GYR, ACC and MAG calibration levels (0-3)."""
        value = self.read_u8(self.CALIB_STAT)

        return {
            "sys": (value >> 6) & 0x03,
            "gyro": (value >> 4) & 0x03,
            "accel": (value >> 2) & 0x03,
            "mag": value & 0x03,
        }

    def gyro_accel_calibrated(self):
        status = self.calibration_status()
        return status["gyro"] == 3 and status["accel"] == 3

    def read_calibration_profile(self):
        """Read the calibration profile in CONFIG mode."""
        if self.read_u8(self.OPR_MODE) != self.CONFIG_MODE:
            raise RuntimeError(
                "Switch to CONFIG mode before reading profile"
            )

        return self.read_bytes(
            self.CALIB_START, self.CALIB_LENGTH
        )

    def write_calibration_profile(self, profile):
        """Restore a validated profile and return to IMUPLUS mode."""
        if len(profile) != self.CALIB_LENGTH:
            raise ValueError(
                "Calibration profile must contain 22 bytes"
            )

        for value in profile:
            if type(value) is not int or not 0 <= value <= 255:
                raise ValueError(
                    "Calibration profile values must be bytes"
                )

        self.set_mode(self.CONFIG_MODE)
        self._i2c.mem_write(
            bytes(profile), self.DEV_ADDR, self.CALIB_START
        )
        delay(10)

        self.set_mode(self.IMUPLUS_MODE)
        self.initialized = True

    def capture_calibration_profile(self):
        """Switch to CONFIG, read profile, and return to IMUPLUS."""
        self.set_mode(self.CONFIG_MODE)
        profile = self.read_calibration_profile()
        self.set_mode(self.IMUPLUS_MODE)
        self.initialized = True
        return profile

    def self_test(self):
        """Run the built-in self-test with motors disabled."""
        self.set_mode(self.CONFIG_MODE)

        # SYS_TRIGGER bit 0 starts the built-in self-test.
        self.write_reg(self.SYS_TRIGGER, 0x01)
        delay(400)

        sys_error = self.read_u8(self.SYS_ERROR)
        st_result = self.read_u8(self.ST_RESULT)

        self.set_mode(self.IMUPLUS_MODE)

        if sys_error != 0:
            raise OSError(
                "BNO055 SYS_ERROR=0x%02X" % sys_error
            )

        if (st_result & 0x0F) != 0x0F:
            raise OSError(
                "BNO055 self-test failed: 0x%02X" % st_result
            )

        return True
