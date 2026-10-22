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
    # EUL_[HEADING/ROLL/PITCH]_[MSB/LSB] - euler angles
    # GRV_DATA_[X/Y/Z] - gravity angle

# TODO:  Use the IMU for orientation and angular rate, and the encoders for forward-speed estimation



from micropython import const
from struct import calcsize, unpack_from

class BNO055:
    DEV_ADDR = const(0x28)

    class reg:
        EUL_HEADING_MSB = (const(0x1B), b"<B")
        EUL_HEADING_LSB = (const(0x1A), b"<B")
        EUL_HEADING = (const(0x1A), b"<h")
        EUL_DATA_ALL = (const(0x1A), b"<hhh")

    def __init__(self, i2c):
        self._i2c = i2c
        self._buf = bytearray((0 for n in range(22)))

    def _read_reg(self, reg):
        # Determine number of bytes to read
        length = calcsize(reg[1])

        # Create a memoryview object of the right size
        buf = memoryview(self._buf)[:length]

        # Read from the I2C bus into the memoryview
        self._i2c.mem_read(buf, self.DEV_ADDR, reg[0])

        # Unpack the bytes into a tuple
        return unpack_from(reg[1], buf)

    def euler(self):
        head, roll, pitch = self._read_reg(BNO055.reg.EUL_DATA_ALL)
        # due to resolution of heading/gyro 16 count/deg
        return (head / 16, roll / 16, pitch / 16)
