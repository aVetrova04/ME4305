from pyb import Timer
from time import ticks_us, ticks_diff


class EncoderDriver:
    '''A quadrature encoder decoding interface encapsulated in a Python class.'''

    def __init__(self, tim, chA_pin, chB_pin):
        '''Initializes an Encoder object and its hardware timer.'''

        # Configure a 16-bit timer for quadrature decoding.
        self.timer = Timer(tim, period=0xFFFF, prescaler=0)
        self.timer.channel(1, pin=chA_pin, mode=Timer.ENC_AB)
        self.timer.channel(2, pin=chB_pin, mode=Timer.ENC_AB)

        # Logical encoder state.
        self.position = 0
        self.delta = 0
        self.dt = 0

        # References used by the next update().
        self.prev_count = self.timer.counter()


        self.prev_time = ticks_us()

    def update(self):
        '''Updates position and velocity information from the timer.'''

        # Read the current raw 16-bit hardware count.
        current_count = self.timer.counter()

        # Find the raw change since the previous update.
        self.delta = current_count - self.prev_count

        # Correct for 16-bit timer underflow/overflow.
        if self.delta > 32767:
            self.delta -= 65536
        elif self.delta < -32768:
            self.delta += 65536

        # Accumulate the corrected signed change.
        self.position += self.delta

        # Find elapsed time in seconds.
        now = ticks_us()
        self.dt = ticks_diff(now, self.prev_time) / 1_000_000

        # Save references for the next update.
        self.prev_count = current_count
        self.prev_time = now

    def get_position(self):
        '''Returns the position calculated by the most recent update().'''
        return self.position

    def get_velocity(self):
        '''Returns velocity in encoder ticks per second.'''
        if self.dt <= 0:
            return 0
        return self.delta / self.dt

    def zero(self):
        '''Sets the current physical encoder location as logical position zero.'''
        self.position = 0
        self.delta = 0
        self.prev_count = self.timer.counter()
        self.prev_time = ticks_us()