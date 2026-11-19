from pyb import Pin, Timer


class MotorDriver:
    '''Interface for a motor driver using PWM, DIR, and nSLP pins.'''

    def __init__(self, pwm_pin, dir_pin, nslp_pin, tim, chan):
        '''Initializes a MotorDriver object.'''

        self._dir_pin = Pin(dir_pin, mode=Pin.OUT_PP, value=0)
        self._nslp_pin = Pin(nslp_pin, mode=Pin.OUT_PP, value=0)

        self._pwm_chan = tim.channel(
            chan,
            pin=pwm_pin,
            mode=Timer.PWM,
            pulse_width_percent=0
        )

    def enable(self):
        '''Takes the driver out of sleep in brake mode.'''
        self._pwm_chan.pulse_width_percent(0)
        self._nslp_pin.high()

    def disable(self):
        '''Puts the driver into sleep/coast mode.'''
        self._pwm_chan.pulse_width_percent(0)
        self._nslp_pin.low()

    def set_effort(self, effort):
        '''Sets motor effort from -100 to +100 percent.'''

        if effort > 100:
            effort = 100
        elif effort < -100:
            effort = -100

        if effort >= 0:
            self._dir_pin.high()
        else:
            self._dir_pin.low()

        self._pwm_chan.pulse_width_percent(int(abs(effort)))
