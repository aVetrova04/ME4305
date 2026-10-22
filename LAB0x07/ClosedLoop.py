class ClosedLoop:

    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.vel_need = 0

        self.vel_now = 0
        self.v_min = 0
        self.v_max = 0

        self.error_prev = 0
        self.error_now = 0

    def set_vel(self, vel):
        self.vel_need = vel

    def set_kp(self, kp):
        self.kp = kp

    def set_ki(self, ki):
        self.ki = ki

    def set_lim(self, v_min, v_max):
        self.v_min = v_min
        self.v_max = v_max

    def reset(self):
        self.error_prev = 0
        self.error_now = 0

    def update(self, meas):
        self.vel_now = meas

        # Velocity error
        self.error_now = self.vel_need - self.vel_now

        # Proportional control
        voltage = self.kp * self.error_now

        # Voltage saturation
        if voltage > self.v_max:
            voltage = self.v_max

        elif voltage < self.v_min:
            voltage = self.v_min

        # Save current error
        self.error_prev = self.error_now

        return voltage