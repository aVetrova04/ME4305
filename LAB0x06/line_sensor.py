from pyb import ADC


# Printed QTR sensor numbers, not MCU pin names or ADC peripheral numbers.
CHANNELS = [1, 2, 3, 4, 5, 6, 7, 8]

# Positive is Romi's left; negative is Romi's right.
# Channel 7 is the center of the complete 13-channel array.
SENSOR_POSITIONS_MM = [24, 20, 16, 12, 8, 4, 0, -4]


class LineSensor:
    """Eight-channel calibrated analog reflectance sensor."""

    def __init__(self, pins, black, white, min_span=10, samples=2):
        if len(pins) != len(CHANNELS):
            raise ValueError("Exactly eight ADC pins are required")
        if len(black) != len(CHANNELS):
            raise ValueError("Expected eight black calibration values")
        if len(white) != len(CHANNELS):
            raise ValueError("Expected eight white calibration values")
        if samples < 1:
            raise ValueError("samples must be positive")

        self.adcs = [ADC(pin) for pin in pins]
        self.black = list(black)
        self.white = list(white)
        self.min_span = min_span
        self.samples = samples

        for w, b in zip(self.white, self.black):
            if not (0 <= w < b <= 4095):
                raise ValueError("Expected white < black on every channel")
            if b - w < min_span:
                raise ValueError("Insufficient calibration contrast")

        self.raw = [0] * len(CHANNELS)
        self.weights = [0.0] * len(CHANNELS)
        self.centroid_mm = None
        self.total_weight = 0.0

    def read_raw(self):
        """Read and average each channel; returns ADC counts."""
        for i, adc in enumerate(self.adcs):
            total = 0
            for _ in range(self.samples):
                total += adc.read()
            self.raw[i] = total / self.samples
        return self.raw

    def update(self):
        """Return centroid in mm, or None if no line weight is detected."""
        self.read_raw()

        weighted_sum = 0.0
        total_weight = 0.0

        for i in range(len(CHANNELS)):
            reading = self.raw[i]
            span = self.black[i] - self.white[i]

            weight = (reading - self.white[i]) / span
            weight = max(0.0, min(1.0, weight))

            self.weights[i] = weight
            total_weight += weight
            weighted_sum += weight * SENSOR_POSITIONS_MM[i]

        self.total_weight = total_weight

        if total_weight <= 1e-9:
            self.centroid_mm = None
            return None

        self.centroid_mm = weighted_sum / total_weight
        return self.centroid_mm
