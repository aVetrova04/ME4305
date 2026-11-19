# boot.py
# MicroPython runs main.py automatically after this file exits.
from pyb import Pin

# keep both motor drivers asleep during startup
Pin(Pin.cpu.C6, Pin.OUT_PP, value=0)
Pin(Pin.cpu.C7, Pin.OUT_PP, value=0)
