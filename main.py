import machine
import time
from machine import Pin

uart = machine.UART(2, baudrate=9600, bits=8, stop=1)
uart.write("ANSW2")
# Definieren der In und Output Pins
LED_Rot = Pin(19, Pin.OUT)
LED_Blau = Pin(23, Pin.OUT)
LED_Gruen = Pin(5, Pin.OUT)
Reed_Switch = Pin(18, Pin.IN)
Start = Pin(13, Pin.IN)
Stop = Pin(12, Pin.IN)
Reset = Pin(27, Pin.IN)
status = 0
# Status kann 1 = Gestoppt, 2 = Laufend oder 3 = Error sein.
while True:
    if status == 0:
        uart.write("DI\r")
        LED_Rot.value(0)
        LED_Blau.value(1)
        LED_Gruen.value(0)
        if Start.value() and not Stop.value():
            status = 1
    elif status == 1:
        uart.write("EN\r")
        uart.write("V1000\r")
        LED_Rot.value(0)
        LED_Blau.value(0)
        LED_Gruen.value(1)
        if not Reed_Switch.value():
            status = 2
        if Stop.value():
            status = 0

    else:
        uart.write("DI\r")
        LED_Blau.value(0)
        LED_Gruen.value(0)
        LED_Rot.value(1)
        if Reset.value():
            status = 0
    time.sleep_ms(10)
    print(status)
