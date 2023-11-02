import machine
import time
from machine import Pin

uart = machine.UART(2, baudrate=9600, bits=8, stop=1)
uart.write("ANSW0")

# Definieren der In und Output Pins
LED_Rot = Pin(19, Pin.OUT)
LED_Blau = Pin(23, Pin.OUT)
LED_Gruen = Pin(5, Pin.OUT)
Reed_Switch = Pin(18, Pin.IN)
Start = Pin(13, Pin.IN)
Stop = Pin(12, Pin.IN)
Reset = Pin(27, Pin.IN)

# Anpassungen
blink_interval = 500  # Zeit in Millisekunden für das Blinken der roten LED.
speed = str(1000)  # Geschwindigkeit des Motors währen dem Betrieb.
Error_Strom = 20  # Grenzstrom, bei wellchem der Motor einen Fehler meldet in mA.
Laufzeit = 20000  # Zeit in ms, nach dem der Motor automatisch stoppt.

# Setzen der Variablen
last_blink_time = 0
status = 0
Zeitmesser = 0

# Status kann 1 = Gestoppt, 2 = Laufend oder 3 = Error sein.
while True:
    if status == 0:
        Zeitmesser = time.ticks_ms()
        uart.write("DEC10\r")
        uart.write("V0\r")
        LED_Blau.value(1)
        LED_Gruen.value(0)
        if Start.value() and not Stop.value():
            status = 1

        # Überprüfen, ob der Reed-Schalter ausgeschaltet ist und die rote LED blinken lassen
        current_time = time.ticks_ms()
        if current_time - last_blink_time >= blink_interval:
            if not Reed_Switch.value():
                last_blink_time = current_time
                LED_Rot.value(not LED_Rot.value())  # Umkehren des aktuellen Zustands der roten LED
            else:
                LED_Rot.value(0)

    elif status == 1:
        uart.write("EN\r")
        uart.write("AC10\r")
        uart.write("V" + speed + "\r")
        LED_Rot.value(0)
        LED_Blau.value(0)
        LED_Gruen.value(1)
        if not Reed_Switch.value():
            print("Der Materialfluss wurde unterbrochen")
            status = 2
        if Stop.value():
            status = 0
        if time.ticks_ms() - Zeitmesser >= Laufzeit:
            status=0

    else:
        uart.write("DI\r")
        LED_Blau.value(0)
        LED_Gruen.value(0)
        LED_Rot.value(1)
        if Reset.value():
            status = 0
    time.sleep_ms(10)

    # Maximalstromerkennen
    uart.write("GRC\r\n")
    data = uart.read()
    if data and not status == 2:
        try:
            data = data.decode("utf-8")
            data = data.strip()
            data_as_float = float(data)
            if data_as_float > Error_Strom:
                print("Der Maximalstrom wurde überschritten")
                status = 2
        except:
            pass
