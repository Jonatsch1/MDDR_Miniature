import serial
import RPi.GPIO as GPIO
import time

# GPIO-Pins
LED_Red = 17
LED_Blue = 27
LED_Green = 22
Reed_Switch = 18
Start = 5
Stop = 6
Reset = 26

# GPIO-Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_Red, GPIO.OUT)
GPIO.setup(LED_Blue, GPIO.OUT)
GPIO.setup(LED_Green, GPIO.OUT)
GPIO.setup(Reed_Switch, GPIO.IN)
GPIO.setup(Start, GPIO.IN)
GPIO.setup(Stop, GPIO.IN)
GPIO.setup(Reset, GPIO.IN)

# UART-Setup
serial_port = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)
print("Serial Setup done")


# Funktion für LED-Blinken
def blink_LED():
    GPIO.output(LED_Red, not GPIO.input(LED_Red))


# Adjustments
blink_interval = 0.5  # Zeit in Sekunden für das Blinken der roten LED
speed = "1000"  # Motorgeschwindigkeit während des Betriebs
Error_Current = 20  # Stromschwellenwert, bei dem der Motor einen Fehler meldet (in mA)
Runtime = 20  # Zeit in Sekunden, nach der der Motor automatisch stoppt

# Setze Variablen
last_blink_time = 0
status = 0
Timer = 0
Readable_Variables = ["GRC", "GV", "TEM", "POS"]

# Status kann 1 = Gestoppt, 2 = Laufend oder 3 = Fehler sein
try:
    while True:
        if status == 0:
            Timer = time.time()
            serial_port.write(b"DEC10\r")
            serial_port.write(b"V0\r")
            GPIO.output(LED_Blue, 1)
            GPIO.output(LED_Green, 0)
            if GPIO.input(Start) and not GPIO.input(Stop):
                status = 1

            # Überprüfe, ob der Reed-Schalter ausgeschaltet ist und lasse die rote LED blinken
            current_time = time.time()
            if current_time - last_blink_time >= blink_interval:
                if not GPIO.input(Reed_Switch):
                    last_blink_time = current_time
                    blink_LED()
                else:
                    GPIO.output(LED_Red, 0)

        elif status == 1:
            serial_port.write(b"EN\r")
            serial_port.write(f"AC10\rV{speed}\r".encode())
            GPIO.output(LED_Red, 0)
            GPIO.output(LED_Blue, 0)
            GPIO.output(LED_Green, 1)
            if not GPIO.input(Reed_Switch):
                print("Materialfluss unterbrochen")
                status = 2
            if GPIO.input(Stop):
                status = 0
            if time.time() - Timer >= Runtime:
                status = 0

        else:
            serial_port.write(b"DI\r")
            GPIO.output(LED_Blue, 0)
            GPIO.output(LED_Green, 0)
            GPIO.output(LED_Red, 1)
            if GPIO.input(Reset):
                status = 0

        time.sleep(0.01)
        time.sleep(0.1)

        # Erkenne maximalen Strom
        serial_port.write(b"GRC\r\n")
        # Lese alle Werte, die in Readable_Variables deklariert sind
        serial_port.write("\r\n".join(Readable_Variables).encode() + b"\r\n")
        data = serial_port.read(100).decode("utf-8").strip()

        if data and not status == 2:
            try:
                data_as_float = float(data)
                if data_as_float > Error_Current:
                    print("Maximaler Strom überschritten")
                    status = 2
            except ValueError:
                pass
except KeyboardInterrupt:
    GPIO.cleanup()
    serial_port.close()
    print("Programm beendet")
