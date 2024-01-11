import serial
import RPi.GPIO as GPIO
import time
import asyncio

# GPIO-Pins
LED_Red = 22
LED_Blue = 27
LED_Green = 17
Reed_Switch = 23
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
serial_port = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.1)
serial_port.write(b"ANSW0\r\n")
print("Serial Setup done")

# Funktion für LED-Blinken
def blink_LED():
    GPIO.output(LED_Red, not GPIO.input(LED_Red))


# Adjustments
blink_interval = 0.5
speed = "1000"
Error_Current = 15
Runtime = 20

# Setze Variablen
last_blink_time = 0
status = 0
Timer = 0
Readable_Variables = ["GRC", "GV", "TEM", "POS"]

# Status kann 1 = Gestoppt, 2 = Laufend oder 3 = Fehler sein
async def uart_send(command, wait_for_response=True):
    serial_port.write(command.encode())
    await asyncio.sleep(0.1)  # Warten Sie kurz, um sicherzustellen, dass Daten gesendet werden
    if wait_for_response:
        response = await asyncio.to_thread(serial_port.read_until, b'\r\n')
        return response.decode().strip()
    else:
        return ''
async def uart_task():
    while True:
        if status == 1:
            await uart_send("EN\r")
            await uart_send(f"AC10\rV{speed}\r")
        elif status == 2:
            await uart_send("DI\r")
        await asyncio.sleep(0.1)

async def main_task():
    global status, Timer, last_blink_time

    while True:
        if status == 0:
            GPIO.output(LED_Blue, 1)
            GPIO.output(LED_Green, 0)
            Timer = time.time()
            last_blink_time = time.time()  # Initialisiere last_blink_time hier
            await uart_send("DEC10\rV0\r")
            while status == 0:
                if GPIO.input(Start) and not GPIO.input(Stop):
                    status = 1
                current_time = time.time()
                if current_time - last_blink_time >= blink_interval:
                    if not GPIO.input(Reed_Switch):
                        last_blink_time = current_time
                        blink_LED()
                    else:
                        GPIO.output(LED_Red, 0)
                await asyncio.sleep(0.1)

        elif status == 1:
            GPIO.output(LED_Red, 0)
            GPIO.output(LED_Blue, 0)
            GPIO.output(LED_Green, 1)
            while status == 1:
                if not GPIO.input(Reed_Switch):
                    print("Materialfluss unterbrochen")
                    status = 2
                if GPIO.input(Stop):
                    status = 0
                if time.time() - Timer >= Runtime:
                    status = 0
                response = await uart_send("GRC\r\n")
                print(response)
                try:
                    data_as_float = float(response)
                    if data_as_float > Error_Current:
                        print("Maximaler Strom überschritten")
                        status = 2
                except ValueError:
                    pass
                await asyncio.sleep(0.1)

        else:
            GPIO.output(LED_Blue, 0)
            GPIO.output(LED_Green, 0)
            GPIO.output(LED_Red, 1)
            while status == 2:
                if GPIO.input(Reset):
                    status = 0
                await asyncio.sleep(0.1)

if __name__ == "__main__":
    try:
        # Initialisiere den Event-Loop
        loop = asyncio.get_event_loop()

        # Starte beide Tasks im Event-Loop
        asyncio.ensure_future(main_task())
        asyncio.ensure_future(uart_task())

        # Führe den Event-Loop aus
        loop.run_forever()
    except KeyboardInterrupt:
        GPIO.cleanup()
        serial_port.close()
        print("Programm beendet")