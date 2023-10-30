import machine
import time
from machine import Pin

uart = machine.UART(2, baudrate=9600, bits=8, stop=1)
uart.write("ANSW0")

# Define the input and output pins
LED_Red = Pin(19, Pin.OUT)
LED_Blue = Pin(23, Pin.OUT)
LED_Green = Pin(5, Pin.OUT)
Reed_Switch = Pin(18, Pin.IN)
Start = Pin(13, Pin.IN)
Stop = Pin(12, Pin.IN)
Reset = Pin(27, Pin.IN)


# Adjustments
blink_interval = 500  # Time in milliseconds for the red LED to blink.
speed = str(1000)  # Motor speed during operation.
Error_Current = 20  # Current threshold at which the motor reports an error in mA.
Runtime = 20000  # Time in ms after which the motor automatically stops.

# Set variables
last_blink_time = 0
status = 0
Timer = 0
Readable_Variables = ["GRC", "GV", "TEM", "POS"]

# Status can be 1 = Stopped, 2 = Running, or 3 = Error.
while True:
    if status == 0:
        Timer = time.ticks_ms()
        uart.write("DEC10\r")
        uart.write("V0\r")
        LED_Blue.value(1)
        LED_Green.value(0)
        if Start.value() and not Stop.value():
            status = 1

        # Check if the Reed switch is turned off and blink the red LED
        current_time = time.ticks_ms()
        if current_time - last_blink_time >= blink_interval:
            if not Reed_Switch.value():
                last_blink_time = current_time
                LED_Red.value(not LED_Red.value())  # Toggle the current state of the red LED
            else:
                LED_Red.value(0)

    elif status == 1:
        uart.write("EN\r")
        uart.write("AC10\r")
        uart.write("V" + speed + "\r")
        LED_Red.value(0)
        LED_Blue.value(0)
        LED_Green.value(1)
        if not Reed_Switch.value():
            print("Material flow interrupted")
            status = 2
        if Stop.value():
            status = 0
        if time.ticks_ms() - Timer >= Runtime:
            status = 0

    else:
        uart.write("DI\r")
        LED_Blue.value(0)
        LED_Green.value(0)
        LED_Red.value(1)
        if Reset.value():
            status = 0
    time.sleep_ms(100)

    # Read all Values that are declared in Readable_Variables

    uart.write("\r\n".join(Readable_Variables) + "\r\n")
    data = uart.read()
    if data and status != 2:
        received_values = data[:-2].decode().split('\r\n')

        if len(received_values) == len(Readable_Variables):
            for i in range(len(Readable_Variables)):
                exec(f"{Readable_Variables[i]} = int(received_values[i])")

            if GRC > Error_Current:
                print("Maximum current exceeded")
                status = 2
            print("GRC:", TEM)
            print("GV:", GV)
            print("GRC:", GRC)
            print("POS:", POS)




