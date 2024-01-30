import serial
import RPi.GPIO as GPIO
import time
import asyncio
import json
import paho.mqtt.client as mqtt

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
serial_port.write(b"ANSW1\r\n")
print("Serial Setup done")

# MQTT Setup
mqtt_server = "localhost"
mqtt_port = 1883
mqtt_topic = "IO"
mqtt_speed_topic = "speed"
mqtt_button_topic = "button"  # New MQTT topic for button commands

# Callback functions for MQTT
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(mqtt_topic)

def on_publish(client, userdata, mid):
    print(f"Message Published with mid={mid}")

# Callback function for MQTT speed update
def on_speed_update(client, userdata, msg):
    global speed
    try:
        new_speed = float(msg.payload.decode())
        print(f"Received new speed: {new_speed}")
        speed = str(new_speed)  # Convert the value to string for use in uart_send
    except ValueError:
        print("Invalid speed value received.")

# Callback function for MQTT button commands
def on_button_command(client, userdata, msg):
    global status
    button_command = msg.payload.decode()

    if button_command == "Start":
        if status == 0:
            status = 1
    elif button_command == "Stop":
        if status == 1:
            status = 0
    elif button_command == "Reset":
        if status == 2:
            status = 0

# Create MQTT client
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_publish = on_publish
mqtt_client.message_callback_add(mqtt_speed_topic, on_speed_update)
mqtt_client.message_callback_add(mqtt_button_topic, on_button_command)

# Global variables
status = 0
GRC = 0
GN = 0
last_blink_time = 0
error_data = {"last_error": 0}
speed = "1000"  # Initial speed value

# Function to publish device state to MQTT
async def publish_state():
    global status, error_data
    prev_state = {
        "LED_Red": GPIO.input(LED_Red),
        "LED_Blue": GPIO.input(LED_Blue),
        "LED_Green": GPIO.input(LED_Green),
        "Reed_Switch": GPIO.input(Reed_Switch),
        "Start": GPIO.input(Start),
        "Stop": GPIO.input(Stop),
        "Reset": GPIO.input(Reset),
        "Status": status,
        "Last_Error": error_data["last_error"],
        "GRC": GRC
    }

    while True:
        current_state = {
            "LED_Red": GPIO.input(LED_Red),
            "LED_Blue": GPIO.input(LED_Blue),
            "LED_Green": GPIO.input(LED_Green),
            "Reed_Switch": GPIO.input(Reed_Switch),
            "Start": GPIO.input(Start),
            "Stop": GPIO.input(Stop),
            "Reset": GPIO.input(Reset),
            "Status": status,
            "Last_Error": error_data["last_error"],
            "GRC": GRC
        }

        if current_state != prev_state:
            payload = current_state.copy()

            # Get and update the GRC value from the device
            response = await uart_send("GRC\r\n")
            try:
                data_as_float = float(response)
                payload["GRC"] = data_as_float
            except ValueError:
                pass

            # Publish the device state to the MQTT topic
            mqtt_client.publish(mqtt_topic, json.dumps(payload), qos=1)

            prev_state = current_state

        await asyncio.sleep(0.1)

# Function for LED-Blinking
def blink_LED():
    GPIO.output(LED_Red, not GPIO.input(LED_Red))

# Adjustments
blink_interval = 0.5
Error_Current = 15
Runtime = 20

# Set variables
last_blink_time = 0

# Status can be 1 = Stopped, 2 = Running, or 3 = Error
async def uart_send(command, wait_for_response=True):
    try:
        serial_port.write(command.encode())
        await asyncio.sleep(0.1)
        if wait_for_response:
            response = await asyncio.to_thread(serial_port.read_until, b'\r\n')
            response_str = response.decode().strip()
            return response_str
        else:
            return ''
    except serial.serialutil.SerialException as e:
        print(f"SerialException: {e}")
        return ''

# Task to handle UART communication based on device status
async def uart_task():
    while True:
        if status == 1:
            await uart_send("EN\r")
            await uart_send(f"AC10\rV{speed}\r")
        elif status == 2:
            await uart_send("DI\r")
        await asyncio.sleep(0.1)

# Main task handling device states and errors
async def main_task():
    global status, last_blink_time, error_data, GRC, GN

    try:
        while True:
            if status == 0:
                GPIO.output(LED_Blue, 1)
                GPIO.output(LED_Green, 0)
                last_blink_time = time.time()
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
                last_blink_time = time.time()
                while status == 1:
                    if not GPIO.input(Reed_Switch):
                        status = 2
                        error_data["last_error"] = 0  # Set last_error to 0
                    if GPIO.input(Stop):
                        status = 0
                    if time.time() - last_blink_time >= Runtime:
                        status = 0

                    # Check for maximum current exceeded error
                    response = await uart_send("GRC\r\n")
                    try:
                        data_as_float = float(response)
                        GRC = data_as_float
                        if data_as_float > Error_Current:
                            print("Maximum current exceeded")
                            status = 2
                            error_data["last_error"] = 1  # Set last_error to 1
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
    except asyncio.CancelledError:
        pass
    finally:
        print("Cleaning up and disconnecting...")
        GPIO.cleanup()
        serial_port.close()
        print("Program terminated")

# Main function running all tasks concurrently
async def main():
    try:
        # Connect to the MQTT broker
        mqtt_client.connect(mqtt_server, mqtt_port, 60)
        mqtt_client.loop_start()

        # Subscribe to the speed and button topics
        mqtt_client.subscribe(mqtt_speed_topic)
        mqtt_client.subscribe(mqtt_button_topic)

        # Run all tasks concurrently
        await asyncio.gather(main_task(), uart_task(), publish_state())
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect from the MQTT broker
        print("Cleaning up and disconnecting...")
        GPIO.cleanup()
        serial_port.close()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("Program terminated")

if __name__ == "__main__":
    asyncio.run(main())
