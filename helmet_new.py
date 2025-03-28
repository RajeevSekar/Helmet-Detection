import sys
import time
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import numpy as np
from picamera2 import Picamera2
from tflite_runtime.interpreter import Interpreter
import cv2

# GPIO Pin Setup
buzzer_pin = 18  # GPIO 18 (Pin 12) for buzzer signal
relay_pin = 17   # GPIO 17 (Pin 11) for relay
sda_pin = 2      # GPIO 2 (Pin 3) for SDA on MPU6050
scl_pin = 3      # GPIO 3 (Pin 5) for SCL on MPU6050

GPIO.setmode(GPIO.BCM)  # Set GPIO to BCM mode
GPIO.setup(buzzer_pin, GPIO.OUT)  # Buzzer pin as output

# Relay pin setup tracking (to avoid re-setup)
relay_setup_done = False

# Initialize MPU6050 accelerometer
mpu = mpu6050(0x68)

# Load the TFLite model
model_path = 'helmet.tflite'  # Update this path if needed
interpreter = Interpreter(model_path)
interpreter.allocate_tensors()

# Get input/output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Function to control the buzzer
def control_buzzer(times, duration):
    for _ in range(times):
        GPIO.output(buzzer_pin, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(buzzer_pin, GPIO.LOW)
        time.sleep(0.2)

# Function to calibrate accelerometer
def calibrate_accelerometer():
    print("Calibrating accelerometer... Please ensure the vehicle is stationary.")
    x_calibration = 0
    calibration_samples = 25  # Calibration sample count
    for _ in range(calibration_samples):
        accel_data = mpu.get_accel_data()
        x_calibration += accel_data['x']
        time.sleep(0.05)
    x_calibration /= calibration_samples
    print(f"Calibration complete. Baseline X-axis: {x_calibration}")
    control_buzzer(2, 1)  # Sound buzzer twice to indicate calibration completion
    return x_calibration

# Function to get accelerometer data and apply calibration
def get_accelerometer_reading(x_calibration):
    accel_data = mpu.get_accel_data()
    x_accel = accel_data['x'] - x_calibration  # Subtract calibration value
    return abs(x_accel)

# Function to capture an image from the camera
def capture_image():
    picam2 = Picamera2()  # Initialize the camera
    picam2.start()
    time.sleep(2)  # Allow the camera to warm up
    image = picam2.capture_array()
    picam2.stop()  # Stop the camera
    picam2.close()  # Close the camera to release resources
    if image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
    # Resize the image to model input size
    image = cv2.resize(image, (input_details[0]['shape'][1], input_details[0]['shape'][2]))
    # Add batch dimension if needed
    if len(input_details[0]['shape']) == 4:
        image = np.expand_dims(image, axis=0)
    image = np.array(image, dtype=np.uint8)  # Ensure image type is uint8
    return image

# Function to run helmet detection logic
def run_helmet_detection():
    control_buzzer(1, 1)  # Initial buzzer sound
    x_calibration = calibrate_accelerometer()  # Calibrate accelerometer

    while True:
        print("Capturing image...")
        image_data = capture_image()
        print("Image captured. Running inference...")

        interpreter.set_tensor(input_details[0]['index'], image_data)
        interpreter.invoke()

        # Get the result of the inference
        output_data = interpreter.get_tensor(output_details[0]['index'])

        # Check the class (0: helmet detected; 1: face detected but no helmet; 2: no person)
        detected_class = np.argmax(output_data)

        # Get accelerometer reading
        accelerometer_reading = get_accelerometer_reading(x_calibration)

        # Helmet detected
        if detected_class == 0:  
            print("Helmet detected!")

            # Only setup relay if not already set up
            if not relay_setup_done:
                GPIO.setup(relay_pin, GPIO.OUT)  # Set the relay pin as an output pin
                relay_setup_done = True  # Mark relay as set up
                print("Relay ON (Helmet detected)")

        else:  # No helmet detected
            if accelerometer_reading < 5:  # Vehicle is stationary
                print("No helmet and vehicle is stationary. Turning relay OFF.")
                
                # Only clean up relay if it was previously set up
                if relay_setup_done:
                    GPIO.cleanup()  # Clean up and turn off relay
                    relay_setup_done = False  # Mark relay as cleaned up
                    print("Relay OFF (No helmet detected, vehicle stationary)")
            
            elif accelerometer_reading >= 5:  # Vehicle is moving
                print("No helmet and vehicle is moving! Not turning off relay.")
                control_buzzer(1, 0.5)  # Buzz once for helmet not detected

        time.sleep(2)  # Delay before next loop iteration

# Main execution
try:
    run_helmet_detection()
except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()  # Clean up all GPIO on exit
    sys.exit(0)
