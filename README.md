# Vehicle Helmet Detection Using Raspberry Pi & TFLite

This project detects whether a rider is wearing a helmet using a Raspberry Pi and a TensorFlow Lite model. It also integrates an MPU6050 accelerometer to check if the vehicle is moving. If a rider is detected without a helmet while the vehicle is moving, an alert buzzer sounds.

## Features
- **Helmet Detection** using a TFLite model and Raspberry Pi camera.
- **Vehicle Motion Detection** using an MPU6050 accelerometer.
- **Relay Control** to enforce helmet usage before vehicle ignition.
- **Buzzer Alert** when the vehicle is moving without helmet detection.

## Hardware Requirements
- **Raspberry Pi 4 / 3B+** (Any model with GPIO support)
- **Raspberry Pi Camera Module**
- **MPU6050 Accelerometer Module**
- **5V Relay Module**
- **Buzzer**
- **Connecting Wires**

## Circuit Connections
| Component        | Raspberry Pi Pin |
|-----------------|----------------|
| **Buzzer**      | GPIO 18 (Pin 12) |
| **Relay**       | GPIO 17 (Pin 11) |
| **MPU6050 SDA** | GPIO 2 (Pin 3) |
| **MPU6050 SCL** | GPIO 3 (Pin 5) |
| **MPU6050 VCC** | 3.3V (Pin 1) |
| **MPU6050 GND** | GND (Pin 6) |

## Software Setup

### 1. Install Required Libraries
Ensure your Raspberry Pi is updated:
```sh
sudo apt update && sudo apt upgrade -y
```

Then, install the required Python packages:
```sh
pip install -r requirements.txt
```

### 2. Enable I2C
Run:
```sh
sudo raspi-config
```
- Navigate to `Interfacing Options`
- Enable `I2C`
- Reboot the Pi: `sudo reboot`

### 3. Run the Helmet Detection Script
After setting up the hardware, run the detection script:
```sh
python helmet_new.py
```

## Expected Output
- If a helmet is detected, the relay remains ON.
- If no helmet is detected and the vehicle is moving, a buzzer alert sounds.
- If no helmet is detected but the vehicle is stationary, the relay turns OFF.

## Notes
- Ensure the model file `helmet.tflite` is placed in the same directory as `helmet_detection.py`.
- You can modify the relay behavior based on your requirements.

## License
This project is open-source under the MIT License.
Authors:
Rajeev Sekar 
Abinaya Raghuraman 

