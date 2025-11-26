# Greenbotics Control Software (`src`)

This folder contains all Python code that runs on the Raspberry Pi 5 to control the Greenbotics robot for the WRO Future Engineers 2025 Open and Obstacle challenges. This code is split into smaller modules for sensors, motors and high-level challenge logic.

## Layout

- `open_challenge/` – high-level logic for the Open Challenge (`main.py`).
- `obstacle_challenge/` – high-level logic for the Obstacle Challenge(`main_v3.py`).
- `sensors/` – drivers/wrappers for the IMU (BNO055), distance sensors (VL53L1X / VL53L8CX / URM09) and camera (Picamera2 + OpenCV).
- `motors/` – drive motor and steering (servo) control.

## Python dependencies

The control software uses the following third‑party Python libraries:

- `numpy` – numerical processing for vision and sensor fusion.
- `opencv-python` / `python3-opencv` – image processing and computer vision.
- `gpiozero` and `lgpio` – GPIO access and motor driver control.
- `rpi-hardware-pwm` – hardware PWM for the drive motor and steering servo.
- `picamera2` and `libcamera` – camera access on Raspberry Pi.
- `adafruit-blinka` – provides `board`/`busio` for CircuitPython drivers on Linux SBCs.
- `adafruit-circuitpython-bno055` – IMU (orientation) sensor driver.
- `adafruit-circuitpython-tca9548a` – I2C multiplexer driver.
- `adafruit-circuitpython-vl53l1x` – VL53L1X ToF distance sensor driver.
- `adafruit-circuitpython-busdevice` – shared I2C device helpers.
- `adafruit-circuitpython-neopixel-spi` – SPI NeoPixel (WS2812-compatible) LED control used via `neopixel_spi`.

Standard-library modules such as `time`, `threading`, `ctypes`, `json`, `traceback`, etc. are also used but do not require separate installation.

## How to install dependencies (Raspberry Pi OS)

On a Raspberry Pi (recommended environment), first install the system-level packages:

```bash
sudo apt update
sudo apt install -y \
  python3 python3-pip python3-opencv python3-numpy \
  python3-gpiozero python3-libcamera python3-picamera2 python3-lgpio
```

Then install the Python libraries that are typically provided via `pip`:

```bash
pip3 install \
  numpy opencv-python gpiozero lgpio rpi-hardware-pwm \
  adafruit-blinka adafruit-circuitpython-bno055 \
  adafruit-circuitpython-tca9548a adafruit-circuitpython-vl53l1x \
  adafruit-circuitpython-busdevice adafruit-circuitpython-neopixel-spi
```

On non-Raspberry-Pi Linux systems, you can generally install the same Python packages with `pip3`, but `picamera2`/`libcamera` support and GPIO access may require additional board-specific setup.

## Running the control software

From the repository root, run the main programs using the `src` package:

```bash
cd /path/to/greenbotics

# Obstacle Challenge
python3 -m src.obstacle_challenge.main_v3

# Open Challenge
python3 -m src.open_challenge.main
```

Make sure all hardware is wired as described in the top-level `README.md` and that the commands above are executed on the robot’s Raspberry Pi.
