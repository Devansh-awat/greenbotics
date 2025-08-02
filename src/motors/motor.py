# motor.py
import lgpio
from rpi_hardware_pwm import HardwarePWM
import time
from src.obstacle_challenge import config

# Module-level hardware objects
gpio_handle = None
motor_pwm = None


def initialize():
    """Initializes the DC motor driver and PWM."""
    global gpio_handle, motor_pwm
    try:
        gpio_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(gpio_handle, config.AIN1_PIN)
        lgpio.gpio_claim_output(gpio_handle, config.AIN2_PIN)
        lgpio.gpio_claim_output(gpio_handle, config.STBY_PIN)

        standby()

        motor_pwm = HardwarePWM(
            pwm_channel=config.MOTOR_PWM_CHANNEL,
            hz=config.MOTOR_PWM_FREQ,
            chip=config.MOTOR_PWM_CHIP,
        )
        motor_pwm.start(0)
        print("INFO: Motor Initialized.")
        return True
    except Exception as e:
        print(f"FATAL: Motor failed to initialize: {e}")
        return False


def _set_speed(speed):
    """Internal function to set motor PWM duty cycle."""
    if motor_pwm:
        motor_pwm.change_duty_cycle(max(0, min(100, speed)))


def forward(speed):
    """Drives the motor forward at a given speed."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 0)
        _set_speed(speed)

def reverse(speed):
    """Drives the motor in reverse at a given speed."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, 0)  # This is the change
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 1)  # This is the change
        _set_speed(speed)

def standby():
    """Puts the motor driver in standby mode (low power, disengaged)."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 0)

def brake():
    """Brakes the motor by shorting its terminals."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 1)
        _set_speed(0) # Set speed to 0 when braking

def cleanup():
    """Stops the motor and releases GPIO resources."""
    print("--- Cleaning up Motor ---")
    if motor_pwm:
        forward(0)
        standby()
        motor_pwm.stop()
    if gpio_handle:
        lgpio.gpiochip_close(gpio_handle)


# Test routine
if __name__ == "__main__":
    print("--- Testing Motor Module ---")
    if not initialize():
        print("Motor test failed during initialization.")
    else:
        try:
            print("Motor forward at 50% for 2 seconds...")
            forward(50)
            time.sleep(2)

            print("Motor forward at 100% for 2 seconds...")
            forward(100)
            time.sleep(2)

            print("Putting motor in standby.")
            standby()
            time.sleep(1)

            print("Motor test complete.")

        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
