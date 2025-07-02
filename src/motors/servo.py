# servo.py
from rpi_hardware_pwm import HardwarePWM
import time
import config

# Module-level hardware object
servo_pwm = None

def initialize():
    """Initializes the servo motor PWM."""
    global servo_pwm
    try:
        servo_pwm = HardwarePWM(
            pwm_channel=config.SERVO_PWM_CHANNEL,
            hz=config.SERVO_PWM_FREQ,
            chip=config.SERVO_PWM_CHIP
        )
        servo_pwm.start(0)
        set_angle(0.0) # Start at center
        time.sleep(0.5)
        print("INFO: Servo Initialized.")
        return True
    except Exception as e:
        print(f"FATAL: Servo failed to initialize: {e}")
        return False

def set_angle(input_angle: float):
    """Sets the servo to a specific angle (-45 to +45 degrees)."""
    if servo_pwm is None: return

    clamped_input = max(config.INPUT_ANGLE_MIN_SERVO, min(config.INPUT_ANGLE_MAX_SERVO, input_angle))
    
    # Map input angle range to calibrated physical angle range
    input_range = config.INPUT_ANGLE_MAX_SERVO - config.INPUT_ANGLE_MIN_SERVO
    output_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    target_output_angle = config.CALIBRATED_ANGLE_MIN + ((clamped_input - config.INPUT_ANGLE_MIN_SERVO) / input_range) * output_range
    
    # Map physical angle range to calibrated pulse width range
    cal_angle_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    cal_pulse_range = config.CALIBRATED_MAX_PW_S - config.CALIBRATED_MIN_PW_S
    target_pulse_s = config.CALIBRATED_MIN_PW_S + ((target_output_angle - config.CALIBRATED_ANGLE_MIN) / cal_angle_range) * cal_pulse_range
    
    clamped_pw_s = max(config.SAFETY_MIN_PW_S, min(config.SAFETY_MAX_PW_S, target_pulse_s))
    duty_cycle = (clamped_pw_s / config.SERVO_PWM_PERIOD_S) * 100.0
    servo_pwm.change_duty_cycle(max(0.0, min(100.0, duty_cycle)))

def cleanup():
    """Centers the servo and stops PWM."""
    print("--- Cleaning up Servo ---")
    if servo_pwm:
        set_angle(0.0)
        time.sleep(0.5)
        servo_pwm.stop()

# Test routine
if __name__ == "__main__":
    print("--- Testing Servo Module ---")
    if not initialize():
        print("Servo test failed during initialization.")
    else:
        try:
            print("Sweeping servo. Press Ctrl+C to exit.")
            while True:
                for angle in range(-45, 46, 5):
                    print(f"\rSetting angle to: {angle}°   ", end="")
                    set_angle(angle)
                    time.sleep(0.05)
                time.sleep(0.5)
                for angle in range(45, -46, -5):
                    print(f"\rSetting angle to: {angle}°   ", end="")
                    set_angle(angle)
                    time.sleep(0.05)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
