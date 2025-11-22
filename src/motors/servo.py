from rpi_hardware_pwm import HardwarePWM
import time
from src.obstacle_challenge import config


servo_pwm = None
_MAX_PWM_RETRIES = 5
_PWM_RETRY_DELAY = 0.1

def initialize():
    """Initializes the servo motor PWM with retries in case the kernel is busy."""
    global servo_pwm

    last_error = None
    for attempt in range(1, _MAX_PWM_RETRIES + 1):
        try:
            servo_pwm = HardwarePWM(
                pwm_channel=config.SERVO_PWM_CHANNEL,
                hz=config.SERVO_PWM_FREQ,
                chip=config.SERVO_PWM_CHIP,
            )
            servo_pwm.start(0)
            set_angle(0.0)
            time.sleep(0.5)
            print("INFO: Servo Initialized.")
            return True
        except PermissionError as err:
            last_error = err
            print(
                f"WARNING: Servo PWM permission denied (attempt {attempt}/{_MAX_PWM_RETRIES}): {err}"
            )
            time.sleep(_PWM_RETRY_DELAY * attempt)
        except Exception as err:
            last_error = err
            print(f"FATAL: Servo failed to initialize: {err}")
            time.sleep(_PWM_RETRY_DELAY * attempt)

    print(f"FATAL: Servo failed to initialize after {_MAX_PWM_RETRIES} attempts: {last_error}")
    return False


def set_angle(input_angle: float):
    """
    Sets the servo to a specific angle, RESPECTING the software limits
    in the config file (e.g., -45 to +45 degrees).
    """
    if servo_pwm is None:
        return

    clamped_input = max(
        config.INPUT_ANGLE_MIN_SERVO, min(config.INPUT_ANGLE_MAX_SERVO, input_angle)
    )
    clamped_input += 5
    input_range = config.INPUT_ANGLE_MAX_SERVO - config.INPUT_ANGLE_MIN_SERVO
    output_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    target_output_angle = (
        config.CALIBRATED_ANGLE_MIN
        + ((clamped_input - config.INPUT_ANGLE_MIN_SERVO) / input_range) * output_range
    )

    cal_angle_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    cal_pulse_range = config.CALIBRATED_MAX_PW_S - config.CALIBRATED_MIN_PW_S
    target_pulse_s = (
        config.CALIBRATED_MIN_PW_S
        + ((target_output_angle - config.CALIBRATED_ANGLE_MIN) / cal_angle_range)
        * cal_pulse_range
    )

    clamped_pw_s = max(
        config.SAFETY_MIN_PW_S, min(config.SAFETY_MAX_PW_S, target_pulse_s)
    )
    duty_cycle = (clamped_pw_s / config.SERVO_PWM_PERIOD_S) * 100.0
    servo_pwm.change_duty_cycle(max(0.0, min(100.0, duty_cycle)))


def set_angle_unlimited(input_angle: float):
    """
    Sets the servo to a specific angle, BYPASSING the software limits.
    This is for special maneuvers like parking that need sharper turns.
    It is still protected by the hardware safety pulse width limits.
    """
    if servo_pwm is None:
        return

    unclamped_input = input_angle

    input_range = config.INPUT_ANGLE_MAX_SERVO - config.INPUT_ANGLE_MIN_SERVO
    output_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    target_output_angle = (
        config.CALIBRATED_ANGLE_MIN
        + ((unclamped_input - config.INPUT_ANGLE_MIN_SERVO) / input_range)
        * output_range
    )

    cal_angle_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    cal_pulse_range = config.CALIBRATED_MAX_PW_S - config.CALIBRATED_MIN_PW_S
    target_pulse_s = (
        config.CALIBRATED_MIN_PW_S
        + ((target_output_angle - config.CALIBRATED_ANGLE_MIN) / cal_angle_range)
        * cal_pulse_range
    )

    clamped_pw_s = max(
        config.SAFETY_MIN_PW_S, min(config.SAFETY_MAX_PW_S, target_pulse_s)
    )
    duty_cycle = (clamped_pw_s / config.SERVO_PWM_PERIOD_S) * 100.0
    servo_pwm.change_duty_cycle(max(0.0, min(100.0, duty_cycle)))


def cleanup():
    """Centers the servo and stops PWM."""
    print("--- Cleaning up Servo ---")
    if servo_pwm:
        set_angle(0.0)
        time.sleep(0.5)
        servo_pwm.change_duty_cycle(0)
        time.sleep(4.0 / config.SERVO_PWM_FREQ)
        servo_pwm.stop()


if __name__ == "__main__":
    print("--- Testing Servo Module ---")
    if not initialize():
        print("Servo test failed during initialization.")
    else:
        try:
            while True:
                set_angle(0)
                # for angle in range(-90, 91, 1):
                #     set_angle_unlimited(angle)
                #     time.sleep(0.01)
                # for angle in range(90, -91, -1):
                #     set_angle_unlimited(angle)
                #     time.sleep(0.01)
                time.sleep(5)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
            set_angle(0)
            time.sleep(2)
        finally:
            cleanup()
