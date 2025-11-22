import os
import time
from rpi_hardware_pwm import HardwarePWM

SERVO_PWM_CHIP = 0
SERVO_PWM_CHANNEL = 2
SERVO_PWM_FREQ = 50

def force_unexport():
    pwm_path = f"/sys/class/pwm/pwmchip{SERVO_PWM_CHIP}/pwm{SERVO_PWM_CHANNEL}"
    unexport_path = f"/sys/class/pwm/pwmchip{SERVO_PWM_CHIP}/unexport"
    
    if os.path.exists(pwm_path):
        try:
            with open(unexport_path, "w") as f:
                f.write(str(SERVO_PWM_CHANNEL))
        except OSError:
            pass
        time.sleep(0.01) 

print(f"--- Stress Testing Servo PWM {SERVO_PWM_CHIP}/{SERVO_PWM_CHANNEL} ---")
print("Press Ctrl+C to stop.")

count = 0
while True:
    count += 1
    
    force_unexport()

    try:
        pwm = HardwarePWM(pwm_channel=SERVO_PWM_CHANNEL, 
                          hz=SERVO_PWM_FREQ, 
                          chip=SERVO_PWM_CHIP)
        
        pwm.start(0)
        pwm.stop() 
        
        print(f"\rAttempt {count}: OK", end="")
        
    except PermissionError:
        print(f"\n\n[!!!] CAUGHT THE BUG on Attempt {count}")
        break
        
    except Exception as e:
        print(f"\nOther error: {e}")
        break