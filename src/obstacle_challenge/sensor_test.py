import time
from src.sensors import distance

def test_sensor_loop():
    print("--- VL53L8CX Continuous Test Loop ---")
    print("Initializing Distance manager...")
    
    distance.initialise()
    print("\nStarting loop (Ctrl+C to stop)...")
    print("Note: Detailed debug logs from distance.py and vl53l8cx_python.py should appear below.\n")
    
    try:
        while True:
            start_time = time.monotonic()
            
            # This mimics what SensorThread does
            print(f"--- Loop Start (Time: {time.time():.2f}) ---")
            
            # We specifically test channel -1 (Back Sensor / VL53L8CX)
            val = distance.get_distance(-1)
            elapsed = time.monotonic() - start_time
            print(f"Loop End. Result: {val} mm. Time taken: {elapsed:.4f}s\n")
            
            # Sleep to match the ~60Hz rate or the thread sleep
            # SensorThread sleeps 0.02s
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    except Exception as e:
        print(f"\nTest crashed: {e}")
    finally:
        distance.cleanup()

if __name__ == "__main__":
    test_sensor_loop()
