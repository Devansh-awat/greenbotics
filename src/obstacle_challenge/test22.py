# ...existing code...
from src.sensors import distance
import threading
import time
from src.sensors import bno055, distance

class SensorThread(threading.Thread):
    def __init__(self, bno, dist, init_event):
        super().__init__()
        self.bno = bno
        self.dist = dist
        self.initialization_complete = init_event
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True
        self.heading = None
        self.distance_left = None
        self.distance_right = None
        self.distance_back = None
        self.distance_center = None

    def run(self):
        try:
            for attempt in range(3):
                try:
                    print("SensorThread: Initializing IMU...")
                    self.bno.initialize()
                    print("SensorThread: Initializing distance sensors...")
                    self.dist.initialise()
                    print("SensorThread: Both sensors initialized.")
                    time.sleep(0.3)
                    print("SensorThread: Initialization complete flag set.")
                    break
                except Exception as e:
                    print(f"SensorThread: ERROR during initialization: {e}")
                    import traceback
                    traceback.print_exc()
                    return
                finally:
                    print("SensorThread: Setting initialization complete event.")
                    self.initialization_complete.set()
            time.sleep(0.5)

            consecutive_none = {ch: 0 for ch in distance._sensors.keys()}
            reinit_threshold = 4
            while not self.stop_event.is_set():
                try:
                    heading = self.bno.get_heading()
                    readings = {}
                    for ch in sorted(self.dist._sensors.keys()):
                        val = self.dist.get_distance(ch)
                        readings[ch] = val
                        if val is None:
                            consecutive_none[ch] = consecutive_none.get(ch, 0) + 1
                        else:
                            consecutive_none[ch] = 0

                    with self.lock:
                        self.heading = heading
                        self.distance_left = readings.get(0)
                        self.distance_center = readings.get(2)
                        self.distance_right = readings.get(3)
                        self.distance_back = readings.get(-1)

                    for ch, count in list(consecutive_none.items()):
                        if count >= reinit_threshold:
                            ok = distance.reinit_sensor(ch)
                            consecutive_none[ch] = 0 if ok else count

                    time.sleep(0.02) # account for timing budget 
                except Exception as e:
                    print(f"SensorThread: ERROR during sensor reading: {e}")
                    import traceback
                    traceback.print_exc()
                    time.sleep(0.1)
        
        except Exception as e:
            print(f"SensorThread: ERROR during initialization/operation: {e}")
            import traceback
            traceback.print_exc()
            # Still set the event so main thread doesn't hang forever
            self.initialization_complete.set()
        
        finally:
            print("SensorThread: Cleaning up distance sensors...")
            self.dist.cleanup()
            self.bno.cleanup()
            print("SensorThread: Distance sensor cleanup complete.")

    def get_readings(self):
        with self.lock:
            return {
                'heading': self.heading,
                'distance_left': self.distance_left,
                'distance_center': self.distance_center,
                'distance_right': self.distance_right,
                'distance_back' : self.distance_back
            }

    def stop(self):
        self.stop_event.set()

if __name__ == "__main__":
    sensors_initialized_event = threading.Event()
    sensor_thread = SensorThread(bno055, distance, sensors_initialized_event)
    sensor_thread.start()
    print("MainThread: Waiting for sensors to initialize...")
    sensors_initialized_event.wait() 
    print("MainThread: Sensors are ready. Proceeding with main logic.")  
    time.sleep(1)
    for _ in range(40000):
        d0 = sensor_thread.get_readings()['distance_left']
        d3 = sensor_thread.get_readings()['distance_right']
        d2 = sensor_thread.get_readings()['distance_center']
        db = sensor_thread.get_readings()['distance_back']
        print(f"SensorThread: Distance readings - Left: {d0}, Center: {d2}, Right: {d3}, Back: {db}")
