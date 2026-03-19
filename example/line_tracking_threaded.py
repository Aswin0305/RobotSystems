'''
    Line Following program for Picar-X:

    Pay attention to modify the reference value of the grayscale module 
    according to the practical usage scenarios.
    Auto calibrate grayscale values:
        Please run ./calibration/grayscale_calibration.py
    Manual modification:
        Use the following: 
            px.set_line_reference([1400, 1400, 1400])
        The reference value be close to the middle of the line gray value
        and the background gray value.

'''
import logging
import os
import sys
from time import sleep
from threading import Event
from concurrent.futures import ThreadPoolExecutor
from readerwriterlock import rwlock

# 1. Add the parent directory to the system path so Python can find the local 'picarx' folder
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

# 2. Now import specifically from the 'picarx_improved' file inside that folder
from picarx.picarx_improved import PicarxImproved
from robot_hat import ADC

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")

logging.getLogger().setLevel(logging.DEBUG)

# ===================== SENSOR =====================

class Sensor:
    def __init__(self):
        self.left = ADC(0)
        self.center = ADC(1)
        self.right = ADC(2)

    def read(self):
        return self.left.read(), self.center.read(), self.right.read()

# ===================== INTERPRETER =====================

class Interpreter:
    def __init__(self, threshold=250):
        self.threshold = threshold
        self.last_deviation = 0.0
        self.last_extreme = False
        self.last_direction = 0

    def process(self, left, center, right, polarity='dark'):
        """
        Returns:
          deviation ∈ [-1.0, 1.0]   → continuous (Ackermann)
          extreme   ∈ {True, False} → pivot required
          direction ∈ {-1, 0, 1}    → pivot direction
        """

        # --- Adjust for polarity ---
        if polarity == 'light':
            left, center, right = -left, -center, -right

        # --- Sensor state detection (dark = low value) ---
        left_on   = left   < center - self.threshold
        right_on  = right  < center - self.threshold
        center_on = center < (left + right) / 2 - self.threshold

        # --- Continuous deviation (for Ackermann steering) ---
        error = right - left
        deviation = error / (abs(left) + abs(right) + 1)
        deviation = max(-1.0, min(1.0, deviation))

        # --- Extreme deviation detection (for pivot turns) ---
        if left_on and not right_on:
            extreme = True
            direction = -1      # pivot left
        elif right_on and not left_on:
            extreme = True
            direction = 1       # pivot right
        else:
            extreme = False
            direction = 0

        # --- Line lost → memory-based recovery ---
        if not (left_on or center_on or right_on):
            deviation = self.last_deviation
            extreme = self.last_extreme
            direction = self.last_direction

        # --- Save state ---
        self.last_deviation = deviation
        self.last_extreme = extreme
        self.last_direction = direction

        return deviation, extreme, direction

# ===================== CONTROLLER =====================

class Controller:
    def __init__(self):
        self.px = PicarxImproved()
        self.max_steer = 30

    def control(self, deviation, extreme, direction):
        speed = 30

        if not extreme:
            # ---- Normal Ackermann steering ----
            steering = deviation * self.max_steer
            self.px.forward_ackerman(speed, steering)

        else:
            # ---- Sharp turn → pivot recovery ----
            self.px.pivot_turn(speed=35, direction=direction)

    def stop(self):
        self.px.stop()

class Bus:
    def __init__(self, initial_message=None):
        self.message = initial_message
        self.lock = rwlock.RWLockWriteD()

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        with self.lock.gen_rlock():
            return self.message

def handle_exception(future):
    exception = future.exception()
    if exception:
        print(f"Exception in worker thread: {exception}")

def sensor_producer(sensor, sensor_bus, delay, shutdown_event):
    logging.info("Sensor thread started")
    while not shutdown_event.is_set():
        values = sensor.read()
        sensor_bus.write(values)
        sleep(delay)
    logging.info("Sensor thread stopped")

def interpreter_consumer_producer(interpreter, sensor_bus, interpreter_bus, delay, shutdown_event):
    logging.info("Interpreter thread started")
    while not shutdown_event.is_set():
        data = sensor_bus.read()
        if data is not None:
            left, center, right = data
            result = interpreter.process(left, center, right)
            interpreter_bus.write(result)
        sleep(delay)
    logging.info("Interpreter thread stopped")

def controller_consumer(controller, interpreter_bus, delay, shutdown_event):
    logging.info("Controller thread started")
    while not shutdown_event.is_set():
        data = interpreter_bus.read()
        if data is not None:
            deviation, extreme, direction = data
            controller.control(deviation, extreme, direction)
        sleep(delay)
    logging.info("Controller thread stopped")

# ===================== MAIN LOOP =====================

if __name__ == "__main__":
    sensor = Sensor()
    interpreter = Interpreter(threshold=250)
    controller = Controller()

    # --- Message busses ---
    sensor_bus = Bus()
    interpreter_bus = Bus()

    # --- Loop rates (tune if I2C conflicts appear) ---
    SENSOR_DELAY = 0.02
    INTERPRETER_DELAY = 0.02
    CONTROLLER_DELAY = 0.02

    shutdown_event = Event()
    futures = []

    try:
        with ThreadPoolExecutor(max_workers=3) as executor:

            futures.append(executor.submit(
                sensor_producer,
                sensor,
                sensor_bus,
                SENSOR_DELAY,
                shutdown_event
            ))

            futures.append(executor.submit(
                interpreter_consumer_producer,
                interpreter,
                sensor_bus,
                interpreter_bus,
                INTERPRETER_DELAY,
                shutdown_event
            ))

            futures.append(executor.submit(
                controller_consumer,
                controller,
                interpreter_bus,
                CONTROLLER_DELAY,
                shutdown_event
            ))

            for f in futures:
                f.add_done_callback(handle_exception)

            logging.info("Line follower running (multithreaded)")

            while not shutdown_event.is_set():
                sleep(1)

    except KeyboardInterrupt:
        print("\nShutting down threads...")
        shutdown_event.set()

    finally:
        controller.stop()
        sleep(0.1)
        logging.info("Line follower stopped")