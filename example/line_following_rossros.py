'''
Line Following program for Picar-X with RossROS + Ultrasonic Safety
'''

import logging
import os
import sys
from time import sleep

# ---------- Path setup ----------
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from picarx.picarx_improved import PicarxImproved
from robot_hat import ADC
from robot_hat.modules import Ultrasonic
import rossros as rr

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

has_obstacle = False

# ===================== SENSOR =====================

class Sensor:
    def __init__(self):
        self.left = ADC(0)
        self.center = ADC(1)
        self.right = ADC(2)

    def read(self):
        print("READING GREYSCALE SENSORS...")
        return (self.left.read(), self.center.read(), self.right.read())

# ===================== INTERPRETER =====================

class Interpreter:
    def __init__(self, threshold=250):
        self.threshold = threshold
        self.last_deviation = 0.0
        self.last_extreme = False
        self.last_direction = 0

    def process(self, left, center, right, polarity='dark'):
        if polarity == 'light':
            left, center, right = -left, -center, -right

        left_on   = left   < center - self.threshold
        right_on  = right  < center - self.threshold
        center_on = center < (left + right) / 2 - self.threshold

        error = right - left
        deviation = error / (abs(left) + abs(right) + 1)
        deviation = max(-1.0, min(1.0, deviation))

        if left_on and not right_on:
            extreme = True
            direction = -1
        elif right_on and not left_on:
            extreme = True
            direction = 1
        else:
            extreme = False
            direction = 0

        if not (left_on or center_on or right_on):
            deviation = self.last_deviation
            extreme = self.last_extreme
            direction = self.last_direction

        self.last_deviation = deviation
        self.last_extreme = extreme
        self.last_direction = direction

        print("LINE INTERPRETER OUTPUT:", deviation, direction, extreme)

        return (deviation, extreme, direction)

# ===================== CONTROLLER =====================

class Controller:
    def __init__(self, px):
        self.px = px
        self.max_steer = 30

    def control(self, deviation, extreme, direction):
        if not has_obstacle:
            speed = 30

            if not extreme:
                steering = deviation * self.max_steer
                self.px.forward_ackerman(speed, steering)
            else:
                self.px.pivot_turn(speed=35, direction=direction)
            print("LINE CONTROLLER OUTPUT:", deviation, direction, extreme)


    def stop(self):
        self.px.stop()

# ===================== ULTRASONIC =====================

class UltrasonicSensor:
    def __init__(self, px):
        self.px = px

    def read(self):
        print("READING ULTRASONIC SENSOR...")
        return round(self.px.ultrasonic.read(), 2)

class UltrasonicInterpreter:
    def __init__(self, stop_distance=10):
        self.stop_distance = stop_distance

    def process(self, distance):
        logging.info("Ultrasonic distance: %.2f cm", distance)
        if distance in (-1, -2):
            return False
        print("ULTRASONIC INTERPRETER OUTPUT:", distance < self.stop_distance)
        return distance < self.stop_distance

class SafetyController:
    def __init__(self, px):
        self.px = px

    def control(self, obstacle_detected):
        if obstacle_detected:
            self.px.stop()
        global has_obstacle
        has_obstacle = obstacle_detected
        print("SAFETY CONTROLLER OUTPUT:", obstacle_detected)


# ===================== MAIN =====================

if __name__ == "__main__":

    px = PicarxImproved()

    sensor = Sensor()
    interpreter = Interpreter(threshold=250)
    controller = Controller(px)

    ultra_sensor = UltrasonicSensor(px)
    ultra_interpreter = UltrasonicInterpreter(stop_distance=15)
    safety_controller = SafetyController(px)

    # ---------- Busses ----------
    termination_bus = rr.Bus(False, "Termination Bus")

    grayscale_bus = rr.Bus((0,0,0), "Grayscale Bus")
    line_result_bus = rr.Bus((0,0,0), "Line Result Bus")

    ultra_distance_bus = rr.Bus(100, "Ultrasonic Distance Bus")
    obstacle_bus = rr.Bus(False, "Obstacle Bus")

    # ---------- Line Following Pipeline ----------
    grayscale_producer = rr.Producer(
        sensor.read,
        grayscale_bus,
        0.02,
        termination_bus,
        "Grayscale Producer"
    )

    line_interpreter_cp = rr.ConsumerProducer(
        lambda data: interpreter.process(*data),
        grayscale_bus,
        line_result_bus,
        0.02,
        termination_bus,
        "Line Interpreter"
    )

    line_controller_consumer = rr.Consumer(
        lambda data: controller.control(*data),
        line_result_bus,
        0.02,
        termination_bus,
        "Line Controller"
    )

    # ---------- Ultrasonic Pipeline ----------
    ultra_producer = rr.Producer(
        ultra_sensor.read,
        ultra_distance_bus,
        0.05,
        termination_bus,
        "Ultrasonic Producer"
    )

    ultra_interpreter_cp = rr.ConsumerProducer(
        ultra_interpreter.process,
        ultra_distance_bus,
        obstacle_bus,
        0.05,
        termination_bus,
        "Ultrasonic Interpreter"
    )

    safety_consumer = rr.Consumer(
        safety_controller.control,
        obstacle_bus,
        0.02,
        termination_bus,
        "Safety Controller"
    )

    # ---------- Timer ----------
    timer = rr.Timer(
        termination_bus,
        60,
        0.1,
        termination_bus,
        "Shutdown Timer"
    )

    # ---------- Run ----------
    logging.info("Running line follower with ultrasonic safety")

    try:
        rr.runConcurrently([
            grayscale_producer,
            line_interpreter_cp,
            line_controller_consumer,
            ultra_producer,
            ultra_interpreter_cp,
            safety_consumer,
            timer
        ])
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received, shutting down...")
        termination_bus.set_message(True)
    finally:
        controller.stop()
        sleep(0.1)
        logging.info("Stopped")