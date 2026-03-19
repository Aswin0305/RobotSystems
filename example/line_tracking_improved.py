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

# 1. Add the parent directory to the system path so Python can find the local 'picarx' folder
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

# 2. Now import specifically from the 'picarx_improved' file inside that folder
from picarx.picarx_improved import PicarxImproved
from robot_hat import ADC

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")

logging.getLogger().setLevel(logging.DEBUG)

# px = Picarx(grayscale_pins=['A0', 'A1', 'A2'])

# Please run ./calibration/grayscale_calibration.py to Auto calibrate grayscale values
# or manual modify reference value by follow code
# px.set_line_reference([1400, 1400, 1400])

# class Sensor:
#     def __init__(self):
#         self.adc0 = ADC(0)
#         self.adc1 = ADC(1)
#         self.adc2 = ADC(2)
#         logging.info("Sensor initialized")

#     def read_sensor(self):
#         return [self.adc0.read(), self.adc1.read(), self.adc2.read()]
        

# class Interpreter:
#     def __init__(self, sensitivity = 300, polarity = 'dark'):
#         self.sensitivity = sensitivity
#         self.polarity = polarity
#         self.current_position = 0.0
#         self.current_state = None
#         self.last_position = 0.0
#         self.last_state = None
#         logging.info(f"Interpreter initialized: sensitivity={sensitivity}, polarity={polarity}")

#     def processing(self, sensor_values):
#         left, center, right = sensor_values
        
#         left_err = left - center
#         right_err = right - center

#         if self.polarity == 'light':
#             left_err = -left_err
#             right_err = -right_err

#         if abs(left_err) > self.sensitivity and abs(right_err) > self.sensitivity:
#             self.last_position = self.current_position
#             self.last_state = self.current_state
#             self.current_position = 0.0 #move forward
#             self.current_state = 'forward'
#         elif abs(left_err) > self.sensitivity:
#             if left_err > 0:
#                 self.last_position = self.current_position
#                 self.last_state = self.current_state
#                 self.current_position = left_err / (abs(left_err) + abs(right_err) + 1) #robot position: left, move right
#                 self.current_position = max(-1.0, min(1.0, self.current_position))
#                 self.current_state = 'forward_right'
#             else:
#                 self.last_position = self.current_position
#                 self.last_state = self.current_state
#                 self.current_position = -left_err / (abs(left_err) + abs(right_err) + 1)#robot position: right, move left
#                 self.current_position = max(-1.0, min(1.0, self.current_position))
#                 self.current_state = 'forward_left'
#         elif abs(right_err) > self.sensitivity:
#             if right_err > 0:
#                 self.last_position = self.current_position
#                 self.last_state = self.current_state
#                 self.current_position = -right_err / (abs(left_err) + abs(right_err) + 1)#robot position: right, move left 
#                 self.current_position = max(-1.0, min(1.0, self.current_position))
#                 self.current_state = 'forward_left'
#             else:
#                 self.last_position = self.current_position
#                 self.last_state = self.current_state
#                 self.current_position = right_err / (abs(left_err) + abs(right_err) + 1) #robot position: left, move right
#                 self.current_position = max(-1.0, min(1.0, self.current_position))
#                 self.current_state = 'forward_right'
#         else:
#             # if self.last_state == 'forward_left':
#             #     self.last_state = self.current_state
#             #     prev_last_position = self.last_position
#             #     self.last_position = self.current_position
#             #     self.current_position = -prev_last_position
#             #     self.current_state = 'forward_right'
#             # elif self.last_state == 'forward_right':
#             #     self.last_state = self.current_state
#             #     prev_last_position = self.last_position
#             #     self.last_position = self.current_position
#             #     self.current_position = -prev_last_position
#             #     self.current_state = 'forward_left'
#             # elif self.last_state == 'forward':
#             #     self.last_state = self.current_state
#             #     prev_last_position = self.last_position
#             #     self.last_position = self.current_position
#             #     self.current_position = -prev_last_position
#             #     self.current_state = 'backward'
#             # elif self.last_state == 'forward':
#             #     self.last_state = self.current_state
#             #     prev_last_position = self.last_position
#             #     self.last_position = self.current_position
#             #     self.current_position = -prev_last_position
#             #     self.current_state = 'backward'
#             # elif self.last_state == 'backward_left' or self.last_state == 'backward_right' or self.last_state == 'backward':
#             #     self.last_state = self.current_state
#             #     self.last_position = self.current_position
#             #     self.current_position = 0.0
#             #     self.current_state = 'forward'
#             pass

#     def get_current_position(self):
#         return self.current_position

#     def get_current_state(self):
#         return self.current_state

# class Controller:
#     def __init__(self, scaling_factor=30):
#         self.scaling_factor = scaling_factor
#         self.px = PicarxImproved()
#         logging.info(f"Controller initialized: scaling_factor={scaling_factor}")  

#     def control(self, position):
#         steering_angle = position * self.scaling_factor
#         # steering_angle = max(-30, min(30, steering_angle))
#         # self.px.set_dir_servo_angle(steering_angle)

#         return steering_angle

#     def stop(self):
#         self.px.stop()

# if __name__=='__main__':
#     sensor = Sensor()
#     interpreter = Interpreter(sensitivity=250, polarity='dark')
#     controller = Controller(scaling_factor=30)
#     try:
#         while True:
#             speed = 30
#             sensor_values = sensor.read_sensor()
#             logging.debug(f"Sensor values: {sensor_values}")

#             interpreter.processing(sensor_values)
#             position = interpreter.get_current_position()
#             state = interpreter.get_current_state()
#             logging.debug(f"Interpreted position: {position}, state: {state}")

#             steering_angle = controller.control(position)
#             logging.debug(f"Steering angle set to: {steering_angle}")
#             if 'backward' in state:
#                 speed = -30

#             # controller.px.forward(speed)
#             # controller.px.forward_ackerman_revised(speed, steering_angle)
#             controller.px.forward_ackerman(speed, steering_angle)


#             logging.debug(f"Sensors: {sensor_values}, Position: {position:.3f}, Steering: {steering_angle:.1f}°, Speed: {speed}, State: {state}")
            
#             sleep(0.01)
#     except KeyboardInterrupt:
#         print("\nKeyboardInterrupt: stop and exit")

#     finally:
#         controller.stop()
#         logging.info("Line follower stopped")
#         sleep(0.1)

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


# ===================== MAIN LOOP =====================

if __name__ == "__main__":
    sensor = Sensor()
    interpreter = Interpreter(threshold=250)
    controller = Controller()

    try:
        while True:
            left, center, right = sensor.read()
            deviation, extreme, direction = interpreter.process(left, center, right)
            controller.control(deviation, extreme, direction)

            logging.debug(
                f"L:{left} C:{center} R:{right} | "
                f"dev:{deviation:.2f} extreme:{extreme} dir:{direction}"
            )

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping line follower")

    finally:
        controller.stop()
        sleep(0.1)
        logging.info("Line follower stopped")