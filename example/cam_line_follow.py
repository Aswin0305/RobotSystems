import logging
import os
import sys
from time import sleep
from picamera2 import Picamera2
import cv2

# Add parent directory to system path for picarx import
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_dir)

from picarx.picarx_improved import PicarxImproved, constrain

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

# -------------------- TUNING CONSTANTS --------------------
CAM_PAN_MAX = 30
CAM_PAN_GAIN = 35
STEER_GAIN = 1.0
STEER_MAX = 30

BASE_SPEED = 35
PIVOT_SPEED = 22

CENTER_THRESH = 0.15
PAN_SAT_THRESH = 28
LOST_LINE_TIMEOUT = 0.3


class Sensor:
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "BGR888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        logging.info("Camera initialized")

    def read(self):
        return self.picam2.capture_array()


class Interpreter:
    def __init__(self, polarity='dark'):
        self.polarity = polarity
        self.last_seen_dir = 0
        self.last_seen_time = 0

    def process(self, frame):
        h, w, _ = frame.shape
        roi = frame[int(h * 0.6):, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        if self.polarity == 'dark':
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        else:
            _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        error = cx - (w // 2)
        pos = constrain(error / (w // 2), -1.0, 1.0)

        self.last_seen_dir = 1 if pos > 0 else -1
        self.last_seen_time = time.time()

        return pos


class Controller:
    def __init__(self):
        self.px = PicarxImproved()
        self.px.set_cam_tilt_angle_without_constrain(-50)
        self.cam_pan = 0.0

    def stop(self):
        self.px.stop()

    def update_camera(self, pos):
        target_pan = CAM_PAN_GAIN * pos
        self.cam_pan = constrain(target_pan, -CAM_PAN_MAX, CAM_PAN_MAX)
        self.px.set_cam_pan_angle(self.cam_pan)

    def drive(self, pos, last_seen_dir):
        steer = STEER_GAIN * self.cam_pan
        steer = constrain(steer, -STEER_MAX, STEER_MAX)

        # -------- SHARP TURN LOGIC --------
        if abs(self.cam_pan) >= PAN_SAT_THRESH and abs(pos) > CENTER_THRESH:
            logging.warning("Camera saturated → pivot to recover line")
            self.px.stop()
            sleep(0.02)
            self.px.pivot_turn(last_seen_dir, PIVOT_SPEED)
            return

        self.px.forward_ackerman_revised(BASE_SPEED, steer)


if __name__ == "__main__":
    import time

    sensor = Sensor()
    interpreter = Interpreter(polarity='dark')
    controller = Controller()

    try:
        while True:
            frame = sensor.read()
            pos = interpreter.process(frame)

            # ---------- LINE LOST ----------
            if pos is None:
                if time.time() - interpreter.last_seen_time < LOST_LINE_TIMEOUT:
                    controller.px.pivot_turn(interpreter.last_seen_dir, PIVOT_SPEED)
                else:
                    controller.stop()
                sleep(0.05)
                continue

            # ---------- NORMAL TRACK ----------
            controller.update_camera(pos)
            controller.drive(pos, interpreter.last_seen_dir)

            logging.debug(
                f"pos={pos:.3f} cam_pan={controller.cam_pan:.1f}"
            )

            sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping")

    finally:
        controller.stop()