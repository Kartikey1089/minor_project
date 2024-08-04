import cv2
import numpy as np
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize servo on channel 0
servo_motor = servo.Servo(pca.channels[0])

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for darker colors in HSV
    lower_dark = np.array([0, 0, 0])       # Lower bound of HSV for dark colors
    upper_dark = np.array([180, 255, 50])  # Upper bound of HSV for dark colors
    mask = cv2.inRange(hsv, lower_dark, upper_dark)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            # Draw rectangle around the detected dark object
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Print the center of the object
            cx, cy = x + w//2, y + h//2
            print(f"Center of the dark object: ({cx}, {cy})")

            # Map the x coordinate of the object to the servo angle
            frame_center = frame.shape[1] / 2
            angle = 90 + (cx - frame_center) * (90 / frame_center)  # Map to range [0, 180]
            angle = max(0, min(180, angle))  # Ensure angle is within [0, 180]
            servo_motor.angle = angle

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
