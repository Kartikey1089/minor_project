import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO
servo_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz PWM frequency
pwm.start(7.5)  # Initial duty cycle (neutral position)

# Function to set servo angle
def set_servo_angle(angle):
    duty = 2.5 + (angle / 18)
    pwm.ChangeDutyCycle(duty)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for red color in HSV
    lower_red = np.array([0, 0, 0])
    upper_red = np.array([180, 255, 50])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            # Draw rectangle around the detected box
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Print the center of the box
            cx, cy = x + w//2, y + h//2
            print(f"Center of the box: ({cx}, {cy})")
            
            # Map the x coordinate of the box to the servo angle
            frame_center = frame.shape[1] / 2
            angle = 90 + (cx - frame_center) * (90 / frame_center)  # Map to range [0, 180]
            angle = max(0, min(180, angle))  # Ensure angle is within [0, 180]
            set_servo_angle(angle)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
pwm.stop()
GPIO.cleanup()
