import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for red color in HSV
    lower_red = np.array([0, 0, 0])
    upper_red = np.array([130, 255, 30])
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

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

