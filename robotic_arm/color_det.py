import cv2
import numpy as np

# Define color ranges in HSV
color_ranges = {
    'black': ([0, 0, 0], [180, 255, 50]),
    'blue': ([100, 150, 0], [140, 255, 255]),
    'green': ([40, 70, 70], [80, 255, 255]),
    'red1': ([0, 120, 70], [10, 255, 255]),
    'red2': ([170, 120, 70], [180, 255, 255])
}

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    detected_colors = []

    for color, (lower, upper) in color_ranges.items():
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_np, upper_np)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                # Draw rectangle around the detected object
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Print the center of the object
                cx, cy = x + w // 2, y + h // 2
                print(f"Center of the {color} object: ({cx}, {cy})")

                if color.startswith('red'):
                    detected_colors.append('red')
                else:
                    detected_colors.append(color)
    
    detected_colors = list(set(detected_colors))  # Remove duplicates
    print(f"Detected colors: {', '.join(detected_colors)}")

    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
