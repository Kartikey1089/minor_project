import cv2
import numpy as np

# Conversion factors
PIXELS_PER_MM_X = 3.2  # Pixels per mm in X direction
PIXELS_PER_MM_Y = 3.2  # Pixels per mm in Y direction

# Function to find contours and calculate centroid
def get_centroid_and_angle(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            rect = cv2.minAreaRect(largest_contour)
            angle = rect[2]
            if angle < -45:
                angle = 90 + angle
            return (cx, cy), angle
    return None, None

# Define color ranges in HSV
color_ranges = {
    "red": [(0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)],  # Two ranges for red
    "green": [(36, 100, 100), (86, 255, 255)],
    "blue": [(94, 80, 2), (126, 255, 255)]
}

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color, ranges in color_ranges.items():
        if color == "red":
            mask1 = cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))
            mask2 = cv2.inRange(hsv, np.array(ranges[2]), np.array(ranges[3]))
            mask = mask1 | mask2
        else:
            mask = cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        (centroid, angle) = get_centroid_and_angle(mask)

        if centroid:
            cx, cy = centroid
            
            # Convert pixel coordinates to millimeters
            real_x = cx / PIXELS_PER_MM_X
            real_y = cy / PIXELS_PER_MM_Y

            cv2.circle(frame, (cx, cy), 5, (0, 0, 0), -1)
            cv2.putText(frame, f"{color.capitalize()} Box: ({real_x:.1f} mm, {real_y:.1f} mm), Angle: {angle:.2f}", 
                        (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Box Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
