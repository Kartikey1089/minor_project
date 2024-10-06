import cv2
import numpy as np

# Conversion factors (as per the previous calibration)
PIXELS_PER_MM_X = 3.2  # Pixels per mm in X direction
PIXELS_PER_MM_Y = 3.2  # Pixels per mm in Y direction

# Function to find contours and calculate the bounding box and area
def get_bounding_box_and_area(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get bounding box around the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Calculate area in pixels
        area_pixels = w * h
        
        # Convert width and height to real-world millimeters
        width_mm = w / PIXELS_PER_MM_X
        height_mm = h / PIXELS_PER_MM_Y
        
        # Calculate real-world area in square millimeters
        area_mm2 = width_mm * height_mm
        
        return (x, y, w, h), area_pixels, area_mm2
    return None, None, None

# Define color ranges in HSV
color_ranges = {
    "red": [(0, 120, 70), (10, 255, 255), (170, 120, 70), (180, 255, 255)],  # Two ranges for red
    "green": [(36, 100, 100), (86, 255, 255)],
    "blue": [(94, 80, 2), (126, 255, 255)],
    "brown": [(10, 100, 20), (20, 255, 200)],  # Brown range as discussed earlier
    "black": [(0, 0, 0), (180, 255, 50)]       # Black range as discussed earlier
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

        (bbox, area_pixels, area_mm2) = get_bounding_box_and_area(mask)

        if bbox:
            x, y, w, h = bbox
            # Draw the bounding box on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Display the area in both pixels and millimeters squared
            cv2.putText(frame, f"Area: {area_pixels} px^2 / {area_mm2:.1f} mm^2", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Show the frame in a window
    cv2.imshow("Object Detection with Bounding Box", frame)

    # Press 'q' to quit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
