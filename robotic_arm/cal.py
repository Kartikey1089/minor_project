import cv2

# Start video capture from the camera
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame
    ret, frame = cap.read()
    if not ret:
        break

    # Get the dimensions of the frame
    height, width = frame.shape[:2]

    # Display the frame dimensions in pixels on the image
    cv2.putText(frame, f"Width: {width} pixels", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Height: {height} pixels", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Show the frame in a window
    cv2.imshow("Camera Frame", frame)

    # Press 'q' to quit the loop and close the window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
