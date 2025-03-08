import cv2

# Open the second webcam (default cam is usually index 0)
cap = cv2.VideoCapture(1)  # Change index if the camera is not detected

if not cap.isOpened():
    print("Error: Could not open the second camera.")
    exit()

print("Streaming from second webcam... Press 'q' to exit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break

        cv2.imshow("Second Webcam Stream", frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stream stopped by user.")

# Release resources
cap.release()
cv2.destroyAllWindows()
