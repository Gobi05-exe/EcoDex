#!/usr/bin/env python3

import cv2
from ultralytics import YOLO

# Load the custom YOLO model
model = YOLO("/home/ayush3112/Desktop/EcoDex/raged_ws/src/raged_pkg/scripts/best.pt")  # Replace with your custom model path

# Load the video stream or image
cap = cv2.VideoCapture(3)  # Use 0 for webcam or replace with video/image path

while True:
    # Read a frame from the video
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame. Exiting...")
        break

    # Perform YOLO inference on the frame
    results = model(frame)

    # Extract detections
    for result in results:
        for box in result.boxes:
            # Extract bounding box coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
            conf = box.conf[0]                     # Confidence score
            cls = int(box.cls[0])                  # Class ID
            class_name = result.names[cls]         # Class name

            # Draw the bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Add label with class name and confidence
            label = f"{class_name} {conf:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Show the frame with bounding boxes
    cv2.imshow("Detections", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
