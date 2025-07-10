#!/usr/bin/env python3
import cv2
import sys

def main():
    # Initialize camera (use index 0 as confirmed working)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return -1
    
    # Set camera properties (optional)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print("Camera opened successfully!")
    print("Press 'q' to quit, 's' to save frame, 'i' for info")
    
    frame_count = 0
    
    while True:
        # Capture frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture frame")
            break
        
        frame_count += 1
        
        # Add frame counter to image
        cv2.putText(frame, f"Frame: {frame_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display frame
        cv2.imshow('Camera Feed', frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("Quitting...")
            break
        elif key == ord('s'):
            filename = f"captured_frame_{frame_count}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Frame saved as {filename}")
        elif key == ord('i'):
            height, width, channels = frame.shape
            print(f"Frame info: {width}x{height}, {channels} channels")
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Camera released and windows closed")
    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)