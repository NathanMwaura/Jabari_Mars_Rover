#!/usr/bin/env python3
"""
Test script to find available cameras
"""

import cv2
import os

def test_cameras():
    print("Testing camera access...")
    
    # Check /dev/video* devices
    video_devices = []
    for i in range(10):
        device_path = f'/dev/video{i}'
        if os.path.exists(device_path):
            video_devices.append(i)
    
    print(f"Found video devices: {video_devices}")
    
    # Test OpenCV camera access
    working_cameras = []
    for i in range(10):
        print(f"\nTesting camera index {i}...")
        cap = cv2.VideoCapture(i)
        
        if cap.isOpened():
            # Try to read a frame
            ret, frame = cap.read()
            if ret and frame is not None:
                height, width = frame.shape[:2]
                print(f"  ✓ Camera {i}: Working - {width}x{height}")
                working_cameras.append(i)
                
                # Get camera properties
                fps = cap.get(cv2.CAP_PROP_FPS)
                fourcc = cap.get(cv2.CAP_PROP_FOURCC)
                print(f"    FPS: {fps}, FOURCC: {fourcc}")
            else:
                print(f"  ✗ Camera {i}: Opens but no frame")
        else:
            print(f"  ✗ Camera {i}: Cannot open")
        
        cap.release()
    
    print(f"\nWorking cameras: {working_cameras}")
    return working_cameras

def test_camera_detailed(camera_index):
    print(f"\nDetailed test for camera {camera_index}:")
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"Cannot open camera {camera_index}")
        return False
    
    # Test multiple frames
    for i in range(5):
        ret, frame = cap.read()
        if ret:
            print(f"Frame {i}: {frame.shape}")
        else:
            print(f"Frame {i}: Failed")
    
    cap.release()
    return True

if __name__ == "__main__":
    # Test all cameras
    working_cameras = test_cameras()
    
    # Test the first working camera in detail
    if working_cameras:
        test_camera_detailed(working_cameras[0])
        print(f"\nRecommended camera index: {working_cameras[0]}")
    else:
        print("\nNo working cameras found!")
        print("Troubleshooting suggestions:")
        print("1. Check if camera is being used by another application")
        print("2. Check camera permissions")
        print("3. Try unplugging and reconnecting external cameras")
        print("4. Check if camera drivers are installed")