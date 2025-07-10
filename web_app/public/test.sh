#!/usr/bin/env python3
import cv2
import os
import subprocess
import sys
import glob

def run_command(cmd):
    """Run a shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip(), result.stderr.strip()
    except Exception as e:
        return "", str(e)

def check_opencv_info():
    """Check OpenCV build information"""
    print("=== OpenCV Information ===")
    print(f"OpenCV Version: {cv2.__version__}")
    print(f"OpenCV Build Info:")
    print(cv2.getBuildInformation())
    print()

def check_system_info():
    """Check system information"""
    print("=== System Information ===")
    
    # Check OS
    stdout, stderr = run_command("uname -a")
    if stdout:
        print(f"System: {stdout}")
    
    # Check video devices
    print("\n--- Video Devices ---")
    video_devices = glob.glob("/dev/video*")
    if video_devices:
        for device in sorted(video_devices):
            stdout, stderr = run_command(f"ls -la {device}")
            print(f"{device}: {stdout}")
    else:
        print("No /dev/video* devices found")
    
    # Check user groups
    print("\n--- User Groups ---")
    stdout, stderr = run_command("groups")
    if stdout:
        print(f"Current user groups: {stdout}")
        if "video" in stdout:
            print("✓ User is in 'video' group")
        else:
            print("✗ User is NOT in 'video' group")
    
    # Check processes using video
    print("\n--- Processes Using Video Devices ---")
    stdout, stderr = run_command("lsof /dev/video* 2>/dev/null")
    if stdout:
        print(f"Processes using video devices:\n{stdout}")
    else:
        print("No processes currently using video devices")
    
    print()

def check_camera_backends():
    """Test different camera backends"""
    print("=== Testing Camera Backends ===")
    
    backends = [
        (cv2.CAP_V4L2, "V4L2"),
        (cv2.CAP_GSTREAMER, "GStreamer"),
        (cv2.CAP_FFMPEG, "FFmpeg"),
        (cv2.CAP_ANY, "Any")
    ]
    
    for backend_id, backend_name in backends:
        print(f"\n--- Testing {backend_name} Backend ---")
        for i in range(3):  # Test first 3 indices
            try:
                cap = cv2.VideoCapture(i, backend_id)
                if cap.isOpened():
                    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                    fps = cap.get(cv2.CAP_PROP_FPS)
                    print(f"  ✓ Camera {i}: {width}x{height} @ {fps} FPS")
                    cap.release()
                else:
                    print(f"  ✗ Camera {i}: Cannot open")
            except Exception as e:
                print(f"  ✗ Camera {i}: Exception - {e}")

def test_camera_properties():
    """Test camera properties for any working cameras"""
    print("\n=== Testing Camera Properties ===")
    
    found_working = False
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            found_working = True
            print(f"\n--- Camera {i} Properties ---")
            
            properties = [
                (cv2.CAP_PROP_FRAME_WIDTH, "Width"),
                (cv2.CAP_PROP_FRAME_HEIGHT, "Height"),
                (cv2.CAP_PROP_FPS, "FPS"),
                (cv2.CAP_PROP_FOURCC, "FOURCC"),
                (cv2.CAP_PROP_BRIGHTNESS, "Brightness"),
                (cv2.CAP_PROP_CONTRAST, "Contrast"),
                (cv2.CAP_PROP_SATURATION, "Saturation"),
                (cv2.CAP_PROP_HUE, "Hue"),
            ]
            
            for prop_id, prop_name in properties:
                try:
                    value = cap.get(prop_id)
                    print(f"  {prop_name}: {value}")
                except Exception as e:
                    print(f"  {prop_name}: Error - {e}")
            
            # Try to read a frame
            ret, frame = cap.read()
            if ret:
                print(f"  ✓ Successfully read frame: {frame.shape}")
            else:
                print(f"  ✗ Failed to read frame")
            
            cap.release()
    
    if not found_working:
        print("No working cameras found to test properties")

def main():
    print("Enhanced Camera Diagnostic Tool")
    print("=" * 50)
    
    check_opencv_info()
    check_system_info()
    check_camera_backends()
    test_camera_properties()
    
    print("\n=== Recommendations ===")
    print("If no cameras are working, try:")
    print("1. Add user to video group: sudo usermod -a -G video $USER")
    print("2. Install v4l-utils: sudo apt install v4l-utils")
    print("3. Check camera permissions: sudo chmod 666 /dev/video*")
    print("4. Restart your system after adding to video group")
    print("5. Try different USB ports or cables")
    print("6. Check if camera works in other applications (cheese, guvcview)")
    print("7. Update camera drivers or kernel modules")

if __name__ == "__main__":
    main()