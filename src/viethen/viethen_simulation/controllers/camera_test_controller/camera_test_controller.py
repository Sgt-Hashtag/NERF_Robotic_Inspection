#!/usr/bin/env python3

"""
Simple Webots controller to test camera functionality
Place this in your Webots controllers directory
"""

from controller import Robot, Camera
import cv2
import numpy as np

def main():
    # Initialize robot
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    print(f"Robot name: {robot.getName()}")
    print(f"Number of devices: {robot.getNumberOfDevices()}")
    
    # List all available devices
    print("Available devices:")
    for i in range(robot.getNumberOfDevices()):
        device = robot.getDeviceByIndex(i)
        print(f"  - {device.getName()} (type: {device.getNodeType()})")
    
    # Try to get camera device
    camera = robot.getDevice('arm_camera')
    if camera is None:
        print("ERROR: Camera 'arm_camera' not found!")
        print("Trying alternative camera names...")
        
        # Try common camera names
        camera_names = ['camera', 'Camera', 'arm_camera', 'camera_link', 'camera0']
        for name in camera_names:
            test_camera = robot.getDevice(name)
            if test_camera is not None:
                print(f"Found camera with name: {name}")
                camera = test_camera
                break
    
    if camera is None:
        print("No camera device found!")
        return
    
    # Enable camera
    camera.enable(timestep)
    
    print(f"Camera enabled: {camera.getWidth()}x{camera.getHeight()}")
    print("Camera FOV:", camera.getFov())
    print("Camera near:", camera.getNear())
    # print("Camera far:", camera.getFar())
    
    # Wait a few simulation steps for camera to initialize
    for i in range(10):
        robot.step(timestep)
    
    # Main loop
    frame_count = 0
    while robot.step(timestep) != -1:
        frame_count += 1
        
        image = camera.getImage()
        if image:
            print("Got image")
        else:
            print("No image received yet")

        # Get camera image
        image_array = camera.getImageArray()
        
        if image_array is not None and len(image_array) > 0:
            # Convert to numpy array
            height = len(image_array)
            width = len(image_array[0]) if height > 0 else 0
            
            if height > 0 and width > 0:
                # Convert Webots image to OpenCV format
                np_image = np.array(image_array, dtype=np.uint8)
                np_image = np.flipud(np_image)  # Flip vertically
                
                # Check if image is completely black
                max_value = np.max(np_image)
                min_value = np.min(np_image)
                mean_value = np.mean(np_image)
                
                if frame_count % 50 == 0:  # Status update every 50 frames
                    print(f"Frame {frame_count}: Image stats - Min: {min_value}, Max: {max_value}, Mean: {mean_value:.2f}")
                
                # Save occasional frames for testing
                if frame_count % 100 == 0:  # Every 100 frames
                    filename = f"camera_test_frame_{frame_count}.png"
                    
                    # Convert BGRA to BGR if needed
                    if len(np_image.shape) == 3 and np_image.shape[2] == 4:
                        bgr_image = np_image[:, :, :3]
                    else:
                        bgr_image = np_image
                    
                    cv2.imwrite(filename, bgr_image)
                    print(f"Saved {filename} - Size: {bgr_image.shape}, Stats: min={min_value}, max={max_value}")
                    
                    # If image is black, print more debug info
                    if max_value < 10:
                        print("WARNING: Image appears to be black!")
                        print(f"Camera position might need adjustment")
            else:
                print(f"Frame {frame_count}: Empty image dimensions: {width}x{height}")
        else:
            print(f"Frame {frame_count}: No image data received")
    
    print("Controller finished")

if __name__ == "__main__":
    main()