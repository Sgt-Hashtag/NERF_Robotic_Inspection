#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

try:
    from controller import Robot, Camera
except ImportError:
    print("Warning: Webots Python API not available. Camera functionality disabled.")

class WebootsCameraNode(Node):
    def __init__(self):
        super().__init__('webots_camera_node')
        
        # Colors for logging
        self.green = "\033[92m"
        self.yellow = "\033[93m"
        self.blue = "\033[94m"
        self.red = "\033[91m"
        self.reset = "\033[0m"
        
        self.get_logger().info(f'{self.green}Webots Camera Node started!{self.reset}')
        
        # Camera ROS2 publishers
        self.image_pub = self.create_publisher(
            Image, 
            '/arm_camera/image_raw', 
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 
            '/arm_camera/camera_info', 
            10
        )
        
        # Service for capturing images
        self.capture_service = self.create_service(
            Trigger,
            '/camera/capture_image',
            self.capture_image_callback
        )
        
        # Initialize Webots camera
        self.camera = None
        self.robot = None
        self.image_counter = 0
        
        try:
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
            self.camera = self.robot.getDevice('arm_camera')
            
            if self.camera is not None:
                self.camera.enable(self.timestep)
                self.width = self.camera.getWidth()
                self.height = self.camera.getHeight()
                self.fov = self.camera.getFov()
                
                self.get_logger().info(f'{self.blue}Camera enabled: {self.width}x{self.height}, FOV: {self.fov}{self.reset}')
                
                # Create camera info
                self.camera_info_msg = self.create_camera_info()
                
                # Timer for camera updates
                self.camera_timer = self.create_timer(self.timestep / 1000.0, self.update_camera)
            else:
                self.get_logger().warn(f'{self.yellow}Camera device not found{self.reset}')
                
        except Exception as e:
            self.get_logger().warn(f'{self.yellow}Webots not available: {e}{self.reset}')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        self.get_logger().info(f'{self.green}Webots Camera Node ready!{self.reset}')

    def create_camera_info(self):
        """Create camera info message"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_optical_frame"
        camera_info.width = self.width
        camera_info.height = self.height
        
        # Calculate camera matrix from FOV
        fx = fy = self.width / (2.0 * np.tan(self.fov / 2.0))
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return camera_info

    def update_camera(self):
        """Update Webots camera and publish ROS2 messages"""
        if self.robot is None or self.camera is None:
            return
            
        if self.robot.step(self.timestep) == -1:
            return
            
        # Get camera image
        image_array = self.camera.getImageArray()
        if image_array is None or len(image_array) == 0:
            return
            
        try:
            height = len(image_array)
            width = len(image_array[0]) if height > 0 else 0
            
            if height == 0 or width == 0:
                return
                
            # Convert to numpy and flip
            np_image = np.array(image_array, dtype=np.uint8)
            np_image = np.flipud(np_image)
            
            # Convert BGRA to RGB
            if len(np_image.shape) == 3 and np_image.shape[2] == 4:
                bgr_image = np_image[:, :, :3]
                rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            else:
                rgb_image = np_image
            
            # Convert to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")
            current_time = self.get_clock().now().to_msg()
            ros_image.header.stamp = current_time
            ros_image.header.frame_id = "camera_optical_frame"
            
            # Publish
            self.image_pub.publish(ros_image)
            
            # Publish camera info
            self.camera_info_msg.header.stamp = current_time
            self.camera_info_pub.publish(self.camera_info_msg)
            
        except Exception as e:
            self.get_logger().error(f"Camera processing error: {e}")

    def capture_image_callback(self, request, response):
        """Service callback to capture and save image"""
        filename = self.capture_and_save_image()
        if filename:
            response.success = True
            response.message = f"Image saved as {filename}"
            self.get_logger().info(f'{self.green}Image capture service: {filename}{self.reset}')
        else:
            response.success = False
            response.message = "Failed to capture image"
            self.get_logger().error(f'{self.red}Image capture service failed{self.reset}')
        
        return response

    def capture_and_save_image(self):
        """Capture and save current camera image"""
        if self.camera is None:
            self.get_logger().warn("No camera available for capture")
            return None
            
        try:
            image_array = self.camera.getImageArray()
            if image_array is None or len(image_array) == 0:
                return None
                
            # Convert to OpenCV format
            np_image = np.array(image_array, dtype=np.uint8)
            np_image = np.flipud(np_image)
            
            if len(np_image.shape) == 3 and np_image.shape[2] == 4:
                bgr_image = np_image[:, :, :3]
            else:
                bgr_image = np_image
            
            # Save high-quality image
            filename = f"capture_{self.image_counter:04d}.png"
            cv2.imwrite(filename, bgr_image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            
            self.image_counter += 1
            return filename
            
        except Exception as e:
            self.get_logger().error(f'{self.red}Image capture failed: {e}{self.reset}')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WebootsCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()