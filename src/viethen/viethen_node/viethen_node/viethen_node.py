import os
import time

from viethen_node.cad_handler import CADHandler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
from moveit_msgs.srv import GetPlanningScene
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Quaternion, Point
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ViethenNode(Node):

    def __init__(self):
        super().__init__('viethen_node')
        self.webots_supervisor = self.create_publisher(String, '/Ros2Supervisor/remove_node', 10)
        self.mesh_publisher = self.create_publisher(Marker, '/workpiece', 10)
        # Define some ANSI escape codes for colors
        red = "\033[91m"
        green = "\033[92m"
        yellow = "\033[93m"
        blue = "\033[94m"
        magenta = "\033[95m"
        cyan = "\033[96m"
        reset = "\033[0m"
        self.declare_parameter('launch_simulation', False)
        self.declare_parameter('use_cad_file', True)
        self.declare_parameter('cad_path', "spoiler.stl")
        self.declare_parameter('cad_frame_id', "spoiler_link")
        self.declare_parameter('cad_parrent_id', "base_link")
        self.declare_parameter('cad_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        launch_simulation = self.get_parameter('launch_simulation').value
        use_cad_file = self.get_parameter('use_cad_file').value
        self.cad_path = self.get_parameter('cad_path').value
        self.cad_frame_id = self.get_parameter('cad_frame_id').value
        self.cad_parrent_id = self.get_parameter('cad_parrent_id').value
        cad_pose = self.get_parameter('cad_pose').get_parameter_value().double_array_value
        self.x, self.y, self.z, self.yaw, self.pitch, self.roll = cad_pose

        # Create colored log messages
        self.get_logger().info(f'{green}Viethen Manager Started!{reset}')
        if launch_simulation:
            self.get_logger().info(f'{yellow}Waiting for the Simulation to be launched!{reset}')
            self.wait_for_subscriber(self.webots_supervisor)
            self.get_logger().info(f'{green}Webots launch finished!{reset}')
        else:
            self.get_logger().warn(f'{yellow}Launched with real robot! (make sure the robot is connected and moveit is configured){reset}')
        self.get_logger().info(f'{yellow}Wait for Moveit to launch!{reset}')
        self.wait_for_service("/get_planning_scene", GetPlanningScene)

        if use_cad_file:
            self.cad_absolute_path = os.path.join(get_package_share_directory('viethen_node'), "models", self.cad_path)
            self.cad_path = f'package://viethen_node/models/{self.cad_path}'
            self.get_logger().info(f'{yellow}Launched with cad-file: {self.cad_path}{reset}')
            self.tf_broadcaster = TransformBroadcaster(self)
            self.timer = self.create_timer(1.0, self.broadcast_transform)
            self.timer2 = self.create_timer(1.0, self.publish_stl)
            self.get_logger().info(f'{blue}In rviz you should the the cad file!{reset}')
            self.cad = CADHandler(self.cad_absolute_path)
            self.get_logger().info(f'{self.cad.get_properties()}')
            up_polygons = self.cad.count_polygons_above_angle()
            total_polygons = self.cad.num_cells
            self.get_logger().info(f'{blue}{up_polygons} of {total_polygons}{reset} are facing up ({(up_polygons/total_polygons)*100}%)')
        else:
            self.get_logger().error(f'{red}Launched with visual scan instead. THIS HAS NOT BEEN IMPLEMENTED YET (TODO){reset}')


        self.get_logger().error(f'{red}This is a red error{reset}')
        self.get_logger().info(f'{blue}This is a blue info message{reset}')
        self.get_logger().debug(f'{magenta}This is a magenta debug message{reset}')

    def publish_stl(self):
        marker = Marker()
        marker.header.frame_id = self.cad_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workpiece'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # Set the scale, pose, and mesh resource
        marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
        marker.pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        marker.mesh_resource = self.cad_path
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.mesh_publisher.publish(marker)

    def broadcast_transform(self):
        # Create the transform message
        t = TransformStamped()

        # Set the time and frame IDs
        t.header.stamp = self.get_clock().now().to_msg()
        current_time = self.get_clock().now()
        t.header.frame_id = self.cad_parrent_id
        t.child_frame_id = self.cad_frame_id

        # Set the translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        # Convert Euler angles (YPR) to a quaternion using transforms3d
        quat = euler2quat(self.roll, self.pitch, self.yaw, axes='sxyz')

        # Set the rotation
        t.transform.rotation.x = quat[1]  # X component of quaternion
        t.transform.rotation.y = quat[2]  # Y component of quaternion
        t.transform.rotation.z = quat[3]  # Z component of quaternion
        t.transform.rotation.w = quat[0]  # W component of quaternion

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def wait_for_subscriber(self, publisher):
        while publisher.get_subscription_count() == 0:
            self.get_logger().info(f'waiting...')
            time.sleep(1.0)

    def wait_for_service(self, service_name, service_type):
        client = self.create_client(service_type, service_name)

        # Keep checking until the service is available
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting...')

def main(args=None):
    rclpy.init(args=args)

    viethen_node = ViethenNode()

    rclpy.spin(viethen_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    viethen_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
