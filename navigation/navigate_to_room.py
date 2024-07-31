import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math

class NavigateToRoom(Node):
    def __init__(self):
        super().__init__('navigate_to_room')

        # Declare the parameter 'room' with a default value
        self.declare_parameter('room', 'ROOM A')  # Default room value

        # Publisher for sending navigation goals
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Subscription to the robot's odometry data
        qos_profile = QoSProfile(depth=10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile
        )
        self.current_position = None

    def odom_callback(self, msg):
        # Update the robot's current position
        self.current_position = msg.pose.pose

    def publish_goal(self, pose):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = pose
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f'Published goal pose: '
                               f'x={pose.position.x}, y={pose.position.y}, '
                               f'z={pose.position.z}, orientation={pose.orientation}')

    def set_to_origin(self):
        # Set the robot's initial pose to the origin
        origin_pose = PoseStamped()
        origin_pose.header.frame_id = 'map'
        origin_pose.header.stamp = self.get_clock().now().to_msg()
        origin_pose.pose.position.x = 0.0
        origin_pose.pose.position.y = 0.0
        origin_pose.pose.position.z = 0.0
        origin_pose.pose.orientation.w = 1.0
        
        # Publish the origin pose as a goal
        self.publish_goal(origin_pose.pose)

    def go_to_room(self):
        # Get the room parameter
        room = self.get_parameter('room').get_parameter_value().string_value

        # Define goal poses for different rooms
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        if room == 'ROOM A':
            goal_pose.pose.position.x = 2.0
            goal_pose.pose.position.y = 3.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
        elif room == 'ROOM B':
            goal_pose.pose.position.x = -1.0
            goal_pose.pose.position.y = -2.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
        elif room == 'ROOM C':
            goal_pose.pose.position.x = 4.0
            goal_pose.pose.position.y = -1.0
            goal_pose.pose.orientation.z = math.sin(math.radians(45 / 2))
            goal_pose.pose.orientation.w = math.cos(math.radians(45 / 2))
        else:
            self.get_logger().info(f'Unknown room: {room}')
            return

        # Set the robot to the origin
        self.set_to_origin()
        self.get_logger().info('Robot set to origin. Now navigating to the room.')

        # Navigate to the goal pose
        self.publish_goal(goal_pose.pose)

        # Wait until the robot reaches the goal
        self.get_logger().info('Waiting for the robot to reach the goal...')
        while not self.is_goal_reached(goal_pose.pose):
            rclpy.spin_once(self, timeout_sec=1.0)  # Spin once to process messages
        self.get_logger().info(f'Reached {room}. Returning to the origin.')

        # Return to origin
        self.set_to_origin()

    def is_goal_reached(self, goal_pose):
        if self.current_position is None:
            return False

        # Define a tolerance for reaching the goal
        tolerance = 0.08  # 10 cm tolerance

        # Calculate the distance to the goal
        dx = self.current_position.position.x - goal_pose.position.x
        dy = self.current_position.position.y - goal_pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        return distance < tolerance

def main(args=None):
    rclpy.init(args=args)

    node = NavigateToRoom()

    # If arguments are provided, use them
    if args and len(args) > 1:
        room = args[1]
        node.set_parameters([rclpy.parameter.Parameter('room', rclpy.Parameter.Type.STRING, room)])
    
    node.go_to_room()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

