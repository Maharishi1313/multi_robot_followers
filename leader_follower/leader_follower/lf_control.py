import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from math import sqrt
from rclpy.qos import QoSProfile, DurabilityPolicy

class LeaderFollowerControl(Node):
    def __init__(self):
        super().__init__('lf_control')
        
        # Declare parameters for namespaces
        self.declare_parameter('leader_namespace', 'tb1')
        self.declare_parameter('follower_namespace', 'tb2')
        self.declare_parameter('threshold_distance',2)  # Threshold in meters
        
        # Get namespaces and threshold
        self.leader_namespace = self.get_parameter('leader_namespace').get_parameter_value().string_value
        self.follower_namespace = self.get_parameter('follower_namespace').get_parameter_value().string_value
        self.threshold_distance = self.get_parameter('threshold_distance').get_parameter_value().double_value

        qos_profile = QoSProfile(
            # history=10,  # Keep up to 10 messages in history
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Transient Local durability
            depth = 10
        )
        
        # Subscribers to leader and follower amcl_pose topics
        self.leader_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.leader_namespace}/amcl_pose',
            self.leader_pose_callback,
            qos_profile
        )
        
        self.follower_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.follower_namespace}/amcl_pose',
            self.follower_pose_callback,
            qos_profile
        )
        
        # Publisher to send velocity commands to follower
        self.follower_cmd_vel_publisher = self.create_publisher(Twist, f'/{self.follower_namespace}/cmd_vel', 10)
        
        # Action Client to send goal to follower (nav2)
        self.follower_goal_publisher = self.create_publisher(PoseStamped, f'/{self.follower_namespace}/goal_pose', 10)

        self.client = ActionClient(self, NavigateToPose, f'/{self.follower_namespace}/navigate_to_pose')  # Change namespace if needed

        # Initialize robot poses
        self.leader_pose = None
        self.follower_pose = None
        self.future_goal_handle = None  
    
    def leader_pose_callback(self, msg):
        self.get_logger().info(f'subscribed to leader pose: /{self.leader_namespace}/amcl_pose')
        self.leader_pose = msg.pose.pose
        self.check_distance_and_control_follower()
    
    def follower_pose_callback(self, msg):
        self.get_logger().info(f'subscribed to follower pose: /{self.follower_namespace}/amcl_pose')
        self.follower_pose = msg.pose.pose
        self.check_distance_and_control_follower()
    
    def calculate_distance(self, pose1, pose2):
        # Euclidean distance calculation
        x1, y1 = pose1.position.x, pose1.position.y
        x2, y2 = pose2.position.x, pose2.position.y
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def check_distance_and_control_follower(self):
        if self.leader_pose and self.follower_pose:
            # Calculate Euclidean distance between leader and follower
            distance = self.calculate_distance(self.leader_pose, self.follower_pose)
            self.get_logger().info(f'Distance between leader and follower: {distance} meters')

            if distance > self.threshold_distance:
                # Publish goal to follower (navigate to leader's pose)
                self.send_follower_goal(self.leader_pose)
            else:
                # Stop the follower robot (publish zero velocity)
                self.stop_follower()

    def send_follower_goal(self, leader_pose):
        # Create a goal message for the follower (move towards the leader's pose)
        goal_msg = NavigateToPose.Goal()

        # Fill the goal message with the leader's pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'  # Adjust the frame ID as needed
        goal_msg.pose.pose = leader_pose

        # Send the goal using the action client
        self.get_logger().info("Sending goal to follower via ActionClient...")
        self.future_goal_handle = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Add a callback for when the goal is completed
        # self.future_goal_handle.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the action server
        self.get_logger().info(f"Received feedback: {feedback_msg.feedback}")
    
    def stop_follower(self):
        
        goal_msg = NavigateToPose.Goal()

        # Fill the goal message with the follower's pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'  # Adjust the frame ID as needed
        goal_msg.pose.pose = self.follower_pose

        # Send the goal using the action client
        self.get_logger().info("Sending goal to follower via ActionClient...")
        self.future_goal_handle = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
