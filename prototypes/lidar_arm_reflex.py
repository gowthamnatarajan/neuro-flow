import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition
import numpy as np

class LidarArmReflex(Node):
    def __init__(self):
        super().__init__('lidar_arm_reflex')

        # 1. Use Best Effort QoS to match high-speed sensor data
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # Subscribers & Publishers
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.publisher_ = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)

        # State Variables
        self.target_pulse = 500  # Default to center
        self.min_dist_threshold = 0.15  # 15cm (ignore self)
        self.max_dist_threshold = 1.5   # 1.5 meters

        # 2. The "Heartbeat" Timer (25Hz) - This mimics the App's smooth motion
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.get_logger().info('Lidar Arm Reflex Node Started')

    def lidar_callback(self, msg):
        # Convert scan to numpy array for fast processing
        ranges = np.array(msg.ranges)
        
        # Filter out noise and self-detection
        ranges[ranges < self.min_dist_threshold] = np.inf
        ranges[ranges > self.max_dist_threshold] = np.inf

        if np.all(np.isinf(ranges)):
            return # No valid targets in range

        # Find the closest point index
        closest_idx = np.argmin(ranges)
        
        # Calculate angle (in radians) of the closest point
        # In ROS, 0 radians is usually straight ahead
        angle = msg.angle_min + (closest_idx * msg.angle_increment)
        
        # 3. Map angle to Servo Pulse (Approx: -90 deg = 200, +90 deg = 800)
        # Adjust these coefficients based on your hardware testing
        degree_angle = np.degrees(angle)
        
        # Simple linear mapping: Center (0 deg) is 500 pulse. 
        # Servo range is 0-1000 for approx 240 degrees (60 to 300 degrees).
        self.target_pulse = 500 + (degree_angle * 4.166) 
        
        # Clamp to safe physical limits
        self.target_pulse = max(min(self.target_pulse, 850), 150)

    def timer_callback(self):
        # The heartbeat ensures the arm doesn't "fall asleep" or stutter
        msg = ServosPosition()
        msg.duration = 40.0 / 1000.0 # Match the 25Hz rate (40ms)
        
        pos = ServoPosition()
        pos.id = 1 # The base servo
        pos.position = int(self.target_pulse)

        pos4 = ServoPosition()
        pos4.id = 4
        pos4.position = 300
        
        msg.position = [pos, pos4]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarArmReflex()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()