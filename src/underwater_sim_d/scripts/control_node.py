
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from underwater_sim_d.msg import Thrust, DVL, IMU
import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.pub_thrust = self.create_publisher(Thrust, 'thrust_cmd', 10)
        self.sub_dvl = self.create_subscription(DVL, 'dvl', self.dvl_callback, 10)
        self.sub_imu = self.create_subscription(IMU, 'imu', self.imu_callback, 10)
        self.sub_mode = self.create_subscription(String, 'control_mode', self.mode_callback, 10)
        self.sub_goal = self.create_subscription(Pose2D, 'goto_goal', self.goal_callback, 10)

        self.sub_pose = self.create_subscription(Pose2D, 'turtle_pose', self.pose_callback, 10)
        self.vx = 0.0
        self.omega = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.mode = 'idle'
        self.target_x = None
        self.target_y = None
        self.threshold = 0.1
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Control node ready. Topics: /control_mode (speed/circle/idle), /goto_goal (Pose2D)')

    def dvl_callback(self, msg):
        self.vx = msg.vx
    def imu_callback(self, msg):
        self.omega = msg.omega_z
    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
    def mode_callback(self, msg):
        cmd = msg.data
        if cmd == 'speed':
            self.mode = 'speed'
            self.target_x = None
            self.get_logger().info('Speed mode')
        elif cmd == 'circle':
            self.mode = 'circle'
            self.target_x = None
            self.get_logger().info('Circle mode')
        elif cmd == 'idle':
            self.mode = 'idle'
            self.target_x = None
            self.get_logger().info('Idle mode')
        else:
            self.get_logger().warn(f'Unknown mode: {cmd}')
    def goal_callback(self, msg):
        self.mode = 'position'
        self.target_x = msg.x
        self.target_y = msg.y
        self.get_logger().info(f'Go to ({msg.x:.2f}, {msg.y:.2f})')
    def control_loop(self):
        if self.mode == 'position' and self.target_x is not None:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            dist = math.hypot(dx, dy)
            if dist < self.threshold:
                self.pub_thrust.publish(Thrust(thrust_forward=0.0, torque_yaw=0.0))
                self.mode = 'idle'
                self.target_x = None
                self.get_logger().info('Target reached')
                return
            target_theta = math.atan2(dy, dx)
            err_theta = target_theta - self.theta
            err_theta = math.atan2(math.sin(err_theta), math.cos(err_theta))
            thrust = 1.5 * dist
            torque = 1.2 * err_theta
            thrust = max(-3.0, min(3.0, thrust))
            torque = max(-2.0, min(2.0, torque))
            self.pub_thrust.publish(Thrust(thrust_forward=thrust, torque_yaw=torque))
        elif self.mode == 'speed':
            error = 0.5 - self.vx
            thrust = 2.0 * error
            thrust = max(-3.0, min(3.0, thrust))
            self.pub_thrust.publish(Thrust(thrust_forward=thrust, torque_yaw=0.0))
        elif self.mode == 'circle':
            self.pub_thrust.publish(Thrust(thrust_forward=1.5, torque_yaw=0.8))
        else:
            self.pub_thrust.publish(Thrust(thrust_forward=0.0, torque_yaw=0.0))

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
