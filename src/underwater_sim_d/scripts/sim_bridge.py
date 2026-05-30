
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from underwater_sim_d.msg import Thrust, DVL, IMU
import math

class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        self.mass = 1.0
        self.iz = 0.5
        self.c = 0.5
        self.cT = 0.3
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.sub_thrust = self.create_subscription(Thrust, 'thrust_cmd', self.thrust_callback, 10)
        self.pub_dvl = self.create_publisher(DVL, 'dvl', 10)
        self.pub_imu = self.create_publisher(IMU, 'imu', 10)
        self.pub_pose = self.create_publisher(Pose2D, 'turtle_pose', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('SimBridge started')

    def thrust_callback(self, msg):
        F = msg.thrust_forward
        tau = msg.torque_yaw
        now = self.get_clock().now().seconds_nanoseconds()[0]
        dt = now - self.last_time
        self.last_time = now
        if dt > 0.1:
            dt = 0.02
        ax = (F - self.c * self.vx * abs(self.vx)) / self.mass
        ay = 0.0
        alpha = (tau - self.cT * self.omega * abs(self.omega)) / self.iz
        self.vx += ax * dt
        self.vy += ay * dt
        self.omega += alpha * dt
        self.x += (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        self.y += (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        self.theta += self.omega * dt

    def timer_callback(self):
        dvl_msg = DVL()
        dvl_msg.vx = self.vx
        dvl_msg.vy = self.vy
        self.pub_dvl.publish(dvl_msg)
        imu_msg = IMU()
        imu_msg.ax = 0.0
        imu_msg.ay = 0.0
        imu_msg.omega_z = self.omega
        self.pub_imu.publish(imu_msg)
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pub_pose.publish(pose_msg)
        self.get_logger().info(f'Pos: ({self.x:.2f}, {self.y:.2f}) | vx={self.vx:.2f} | omega={self.omega:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
