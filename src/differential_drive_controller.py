#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import JointState
import math
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.42)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        
        # Variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.last_time = self.get_clock().now()
        
        # QoS profiles
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )
        
        # Publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', odom_qos)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing joint states and odometry
        self.timer = self.create_timer(0.1, self.update_odometry)
        
        self.get_logger().info("Differential Drive Controller started")

    def cmd_vel_callback(self, msg):
        # Convert Twist to wheel velocities
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        
        # Calculate wheel velocities
        self.right_wheel_vel = (self.vx + self.vth * self.wheel_separation / 2.0) / self.wheel_radius
        self.left_wheel_vel = (self.vx - self.vth * self.wheel_separation / 2.0) / self.wheel_radius

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Compute odometry
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Create quaternion from yaw
        quat = self.euler_to_quaternion(0, 0, self.th)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Set velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vth
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Publish joint states (for visualization)
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['Left_wheel_Axis', 'Right_wheel_Axis']
        
        # Simulate wheel positions by integrating velocities
        joint_state.position = [0.0, 0.0]  # We don't track absolute position, just velocities
        joint_state.velocity = [self.left_wheel_vel, self.right_wheel_vel]
        
        self.joint_pub.publish(joint_state)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        
        return q

def main(args=None):
    rclpy.init(args=args)
    controller = DifferentialDriveController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 