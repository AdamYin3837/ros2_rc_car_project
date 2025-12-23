#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import struct

class CanBus(Node):
    def __init__(self):
        super().__init__('canbus_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Initialize CAN Bus
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        self.get_logger().info("Connected to CAN bus.")
    
    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        self.get_logger().info(f"Received cmd_vel: v={v:.2f}, omega={omega:.2f}")
        
        # Simple differential drive: left and right wheels have different speeds
        left_speed = v - omega
        right_speed = v + omega
        
        # Assign speeds to wheels
        wheel_velocities = {
            'fl': left_speed,   # Front Left
            'fr': right_speed,  # Front Right
            'bl': left_speed,   # Back Left
            'br': right_speed   # Back Right
        }
        
        # All steering angles are zero (no steering, only speed difference)
        steering_angles = {
            'fl': 0.0,
            'fr': 0.0,
            'bl': 0.0,
            'br': 0.0
        }
        
        # Send drive messages
        for actuator_id, vel in wheel_velocities.items():
            can_id = self.drive_actuator_id_to_can_id(actuator_id)
            msg_can = self.create_drive_command(can_id, vel)
            self.bus.send(msg_can)
        
        # Send steering messages
        for actuator_id, angle in steering_angles.items():
            can_id = self.steering_actuator_id_to_can_id(actuator_id)
            msg_can = self.create_steering_command(can_id, angle)
            self.bus.send(msg_can)
        
        self.get_logger().info("Wheel commands sent.")
    
    def drive_actuator_id_to_can_id(self, actuator_id):
        mapping = {'fl': 0x11, 'fr': 0x12, 'bl': 0x13, 'br': 0x14}
        return mapping[actuator_id]
    
    def steering_actuator_id_to_can_id(self, actuator_id):
        mapping = {'fl': 0x21, 'fr': 0x22, 'bl': 0x23, 'br': 0x24}
        return mapping[actuator_id]
    
    def create_drive_command(self, actuator_id, velocity=0.0):
        priority = 0x0
        command_id = 0x03
        sender_node_id = 1
        arbitration_id = (priority << 24 | command_id << 16 | 
                         actuator_id << 8 | sender_node_id)
        data = struct.pack(">f", velocity)
        return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
    
    def create_steering_command(self, actuator_id, angle=0.0):
        priority = 0x0
        command_id = 0x02
        sender_node_id = 1
        arbitration_id = (priority << 24 | command_id << 16 | 
                         actuator_id << 8 | sender_node_id)
        data = struct.pack(">f", angle)
        return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)

def main(args=None):
    rclpy.init(args=args)
    canbus_node = CanBus()
    rclpy.spin(canbus_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()