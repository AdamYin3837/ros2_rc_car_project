#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#For receiving Xbox controller inputs
from sensor_msgs.msg import Joy
#For sending velocity commands to the car
from geometry_msgs.msg import Twist


class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_controller_node')
        #The code of self.sbuscription is when the /joy topic listens to an update, execute joy_callback once!
        self.subscription_ = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #Max forward/backward speed (m/s)
        self.max_linear_speed_ = 1.0
        #Max turning rate (rad/s)
        self.max_angular_speed_ = 1.0
        #Keeps current speed state (m/s)
        self.current_linear_speed_ = 0.0
        #Step size for speed change (m/s per cycle)
        #if executing a loop takes 0.1 sec, then speeding up from 0.0 to 1.0 m/s takes 20 loops (2 sec)
        self.ramp_rate_ = 0.05

    def joy_callback(self, msg):
        twist = Twist()
        #axes[1] = Left stick (up/down), ragnging from -1.0 to 1.0
        #For xbox stick: +1.0 = fully released; 0.0 = half pressed; -1.0 = fully pressed
        linear_trigger = 1 - msg.axes[1]
        #axes[2] = Right stick (left/right), ranging from -1/0 to 1.0
        #For xbox stick: -1.0 = turn left; 0.0 = middle; +1.0 = turn right
        angular_trigger = msg.axes[2]

        linear_speed = linear_trigger * self.max_linear_speed_
        angular_speed = -angular_trigger * self.max_angular_speed_
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    xbox_controller_node = XboxControllerNode()
    rclpy.spin(xbox_controller_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
