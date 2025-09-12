#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class SimpleJointTest(Node):
    def __init__(self):
        super().__init__('simple_joint_test')
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
    def send_simple_command(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'simple_test'
        msg.name = self.joint_names
        # Send simple positions (in radians)
        msg.position = [0.5, -0.5, 0.3, 0.0, 0.2, 0.0]  
        msg.velocity = []
        msg.effort = []
        
        self.joint_pub.publish(msg)
        print(f"📤 Sent joint command: {msg.position}")

def main():
    rclpy.init()
    tester = SimpleJointTest()
    
    print("🔄 Sending simple joint commands every 2 seconds...")
    print("Watch Unity robot for movement!")
    
    for i in range(10):
        tester.send_simple_command()
        time.sleep(2)
        print(f"Command {i+1}/10 sent")
    
    print("✅ Test completed")
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()