#!/usr/bin/env python3

"""
Test Unity Connection
Direct test to send joint commands to Unity
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import math

class TestUnityConnection(Node):
    def __init__(self):
        super().__init__('test_unity_connection')
        
        # Publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.trajectory_pub = self.create_publisher(JointState, '/trajectory_preview', 10)
        
        # Joint names
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        self.get_logger().info('Unity Connection Test initialized')
        
    def send_test_command(self, positions, message="Test command"):
        """Send test joint command to Unity"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'unity_test'
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        
        self.joint_pub.publish(msg)
        self.get_logger().info(f'📤 Sent to Unity: {message} - {positions}')
        
    def send_trajectory_preview(self, positions, waypoint_index):
        """Send trajectory preview waypoint to Unity"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'waypoint_{waypoint_index}'
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        
        self.trajectory_pub.publish(msg)
        self.get_logger().info(f'🟡 Sent trajectory waypoint {waypoint_index}: {positions}')
        
    def run_tests(self):
        """Run various Unity connection tests"""
        print("\n🧪 UNITY CONNECTION TESTS")
        print("="*50)
        
        # Test 1: Home position
        print("Test 1: Home position")
        self.send_test_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "Home position")
        time.sleep(2)
        
        # Test 2: Ready position
        print("Test 2: Ready position")
        self.send_test_command([0.0, -0.5, 0.5, 0.0, 0.0, 0.0], "Ready position")
        time.sleep(2)
        
        # Test 3: Simple movement
        print("Test 3: Simple joint movements")
        test_positions = [
            [0.3, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, -0.3, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]
        
        for i, pos in enumerate(test_positions):
            self.send_test_command(pos, f"Test move {i+1}")
            time.sleep(1.5)
            
        # Test 4: Trajectory preview (waypoints)
        print("Test 4: Trajectory waypoints visualization")
        waypoint_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.2, -0.2, 0.2, 0.0, 0.0, 0.0],
            [0.4, -0.4, 0.4, 0.0, 0.0, 0.0],
            [0.2, -0.6, 0.6, 0.0, 0.0, 0.0],
            [0.0, -0.8, 0.8, 0.0, 0.0, 0.0]
        ]
        
        for i, pos in enumerate(waypoint_positions):
            self.send_trajectory_preview(pos, i)
            time.sleep(0.3)
            
        time.sleep(2)
        
        # Test 5: Execute the waypoint trajectory
        print("Test 5: Execute waypoint trajectory")
        for i, pos in enumerate(waypoint_positions):
            self.send_test_command(pos, f"Executing waypoint {i}")
            time.sleep(1.0)
            
        print("✅ All tests completed!")

def main():
    rclpy.init()
    
    tester = TestUnityConnection()
    
    print("🔌 Testing Unity connection...")
    print("Unity should show:")
    print("• Joint movements for /joint_command")
    print("• Yellow waypoints for /trajectory_preview")
    
    time.sleep(2)
    tester.run_tests()
    
    print("\n📊 Check Unity for any movements or waypoints!")
    print("If nothing happens, there may be a Unity setup issue.")
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()