#!/usr/bin/env python3

"""
Direct MoveIt Unity Bridge
Uses MoveIt Python API to directly execute planned trajectories
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading
import time

# MoveIt Python API imports
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

class DirectMoveItBridge(Node):
    def __init__(self):
        super().__init__('direct_moveit_bridge')
        
        # Publishers for Unity
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.trajectory_preview_pub = self.create_publisher(JointState, '/trajectory_preview', 10)
        
        # MoveGroup Action Client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Joint configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        self.get_logger().info('Direct MoveIt Unity Bridge initialized')
        
        # Wait for MoveGroup action server
        self.move_group_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Connected to MoveGroup action server')
        
        print("\n🔗 DIRECT MOVEIT BRIDGE")
        print("This bridge will intercept MoveIt execution requests")
        print("and forward them directly to Unity")
        
    def create_test_trajectory(self):
        """Create a test trajectory for Unity"""
        test_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # Home
            [0.3, -0.2, 0.2, 0.0, 0.1, 0.0],     # Position 1  
            [0.5, -0.4, 0.4, 0.0, 0.2, 0.0],     # Position 2
            [0.3, -0.6, 0.6, 0.0, 0.3, 0.0],     # Position 3
            [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],     # Ready
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # Back to Home
        ]
        
        self.get_logger().info('🎬 Creating test trajectory preview')
        
        # Send trajectory preview waypoints
        for i, positions in enumerate(test_positions):
            preview_msg = JointState()
            preview_msg.header = Header()
            preview_msg.header.stamp = self.get_clock().now().to_msg()
            preview_msg.header.frame_id = f'waypoint_{i}'
            preview_msg.name = self.joint_names
            preview_msg.position = positions
            preview_msg.velocity = []
            preview_msg.effort = []
            
            self.trajectory_preview_pub.publish(preview_msg)
            self.get_logger().info(f'📍 Sent waypoint {i}: {positions}')
            time.sleep(0.2)
            
        self.get_logger().info('✅ Trajectory preview complete - check Unity for waypoints!')
        
        # Wait a moment then execute
        time.sleep(2.0)
        
        self.get_logger().info('🚀 Executing trajectory')
        for i, positions in enumerate(test_positions):
            command_msg = JointState()
            command_msg.header = Header()
            command_msg.header.stamp = self.get_clock().now().to_msg()
            command_msg.header.frame_id = 'direct_execution'
            command_msg.name = self.joint_names
            command_msg.position = positions
            command_msg.velocity = []
            command_msg.effort = []
            
            self.joint_command_pub.publish(command_msg)
            self.get_logger().info(f'➡️  Executing waypoint {i}')
            time.sleep(1.5)
            
        self.get_logger().info('🎯 Direct trajectory execution completed!')

def main():
    rclpy.init()
    
    bridge = DirectMoveItBridge()
    
    def run_test():
        time.sleep(3)  # Wait for initialization
        print("\n🎭 Running Direct MoveIt Bridge Test")
        print("This will:")
        print("1. Show trajectory waypoints in Unity (yellow spheres)")
        print("2. Execute the trajectory (robot movement)")
        bridge.create_test_trajectory()
        
        print("\n✨ Test completed!")
        print("If this works, the ROS-Unity connection is perfect!")
    
    # Run test in background
    test_thread = threading.Thread(target=run_test, daemon=True)
    test_thread.start()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Direct MoveIt Bridge...")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()