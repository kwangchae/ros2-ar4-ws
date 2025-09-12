#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Header
import threading
import time

class MoveItUnityBridge(Node):
    def __init__(self):
        super().__init__('moveit_unity_bridge')
        
        # Publishers
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.trajectory_preview_pub = self.create_publisher(JointState, '/trajectory_preview', 10)
        
        # Subscribers for MoveIt trajectories
        self.display_trajectory_sub = self.create_subscription(
            DisplayTrajectory, '/display_planned_path',
            self.display_trajectory_callback, 10
        )
        
        self.execute_trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory',
            self.execute_trajectory_callback, 10
        )
        
        # Joint configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        self.get_logger().info('MoveIt Unity Bridge initialized')
        print("\n🌉 MOVEIT ↔ UNITY BRIDGE READY")
        print("• Listening to MoveIt planned paths")
        print("• Will send trajectories to Unity")
        print("• Drag interactive marker in RViz and click 'Plan & Execute'!")
        
    def display_trajectory_callback(self, display_trajectory_msg):
        """Handle planned path visualization"""
        # Extract joint trajectory from DisplayTrajectory message
        if len(display_trajectory_msg.trajectory) == 0:
            return
            
        # Get the first trajectory (usually there's only one)
        trajectory_msg = display_trajectory_msg.trajectory[0].joint_trajectory
        
        if len(trajectory_msg.points) == 0:
            return
            
        self.get_logger().info(f'📋 Received planned path with {len(trajectory_msg.points)} points')
        
        # Send waypoints for Unity visualization
        for i, point in enumerate(trajectory_msg.points):
            if len(point.positions) >= 6:
                preview_msg = JointState()
                preview_msg.header = Header()
                preview_msg.header.stamp = self.get_clock().now().to_msg()
                preview_msg.header.frame_id = f'waypoint_{i}'
                preview_msg.name = self.joint_names
                preview_msg.position = list(point.positions[:6])
                preview_msg.velocity = []
                preview_msg.effort = []
                
                self.trajectory_preview_pub.publish(preview_msg)
                time.sleep(0.05)  # Small delay for visualization
                
        self.get_logger().info('✅ Path preview sent to Unity')
    
    def execute_trajectory_callback(self, trajectory_msg):
        """Handle trajectory execution"""
        if len(trajectory_msg.points) == 0:
            return
            
        self.get_logger().info(f'🚀 Executing trajectory with {len(trajectory_msg.points)} points')
        
        # Execute trajectory in separate thread
        execution_thread = threading.Thread(
            target=self.execute_trajectory, 
            args=(trajectory_msg,), 
            daemon=True
        )
        execution_thread.start()
    
    def execute_trajectory(self, trajectory_msg):
        """Execute trajectory with proper timing"""
        start_time = time.time()
        
        for i, point in enumerate(trajectory_msg.points):
            if len(point.positions) < 6:
                continue
            
            # Calculate target time
            target_time = start_time + point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            current_time = time.time()
            
            # Wait until target time
            wait_time = target_time - current_time
            if wait_time > 0:
                time.sleep(wait_time)
            
            # Send joint command
            command_msg = JointState()
            command_msg.header = Header()
            command_msg.header.stamp = self.get_clock().now().to_msg()
            command_msg.header.frame_id = 'moveit_execution'
            command_msg.name = self.joint_names
            command_msg.position = list(point.positions[:6])
            command_msg.velocity = []
            command_msg.effort = []
            
            self.joint_command_pub.publish(command_msg)
            
            # Progress feedback
            if i % max(1, len(trajectory_msg.points)//5) == 0:
                progress = (i + 1) / len(trajectory_msg.points) * 100
                self.get_logger().info(f'Progress: {progress:.1f}%')
        
        self.get_logger().info('✅ Trajectory execution completed!')

def main():
    rclpy.init()
    bridge = MoveItUnityBridge()
    
    print("\n🎮 INSTRUCTIONS:")
    print("1. In RViz, drag the orange interactive marker")
    print("2. Click 'Plan & Execute' in the Planning panel")
    print("3. Watch Unity robot follow the planned path!")
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down bridge...")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()