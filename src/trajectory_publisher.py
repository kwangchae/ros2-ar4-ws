#!/usr/bin/env python3

"""
AR4 Trajectory Publisher for Unity Visualization
Publishes intelligent planned trajectories with waypoints for Unity TrajectoryVisualizer
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import math
import numpy as np
from typing import List, Tuple

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Publishers
        self.trajectory_pub = self.create_publisher(JointState, '/trajectory_preview', 10)
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Joint names and limits
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_limits = {
            'joint_1': (-2.96, 2.96),
            'joint_2': (-1.74, 1.04),
            'joint_3': (-1.57, 1.57),
            'joint_4': (-1.91, 1.91),
            'joint_5': (-2.18, 2.18),
            'joint_6': (-3.05, 3.05)
        }
        
        # Predefined poses for intelligent trajectories
        self.poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
            'pick_high': [0.5, -0.8, 1.2, 0.0, 0.6, 0.0],
            'pick_low': [0.5, -1.1, 1.4, 0.0, 0.8, 0.0],
            'transfer': [0.0, -0.6, 0.8, 0.0, 0.3, 0.0],
            'place_high': [-0.5, -0.8, 1.2, 0.0, 0.6, 0.0],
            'place_low': [-0.5, -1.1, 1.4, 0.0, 0.8, 0.0],
            'inspect': [0.0, -0.3, 0.2, 0.0, -0.5, 0.0],
            'side_left': [1.5, -0.7, 1.0, 0.0, 0.4, 0.0],
            'side_right': [-1.5, -0.7, 1.0, 0.0, 0.4, 0.0],
        }
        
        self.get_logger().info('AR4 Trajectory Publisher initialized')
        self.print_menu()
        
    def print_menu(self):
        """Print available trajectory commands"""
        print("\n" + "="*60)
        print("🛣️  AR4 TRAJECTORY PUBLISHER")
        print("="*60)
        print("INTELLIGENT TRAJECTORY PATTERNS:")
        print("1. pick_and_place()      - Complete pick & place sequence")
        print("2. inspection_pattern()  - Multi-angle inspection trajectory") 
        print("3. sweep_motion()        - Wide sweeping motion")
        print("4. figure_eight()        - Figure-8 pattern in workspace")
        print("5. spiral_motion()       - 3D spiral trajectory")
        print("6. waypoint_navigation() - Multi-waypoint navigation")
        print("")
        print("CUSTOM TRAJECTORIES:")
        print("7. smooth_path(start, end, waypoints) - Smooth interpolated path")
        print("8. circular_motion()     - Circular end-effector motion")
        print("9. zig_zag_pattern()     - Zig-zag scanning pattern")
        print("")
        print("Available poses:", list(self.poses.keys()))
        print("="*60)
        
    def clamp_joint_positions(self, positions: List[float]) -> List[float]:
        """Clamp all joint positions within their limits"""
        clamped = []
        for i, (joint_name, pos) in enumerate(zip(self.joint_names, positions)):
            min_pos, max_pos = self.joint_limits[joint_name]
            clamped.append(max(min_pos, min(max_pos, pos)))
        return clamped
        
    def interpolate_trajectory(self, start_pose: List[float], end_pose: List[float], 
                             num_waypoints: int = 10, interpolation_type: str = 'cubic') -> List[List[float]]:
        """Generate smooth interpolated trajectory between two poses"""
        trajectory = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            # Different interpolation methods
            if interpolation_type == 'linear':
                t_smooth = t
            elif interpolation_type == 'cubic':
                t_smooth = t * t * (3.0 - 2.0 * t)  # Cubic Hermite
            elif interpolation_type == 'quintic':
                t_smooth = t * t * t * (6.0 * t * t - 15.0 * t + 10.0)  # Quintic
            else:
                t_smooth = t
                
            # Interpolate each joint
            waypoint = []
            for j in range(len(start_pose)):
                pos = start_pose[j] + t_smooth * (end_pose[j] - start_pose[j])
                waypoint.append(pos)
                
            trajectory.append(self.clamp_joint_positions(waypoint))
            
        return trajectory
        
    def publish_trajectory_waypoint(self, joint_positions: List[float], waypoint_index: int):
        """Publish a single waypoint for Unity visualization"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'waypoint_{waypoint_index}'
        msg.name = self.joint_names
        msg.position = joint_positions
        msg.velocity = []
        msg.effort = []
        
        self.trajectory_pub.publish(msg)
        self.get_logger().debug(f'Published waypoint {waypoint_index}')
        
    def publish_and_execute_trajectory(self, trajectory: List[List[float]], 
                                     execution_delay: float = 0.5, 
                                     preview_delay: float = 0.1):
        """Publish trajectory for visualization and optionally execute it"""
        
        self.get_logger().info(f'Publishing trajectory with {len(trajectory)} waypoints')
        
        # First, publish all waypoints for visualization
        for i, waypoint in enumerate(trajectory):
            self.publish_trajectory_waypoint(waypoint, i)
            time.sleep(preview_delay)
            
        self.get_logger().info('✅ Trajectory visualization published. Starting execution...')
        time.sleep(1.0)
        
        # Then execute the trajectory by publishing joint commands
        for i, waypoint in enumerate(trajectory):
            # Publish joint command for actual robot movement
            cmd_msg = JointState()
            cmd_msg.header = Header()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = 'ar4'
            cmd_msg.name = self.joint_names
            cmd_msg.position = waypoint
            cmd_msg.velocity = []
            cmd_msg.effort = []
            
            self.joint_command_pub.publish(cmd_msg)
            self.get_logger().info(f'→ Executing waypoint {i+1}/{len(trajectory)}')
            
            time.sleep(execution_delay)
            
        self.get_logger().info('🎯 Trajectory execution completed!')
        
    def pick_and_place(self):
        """Intelligent pick and place trajectory with optimized waypoints"""
        self.get_logger().info('🔄 Starting Pick & Place Trajectory')
        
        sequence = [
            ('home', 'ready'),
            ('ready', 'pick_high'),
            ('pick_high', 'pick_low'),
            ('pick_low', 'pick_high'),
            ('pick_high', 'transfer'),
            ('transfer', 'place_high'),
            ('place_high', 'place_low'),
            ('place_low', 'place_high'),
            ('place_high', 'ready'),
            ('ready', 'home')
        ]
        
        full_trajectory = []
        for start_pose, end_pose in sequence:
            segment = self.interpolate_trajectory(
                self.poses[start_pose], 
                self.poses[end_pose], 
                num_waypoints=8,
                interpolation_type='cubic'
            )
            full_trajectory.extend(segment[1:])  # Skip first point to avoid duplication
            
        self.publish_and_execute_trajectory(full_trajectory, execution_delay=0.3)
        
    def inspection_pattern(self):
        """Multi-angle inspection trajectory around a central object"""
        self.get_logger().info('🔍 Starting Inspection Pattern')
        
        # Create circular inspection pattern
        center_pose = self.poses['inspect']
        inspection_trajectory = []
        
        num_positions = 12  # 12 inspection angles
        radius_variation = 0.3
        
        for i in range(num_positions + 1):
            angle = 2 * math.pi * i / num_positions
            
            # Vary the inspection distance and height
            offset_x = radius_variation * math.cos(angle)
            offset_y = 0.1 * math.sin(2 * angle)  # Slight vertical movement
            
            inspect_pose = center_pose.copy()
            inspect_pose[0] += offset_x  # Joint 1 (base rotation)
            inspect_pose[1] += offset_y  # Joint 2 (shoulder)
            inspect_pose[4] = angle * 0.2  # Joint 5 (wrist rotation for different angles)
            
            inspection_trajectory.append(self.clamp_joint_positions(inspect_pose))
            
        self.publish_and_execute_trajectory(inspection_trajectory, execution_delay=0.4)
        
    def figure_eight(self):
        """Figure-8 trajectory pattern in the robot workspace"""
        self.get_logger().info('∞ Starting Figure-8 Pattern')
        
        trajectory = []
        num_points = 24
        
        for i in range(num_points + 1):
            t = 2 * math.pi * i / num_points
            
            # Figure-8 parametric equations
            scale = 0.8
            x_offset = scale * math.sin(t)
            y_offset = scale * math.sin(2 * t) * 0.3
            z_offset = scale * math.cos(t) * 0.2
            
            # Base pose for figure-8
            base_pose = self.poses['ready'].copy()
            base_pose[0] += x_offset  # Joint 1
            base_pose[1] += y_offset  # Joint 2  
            base_pose[2] += z_offset  # Joint 3
            
            trajectory.append(self.clamp_joint_positions(base_pose))
            
        self.publish_and_execute_trajectory(trajectory, execution_delay=0.3)
        
    def spiral_motion(self):
        """3D spiral trajectory ascending through workspace"""
        self.get_logger().info('🌀 Starting Spiral Motion')
        
        trajectory = []
        num_points = 20
        height_gain = 0.5
        
        for i in range(num_points + 1):
            t = 3 * math.pi * i / num_points  # 1.5 full rotations
            height_factor = i / num_points
            
            # Spiral parameters
            radius = 0.6 * (1 - height_factor * 0.3)  # Shrinking radius
            x_spiral = radius * math.cos(t)
            y_spiral = radius * math.sin(t)
            z_spiral = height_gain * height_factor
            
            spiral_pose = self.poses['ready'].copy()
            spiral_pose[0] += x_spiral
            spiral_pose[1] += y_spiral * 0.3
            spiral_pose[2] += z_spiral
            spiral_pose[4] = t * 0.2  # Wrist follows spiral
            
            trajectory.append(self.clamp_joint_positions(spiral_pose))
            
        self.publish_and_execute_trajectory(trajectory, execution_delay=0.4)
        
    def sweep_motion(self):
        """Wide sweeping motion across the workspace"""
        self.get_logger().info('🧹 Starting Sweep Motion')
        
        # Create sweeping pattern from left to right, multiple passes at different heights
        trajectory = []
        
        sweep_positions = [
            ('side_left', 'side_right'),
            ('side_right', 'side_left'),
        ]
        
        for start_pos, end_pos in sweep_positions:
            segment = self.interpolate_trajectory(
                self.poses[start_pos],
                self.poses[end_pos],
                num_waypoints=15,
                interpolation_type='quintic'
            )
            trajectory.extend(segment)
            
        # Add vertical sweep
        high_sweep = self.poses['side_left'].copy()
        high_sweep[1] -= 0.3  # Higher position
        low_sweep = self.poses['side_right'].copy()
        low_sweep[1] -= 0.1   # Lower position
        
        vertical_segment = self.interpolate_trajectory(high_sweep, low_sweep, num_waypoints=10)
        trajectory.extend(vertical_segment)
        
        self.publish_and_execute_trajectory(trajectory, execution_delay=0.25)
        
    def waypoint_navigation(self):
        """Navigate through multiple strategic waypoints"""
        self.get_logger().info('🗺️  Starting Waypoint Navigation')
        
        waypoint_sequence = ['home', 'ready', 'side_left', 'inspect', 'side_right', 'transfer', 'ready', 'home']
        
        full_trajectory = []
        for i in range(len(waypoint_sequence) - 1):
            start_pose = waypoint_sequence[i]
            end_pose = waypoint_sequence[i + 1]
            
            segment = self.interpolate_trajectory(
                self.poses[start_pose],
                self.poses[end_pose],
                num_waypoints=6,
                interpolation_type='cubic'
            )
            full_trajectory.extend(segment[1:] if i > 0 else segment)
            
        self.publish_and_execute_trajectory(full_trajectory, execution_delay=0.4)
        
    def circular_motion(self):
        """Circular end-effector motion maintaining orientation"""
        self.get_logger().info('⭕ Starting Circular Motion')
        
        trajectory = []
        num_points = 16
        radius = 0.4
        
        center_pose = self.poses['ready'].copy()
        
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            
            circular_pose = center_pose.copy()
            circular_pose[0] += radius * math.cos(angle)  # X movement
            circular_pose[1] += radius * math.sin(angle) * 0.3  # Y movement (scaled)
            
            trajectory.append(self.clamp_joint_positions(circular_pose))
            
        self.publish_and_execute_trajectory(trajectory, execution_delay=0.3)

def main():
    rclpy.init()
    
    trajectory_publisher = TrajectoryPublisher()
    
    def run_demo_sequence():
        """Run a sequence of different trajectory patterns"""
        time.sleep(2)
        
        print("\n🚀 Starting Intelligent Trajectory Demo Sequence...")
        
        demo_patterns = [
            ('Pick & Place', trajectory_publisher.pick_and_place),
            ('Inspection Pattern', trajectory_publisher.inspection_pattern),
            ('Figure-8', trajectory_publisher.figure_eight),
            ('Spiral Motion', trajectory_publisher.spiral_motion),
            ('Circular Motion', trajectory_publisher.circular_motion),
            ('Waypoint Navigation', trajectory_publisher.waypoint_navigation),
        ]
        
        for pattern_name, pattern_func in demo_patterns:
            print(f"\n▶️  Executing: {pattern_name}")
            time.sleep(1)
            pattern_func()
            print(f"✅ Completed: {pattern_name}")
            time.sleep(2)
            
        print("\n🎉 All trajectory patterns completed!")
    
    # Start demo sequence in background
    import threading
    demo_thread = threading.Thread(target=run_demo_sequence, daemon=True)
    demo_thread.start()
    
    try:
        rclpy.spin(trajectory_publisher)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Trajectory Publisher...")
    finally:
        trajectory_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()