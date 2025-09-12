#!/usr/bin/env python3

"""
AR4 Smooth Robot Controller
Provides smooth, controlled movement for Unity robot to prevent erratic motion
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import threading
import time
from typing import List

class SmoothRobotController(Node):
    def __init__(self):
        super().__init__('smooth_robot_controller')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointState, '/trajectory_preview', self.trajectory_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Joint configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Current state
        self.current_positions = [0.0] * 6
        self.target_positions = [0.0] * 6
        self.previous_positions = [0.0] * 6
        
        # Movement parameters
        self.max_velocity = 0.5  # rad/s per joint
        self.max_acceleration = 1.0  # rad/s² per joint
        self.position_tolerance = 0.01  # rad
        self.control_frequency = 50.0  # Hz
        
        # Control state
        self.is_moving = False
        self.movement_start_time = 0.0
        self.target_velocities = [0.0] * 6
        
        # Create control timer
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        self.get_logger().info('Smooth Robot Controller initialized')
        print("\n" + "="*60)
        print("🤖 AR4 SMOOTH ROBOT CONTROLLER")
        print("="*60)
        print("Features:")
        print("• Velocity limiting to prevent jerky movements")
        print("• Acceleration limiting for smooth starts/stops")
        print("• Position filtering to avoid excessive oscillation")
        print("• Real-time trajectory following with interpolation")
        print("="*60)
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        if len(msg.position) >= 6:
            self.current_positions = list(msg.position[:6])
            
    def trajectory_callback(self, msg):
        """Handle incoming trajectory waypoints with smooth interpolation"""
        if len(msg.position) >= 6:
            new_targets = list(msg.position[:6])
            
            # Check if this is significantly different from current target
            position_diff = sum(abs(new - curr) for new, curr in 
                              zip(new_targets, self.target_positions))
                              
            if position_diff > self.position_tolerance:
                self.target_positions = new_targets
                self.is_moving = True
                self.movement_start_time = time.time()
                
                self.get_logger().debug(f'New target: {[f"{pos:.3f}" for pos in new_targets]}')
                
    def control_loop(self):
        """Main control loop for smooth movement"""
        if not self.is_moving:
            return
            
        dt = 1.0 / self.control_frequency
        current_time = time.time()
        
        # Calculate desired positions with smooth interpolation
        desired_positions = []
        all_joints_at_target = True
        
        for i in range(6):
            current = self.current_positions[i]
            target = self.target_positions[i]
            
            # Calculate position error
            error = target - current
            
            if abs(error) > self.position_tolerance:
                all_joints_at_target = False
                
                # Apply velocity limiting
                max_step = self.max_velocity * dt
                if abs(error) > max_step:
                    step = max_step if error > 0 else -max_step
                else:
                    step = error
                    
                # Apply acceleration limiting
                desired_velocity = step / dt
                current_velocity = self.target_velocities[i]
                
                max_accel_step = self.max_acceleration * dt
                velocity_diff = desired_velocity - current_velocity
                
                if abs(velocity_diff) > max_accel_step:
                    velocity_diff = max_accel_step if velocity_diff > 0 else -max_accel_step
                    
                self.target_velocities[i] = current_velocity + velocity_diff
                desired_position = current + self.target_velocities[i] * dt
                
            else:
                # Joint is at target
                desired_position = target
                self.target_velocities[i] = 0.0
                
            desired_positions.append(desired_position)
            
        # Publish smooth command
        self.publish_smooth_command(desired_positions)
        
        # Update previous positions
        self.previous_positions = desired_positions.copy()
        
        # Check if movement is complete
        if all_joints_at_target:
            self.is_moving = False
            self.target_velocities = [0.0] * 6
            self.get_logger().debug('Movement completed')
            
    def publish_smooth_command(self, positions: List[float]):
        """Publish smooth joint command"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ar4_smooth'
        msg.name = self.joint_names
        msg.position = positions
        
        # Add velocity information for Unity
        msg.velocity = self.target_velocities
        msg.effort = []
        
        self.joint_pub.publish(msg)
        
    def set_movement_parameters(self, max_velocity: float = None, 
                              max_acceleration: float = None):
        """Update movement parameters for different motion profiles"""
        if max_velocity is not None:
            self.max_velocity = max_velocity
            self.get_logger().info(f'Max velocity set to {max_velocity:.3f} rad/s')
            
        if max_acceleration is not None:
            self.max_acceleration = max_acceleration  
            self.get_logger().info(f'Max acceleration set to {max_acceleration:.3f} rad/s²')
            
    def emergency_stop(self):
        """Emergency stop - freeze at current position"""
        self.target_positions = self.current_positions.copy()
        self.target_velocities = [0.0] * 6
        self.is_moving = False
        
        self.publish_smooth_command(self.current_positions)
        self.get_logger().warn('🛑 Emergency stop activated!')
        
    def go_to_position_smooth(self, target_positions: List[float], duration: float = 2.0):
        """Move to specific position with controlled timing"""
        if len(target_positions) != 6:
            self.get_logger().error('Target positions must have 6 values')
            return
            
        # Calculate required velocity based on duration
        max_distance = max(abs(target - current) for target, current in 
                          zip(target_positions, self.current_positions))
                          
        if max_distance > 0:
            required_velocity = max_distance / duration
            old_velocity = self.max_velocity
            
            # Temporarily adjust velocity for this movement
            self.max_velocity = min(required_velocity, 2.0)  # Cap at 2.0 rad/s
            
            self.target_positions = target_positions
            self.is_moving = True
            self.movement_start_time = time.time()
            
            self.get_logger().info(f'Moving to position over {duration:.1f}s '
                                 f'(velocity: {self.max_velocity:.3f} rad/s)')
            
            # Restore original velocity after movement
            def restore_velocity():
                time.sleep(duration + 0.5)
                self.max_velocity = old_velocity
                
            threading.Thread(target=restore_velocity, daemon=True).start()

def main():
    rclpy.init()
    
    controller = SmoothRobotController()
    
    def interactive_control():
        """Interactive control for testing"""
        time.sleep(2)
        
        print("\n🎮 Smooth Control Commands:")
        print("• The controller is now filtering all trajectory commands")
        print("• Robot movements will be smooth and controlled")
        print("• Original trajectory publisher can continue running")
        print("\nTesting smooth movements...")
        
        # Test some smooth movements
        test_positions = [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2.0),  # Home
            ([0.0, -0.5, 0.5, 0.0, 0.0, 0.0], 3.0),  # Ready
            ([0.5, -0.8, 1.0, 0.0, 0.5, 0.0], 4.0),  # Pick position
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 3.0),   # Back to home
        ]
        
        for i, (pos, duration) in enumerate(test_positions):
            print(f"\n▶️  Test movement {i+1}: Moving over {duration}s")
            controller.go_to_position_smooth(pos, duration)
            time.sleep(duration + 1)
            
        print("\n✅ Smooth control test completed!")
        
        # Set optimal parameters for trajectory following
        controller.set_movement_parameters(max_velocity=0.3, max_acceleration=0.8)
        print("🎯 Controller optimized for trajectory following")
    
    # Start interactive control in background
    control_thread = threading.Thread(target=interactive_control, daemon=True)
    control_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Smooth Robot Controller...")
        controller.emergency_stop()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()