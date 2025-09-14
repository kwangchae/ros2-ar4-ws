#!/usr/bin/env python3

"""
AR4 Keyboard Teleoperation Controller
Real-time keyboard control for AR4 robot joints
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading
import sys
try:
    import termios
    import tty
    UNIX_SYSTEM = True
except ImportError:
    UNIX_SYSTEM = False
import math

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('ar4_keyboard_teleop')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Subscriber to current joint states
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Joint configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.current_positions = [0.0] * 6
        self.joint_limits = {
            'joint_1': (-2.96, 2.96),    # ±170°
            'joint_2': (-1.74, 1.04),    # -100° to +60°
            'joint_3': (-1.57, 1.57),    # ±90°
            'joint_4': (-1.91, 1.91),    # ±110°
            'joint_5': (-2.18, 2.18),    # ±125°
            'joint_6': (-3.05, 3.05)     # ±175°
        }
        
        # Control parameters
        self.step_size = 0.05  # radians per keystroke (about 3 degrees)
        self.selected_joint = 0  # Currently selected joint (0-5)
        self.running = True
        
        # Terminal settings for raw input
        self.old_settings = None
        
        self.get_logger().info('AR4 Keyboard Teleoperation Controller initialized')
        self.print_instructions()
        
    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        if len(msg.position) >= 6:
            self.current_positions = list(msg.position[:6])
    
    def print_instructions(self):
        """Print keyboard controls"""
        print("\n" + "="*60)
        print("🤖 AR4 KEYBOARD TELEOPERATION CONTROLLER")
        print("="*60)
        print("JOINT SELECTION:")
        print("  1-6      : Select joint 1-6")
        print("  q/w      : Select previous/next joint")
        print("")
        print("JOINT CONTROL:")
        print("  +/=      : Increase selected joint angle")
        print("  -/_      : Decrease selected joint angle")
        print("  a/d      : Decrease/Increase selected joint (alternative)")
        print("")
        print("PRESET POSES:")
        print("  h        : Go to home position (all zeros)")
        print("  r        : Go to ready position")
        print("  space    : Emergency stop (current position)")
        print("")
        print("CONTROL:")
        print("  [/]      : Decrease/Increase step size")
        print("  i        : Show current joint info")
        print("  ESC/Ctrl+C: Exit")
        print("="*60)
        
    def get_key(self):
        """Get single keypress without Enter"""
        if UNIX_SYSTEM:
            if self.old_settings is None:
                self.old_settings = termios.tcgetattr(sys.stdin)

            try:
                tty.cbreak(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

            return ch
        else:
            # Windows fallback - use input() with prompt
            try:
                return input("Enter command (or 'help' for instructions): ").strip()
            except EOFError:
                return '\x03'  # Ctrl+C
    
    def clamp_joint(self, joint_idx, position):
        """Clamp joint position within limits"""
        joint_name = self.joint_names[joint_idx]
        min_pos, max_pos = self.joint_limits[joint_name]
        return max(min_pos, min(max_pos, position))
    
    def publish_joint_positions(self, positions):
        """Publish joint command"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ar4'
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        
        self.joint_pub.publish(msg)
        
    def go_to_preset(self, preset_name):
        """Move to preset position"""
        presets = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]
        }
        
        if preset_name in presets:
            self.current_positions = presets[preset_name].copy()
            self.publish_joint_positions(self.current_positions)
            self.get_logger().info(f"Moving to {preset_name} position")
            self.print_current_state()
        
    def print_current_state(self):
        """Print current joint states"""
        print(f"\n📊 Current Joint States (Selected: Joint {self.selected_joint + 1})")
        print("-" * 50)
        for i, (name, pos) in enumerate(zip(self.joint_names, self.current_positions)):
            marker = "👉" if i == self.selected_joint else "  "
            deg = math.degrees(pos)
            min_deg = math.degrees(self.joint_limits[name][0])
            max_deg = math.degrees(self.joint_limits[name][1])
            print(f"{marker} {name}: {pos:+6.3f} rad ({deg:+6.1f}°) [{min_deg:+6.1f}° to {max_deg:+6.1f}°]")
        print(f"Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)")
        
    def keyboard_loop(self):
        """Main keyboard input loop"""
        print("\n🎮 Keyboard control active. Press 'i' for current state info.")
        
        while self.running:
            try:
                key = self.get_key()
                
                # Exit conditions
                if key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
                    break
                    
                # Joint selection (1-6)
                elif key.isdigit() and '1' <= key <= '6':
                    self.selected_joint = int(key) - 1
                    print(f"Selected joint_{self.selected_joint + 1}")
                    
                # Joint selection (q/w)
                elif key == 'q':
                    self.selected_joint = (self.selected_joint - 1) % 6
                    print(f"Selected joint_{self.selected_joint + 1}")
                elif key == 'w':
                    self.selected_joint = (self.selected_joint + 1) % 6
                    print(f"Selected joint_{self.selected_joint + 1}")
                    
                # Joint control
                elif key in ['+', '=']:
                    new_pos = self.current_positions[self.selected_joint] + self.step_size
                    self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                    self.publish_joint_positions(self.current_positions)
                    print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad")
                    
                elif key in ['-', '_']:
                    new_pos = self.current_positions[self.selected_joint] - self.step_size
                    self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                    self.publish_joint_positions(self.current_positions)
                    print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad")
                    
                # Alternative control (a/d)
                elif key == 'a':
                    new_pos = self.current_positions[self.selected_joint] - self.step_size
                    self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                    self.publish_joint_positions(self.current_positions)
                    print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad")
                    
                elif key == 'd':
                    new_pos = self.current_positions[self.selected_joint] + self.step_size
                    self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                    self.publish_joint_positions(self.current_positions)
                    print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad")
                    
                # Step size adjustment
                elif key == '[':
                    self.step_size = max(0.01, self.step_size - 0.01)
                    print(f"Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)")
                elif key == ']':
                    self.step_size = min(0.20, self.step_size + 0.01)
                    print(f"Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)")
                    
                # Preset poses
                elif key == 'h':
                    self.go_to_preset('home')
                elif key == 'r':
                    self.go_to_preset('ready')
                    
                # Emergency stop
                elif key == ' ':
                    self.publish_joint_positions(self.current_positions)
                    print("🛑 Emergency stop - holding current position")
                    
                # Info
                elif key == 'i':
                    self.print_current_state()
                    
            except Exception as e:
                self.get_logger().error(f"Keyboard input error: {e}")
                break
                
        self.running = False
        
    def cleanup(self):
        """Cleanup terminal settings"""
        if UNIX_SYSTEM and self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def main():
    rclpy.init()
    
    teleop = KeyboardTeleop()
    
    # Start keyboard input in separate thread
    keyboard_thread = threading.Thread(target=teleop.keyboard_loop, daemon=True)
    keyboard_thread.start()
    
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down keyboard teleop...")
    finally:
        teleop.running = False
        teleop.cleanup()
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()