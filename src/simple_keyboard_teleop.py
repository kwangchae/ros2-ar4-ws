#!/usr/bin/env python3

"""
AR4 Simple Keyboard Teleoperation Controller
Simple keyboard control using input() for WSL2 compatibility
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import threading
import time

class SimpleKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('ar4_simple_teleop')
        
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
        self.step_size = 0.1  # radians per command
        self.selected_joint = 0  # Currently selected joint (0-5)
        self.running = True
        
        self.get_logger().info('AR4 Simple Keyboard Teleoperation Controller initialized')
        self.print_instructions()
        
    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        if len(msg.position) >= 6:
            self.current_positions = list(msg.position[:6])
    
    def print_instructions(self):
        """Print available commands"""
        print("\n" + "="*70)
        print("🤖 AR4 SIMPLE KEYBOARD TELEOPERATION CONTROLLER")
        print("="*70)
        print("COMMANDS (type and press Enter):")
        print("")
        print("JOINT SELECTION:")
        print("  1, 2, 3, 4, 5, 6  : Select joint")
        print("  next, prev         : Next/Previous joint")
        print("")
        print("JOINT CONTROL:")
        print("  +, -, up, down     : Increase/Decrease selected joint")
        print("  ++, --             : Large step increase/decrease")
        print("")
        print("DIRECT CONTROL:")
        print("  j1+, j1-, j2+, j2-, etc. : Direct joint control")
        print("")
        print("PRESET POSES:")
        print("  home               : Go to home position")
        print("  ready              : Go to ready position")
        print("  demo               : Run demo sequence")
        print("")
        print("UTILITY:")
        print("  info, i            : Show current joint states")
        print("  step [value]       : Set step size (e.g., 'step 0.05')")
        print("  stop               : Emergency stop")
        print("  help               : Show this help")
        print("  quit, exit         : Exit program")
        print("="*70)
        
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
            print(f"✅ Moving to {preset_name} position")
            self.print_current_state()
        
    def print_current_state(self):
        """Print current joint states"""
        print(f"\n📊 Current Joint States (Selected: Joint {self.selected_joint + 1})")
        print("-" * 60)
        for i, (name, pos) in enumerate(zip(self.joint_names, self.current_positions)):
            marker = "👉" if i == self.selected_joint else "  "
            deg = math.degrees(pos)
            min_deg = math.degrees(self.joint_limits[name][0])
            max_deg = math.degrees(self.joint_limits[name][1])
            print(f"{marker} {name}: {pos:+7.3f} rad ({deg:+7.1f}°) "
                  f"[{min_deg:+6.1f}° to {max_deg:+6.1f}°]")
        print(f"Step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)")
        
    def run_demo_sequence(self):
        """Run a quick demo sequence"""
        print("🎬 Starting demo sequence...")
        poses = [
            ('home', 1.0),
            ('ready', 1.5),
            ('home', 1.0)
        ]
        
        for pose_name, duration in poses:
            self.go_to_preset(pose_name)
            time.sleep(duration)
            
        print("✅ Demo sequence completed!")
        
    def process_command(self, command):
        """Process user command"""
        cmd = command.lower().strip()
        
        try:
            # Joint selection
            if cmd.isdigit() and '1' <= cmd <= '6':
                self.selected_joint = int(cmd) - 1
                print(f"Selected joint_{self.selected_joint + 1}")
                return True
                
            elif cmd in ['next', 'n']:
                self.selected_joint = (self.selected_joint + 1) % 6
                print(f"Selected joint_{self.selected_joint + 1}")
                return True
                
            elif cmd in ['prev', 'p']:
                self.selected_joint = (self.selected_joint - 1) % 6
                print(f"Selected joint_{self.selected_joint + 1}")
                return True
                
            # Joint control
            elif cmd in ['+', 'up']:
                new_pos = self.current_positions[self.selected_joint] + self.step_size
                self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                self.publish_joint_positions(self.current_positions)
                print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad")
                return True
                
            elif cmd in ['-', 'down']:
                new_pos = self.current_positions[self.selected_joint] - self.step_size
                self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                self.publish_joint_positions(self.current_positions)
                print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad")
                return True
                
            elif cmd == '++':
                new_pos = self.current_positions[self.selected_joint] + (self.step_size * 3)
                self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                self.publish_joint_positions(self.current_positions)
                print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad (large step)")
                return True
                
            elif cmd == '--':
                new_pos = self.current_positions[self.selected_joint] - (self.step_size * 3)
                self.current_positions[self.selected_joint] = self.clamp_joint(self.selected_joint, new_pos)
                self.publish_joint_positions(self.current_positions)
                print(f"Joint {self.selected_joint + 1}: {self.current_positions[self.selected_joint]:+.3f} rad (large step)")
                return True
                
            # Direct joint control
            elif cmd.startswith('j') and len(cmd) >= 3:
                try:
                    joint_num = int(cmd[1]) - 1
                    direction = cmd[2:]
                    if 0 <= joint_num < 6 and direction in ['+', '-']:
                        step = self.step_size if direction == '+' else -self.step_size
                        new_pos = self.current_positions[joint_num] + step
                        self.current_positions[joint_num] = self.clamp_joint(joint_num, new_pos)
                        self.publish_joint_positions(self.current_positions)
                        print(f"Joint {joint_num + 1}: {self.current_positions[joint_num]:+.3f} rad")
                        return True
                except:
                    pass
                    
            # Preset poses
            elif cmd in ['home', 'h']:
                self.go_to_preset('home')
                return True
                
            elif cmd in ['ready', 'r']:
                self.go_to_preset('ready')
                return True
                
            elif cmd in ['demo', 'd']:
                demo_thread = threading.Thread(target=self.run_demo_sequence, daemon=True)
                demo_thread.start()
                return True
                
            # Utility commands
            elif cmd in ['info', 'i']:
                self.print_current_state()
                return True
                
            elif cmd.startswith('step'):
                try:
                    parts = cmd.split()
                    if len(parts) == 2:
                        new_step = float(parts[1])
                        if 0.01 <= new_step <= 0.5:
                            self.step_size = new_step
                            print(f"Step size set to {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)")
                        else:
                            print("Step size must be between 0.01 and 0.5 radians")
                    else:
                        print(f"Current step size: {self.step_size:.3f} rad ({math.degrees(self.step_size):.1f}°)")
                except ValueError:
                    print("Invalid step size value")
                return True
                
            elif cmd in ['stop', 's']:
                self.publish_joint_positions(self.current_positions)
                print("🛑 Emergency stop - holding current position")
                return True
                
            elif cmd in ['help']:
                self.print_instructions()
                return True
                
            elif cmd in ['quit', 'exit', 'q']:
                return False
                
            else:
                print(f"❓ Unknown command: '{command}'. Type 'help' for available commands.")
                return True
                
        except Exception as e:
            print(f"❌ Error processing command: {e}")
            return True
    
    def input_loop(self):
        """Main input loop"""
        print(f"\n🎮 Control active. Selected joint: {self.selected_joint + 1}")
        print("Type commands and press Enter. Type 'help' for instructions.")
        
        while self.running:
            try:
                command = input("AR4> ").strip()
                if command:
                    if not self.process_command(command):
                        break
            except KeyboardInterrupt:
                print("\n🛑 Interrupted by user")
                break
            except EOFError:
                print("\n🛑 EOF received")
                break
                
        self.running = False

def main():
    rclpy.init()
    
    teleop = SimpleKeyboardTeleop()
    
    # Start input loop in separate thread
    input_thread = threading.Thread(target=teleop.input_loop, daemon=True)
    input_thread.start()
    
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down simple keyboard teleop...")
    finally:
        teleop.running = False
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()