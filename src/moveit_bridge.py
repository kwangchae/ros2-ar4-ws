#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Header
from action_msgs.msg import GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_FeedbackMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import threading
import time

class MoveItUnityBridge(Node):
    def __init__(self):
        super().__init__('moveit_unity_bridge')

        # Publishers
        self.joint_command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.trajectory_preview_pub = self.create_publisher(JointState, '/trajectory_preview', 10)

        # QoS: Some MoveIt publishers use TRANSIENT_LOCAL (latched) for display topics.
        self.display_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.display_qos_volatile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscribers for MoveIt trajectories (support common topic variants)
        self.display_trajectory_sub = self.create_subscription(
            DisplayTrajectory, '/display_planned_path',
            self.display_trajectory_callback, self.display_qos
        )
        # Some configs namespace the topic under /move_group
        self.display_trajectory_sub2 = self.create_subscription(
            DisplayTrajectory, '/move_group/display_planned_path',
            self.display_trajectory_callback, self.display_qos
        )
        # Fallback volatile subscriptions (when publisher uses VOLATILE durability)
        self.display_trajectory_sub_v = self.create_subscription(
            DisplayTrajectory, '/display_planned_path',
            self.display_trajectory_callback, self.display_qos_volatile
        )
        self.display_trajectory_sub2_v = self.create_subscription(
            DisplayTrajectory, '/move_group/display_planned_path',
            self.display_trajectory_callback, self.display_qos_volatile
        )
        
        self.execute_trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory',
            self.execute_trajectory_callback, 10
        )

        # Monitor MoveIt ExecuteTrajectory action status as a fallback trigger
        # Many demo configs use the simple controller manager (no real controller topic),
        # so we watch the action status to start executing the last planned path.
        self.exec_status_sub = self.create_subscription(
            GoalStatusArray, '/execute_trajectory/_action/status',
            self.execute_status_callback, 10
        )

        # Also mirror controller feedback to Unity during execution
        self.follow_fb_sub = self.create_subscription(
            FollowJointTrajectory_FeedbackMessage,
            '/joint_trajectory_controller/follow_joint_trajectory/_action/feedback',
            self.follow_feedback_callback,
            10,
        )
        
        # Joint configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Buffer for last planned trajectory (for fallback execution)
        self._last_planned_points = []
        self._executing = False

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
        
        # Cache points for potential execution fallback
        self._last_planned_points = list(trajectory_msg.points)

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

    def follow_feedback_callback(self, fb_msg: FollowJointTrajectory_FeedbackMessage):
        """Drive Unity from controller feedback (desired positions)."""
        try:
            pt = fb_msg.feedback.desired
            if not pt.positions:
                return
            cmd = JointState()
            cmd.header = Header()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'moveit_fb'
            cmd.name = self.joint_names
            cmd.position = list(pt.positions[:6])
            self.joint_command_pub.publish(cmd)

            # Preview as well (helps Unity visualize live)
            preview = JointState()
            preview.header = Header()
            preview.header.stamp = self.get_clock().now().to_msg()
            preview.header.frame_id = 'waypoint_fb'
            preview.name = self.joint_names
            preview.position = list(pt.positions[:6])
            self.trajectory_preview_pub.publish(preview)
        except Exception as e:
            self.get_logger().warn(f'follow_feedback_callback error: {e}')

    def execute_status_callback(self, status_array: GoalStatusArray):
        """Fallback: when MoveIt starts executing, drive Unity using the last planned path."""
        # Start when we see an ACCEPTED or EXECUTING status
        if self._executing or not self._last_planned_points:
            return

        if not status_array.status_list:
            return

        # 1=ACCEPTED, 2=EXECUTING in ROS 2 action status codes
        if any(s.status in (1, 2) for s in status_array.status_list):
            self.get_logger().info('⚑ Detected MoveIt execute request (action status). Starting Unity execution...')
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            traj.points = self._last_planned_points

            # Reuse timed execution logic
            threading.Thread(target=self.execute_trajectory, args=(traj,), daemon=True).start()
            self._executing = True
    
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

            # Also publish preview waypoints so Unity shows path even if no DisplayTrajectory was received
            preview_msg = JointState()
            preview_msg.header = Header()
            preview_msg.header.stamp = self.get_clock().now().to_msg()
            # Use waypoint_0 on first point to trigger clear in visualizer
            preview_msg.header.frame_id = f'waypoint_{i}'
            preview_msg.name = self.joint_names
            preview_msg.position = list(point.positions[:6])
            preview_msg.velocity = []
            preview_msg.effort = []
            self.trajectory_preview_pub.publish(preview_msg)
            
            # Progress feedback
            if i % max(1, len(trajectory_msg.points)//5) == 0:
                progress = (i + 1) / len(trajectory_msg.points) * 100
                self.get_logger().info(f'Progress: {progress:.1f}%')
        
        self.get_logger().info('✅ Trajectory execution completed!')
        # Allow subsequent executions
        self._executing = False

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
