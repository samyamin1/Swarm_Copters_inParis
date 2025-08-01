#!/usr/bin/env python3
"""
Advanced AI Flight Controller for Quadcopter Swarm
Handles individual quadcopter flight dynamics, navigation, and control
Integrates with SMOLLM AI for real-time decision making
"""

import rospy
import numpy as np
import requests
import json
import time
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, TransformStamped
import tf2_geometry_msgs
from enum import Enum
import math

class FlightState(Enum):
    GROUNDED = "grounded"
    TAKEOFF = "takeoff"
    HOVER = "hover"
    NAVIGATING = "navigating"
    LANDING = "landing"
    EMERGENCY = "emergency"

class AIFlightController:
    """
    Advanced AI-based flight controller for individual quadcopters
    Handles flight dynamics, navigation, and autonomous decision making
    """
    
    def __init__(self, quad_id, initial_position=[0, 0, 0]):
        self.quad_id = quad_id
        self.node_name = f"flight_controller_{quad_id}"
        
        # Initialize ROS node
        rospy.init_node(self.node_name, anonymous=True)
        
        # Flight state and parameters
        self.current_state = FlightState.GROUNDED
        self.target_position = np.array(initial_position)
        self.current_position = np.array(initial_position)
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        
        # Flight control parameters
        self.max_velocity = 5.0  # m/s
        self.max_acceleration = 2.0  # m/s²
        self.hover_height = 10.0  # meters
        self.safety_radius = 2.0  # meters
        
        # AI control gains
        self.position_gain = 1.0
        self.velocity_gain = 0.5
        self.yaw_gain = 0.8
        
        # SMOLLM AI Integration
        self.ai_service_url = "http://ai_decision_service:5000"
        self.ai_timeout = 3.0  # seconds
        self.last_ai_decision = "HOVER"
        self.ai_decision_cache = {}
        
        # Publishers and Subscribers
        self.setup_ros_communication()
        
        # Control loop
        self.control_rate = 50  # Hz
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0/self.control_rate), 
            self.control_loop
        )
        
        rospy.loginfo(f"🚁 AI Flight Controller {quad_id} initialized with SMOLLM integration")
    
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            f"/quad_{self.quad_id}/cmd_vel", 
            TwistStamped, 
            queue_size=10
        )
        
        self.target_pose_pub = rospy.Publisher(
            f"/quad_{self.quad_id}/target_pose", 
            PoseStamped, 
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            f"/quad_{self.quad_id}/odom", 
            Odometry, 
            self.odometry_callback
        )
        
        rospy.Subscriber(
            f"/quad_{self.quad_id}/imu", 
            Imu, 
            self.imu_callback
        )
        
        rospy.Subscriber(
            f"/quad_{self.quad_id}/battery", 
            BatteryState, 
            self.battery_callback
        )
        
        # Swarm communication
        self.swarm_status_pub = rospy.Publisher(
            "/swarm/status", 
            rospy.String, 
            queue_size=10
        )
        
        rospy.Subscriber(
            "/swarm/commands", 
            rospy.String, 
            self.swarm_command_callback
        )
    
    def odometry_callback(self, msg):
        """Handle odometry updates"""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        self.current_orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
    
    def imu_callback(self, msg):
        """Handle IMU data for attitude control"""
        # Process IMU data for attitude estimation
        pass
    
    def battery_callback(self, msg):
        """Handle battery status"""
        if msg.percentage < 0.2:  # 20% battery
            self.emergency_landing()
    
    def swarm_command_callback(self, msg):
        """Handle swarm-level commands"""
        command = msg.data
        rospy.loginfo(f"🚁 Quad {self.quad_id} received command: {command}")
        
        if "TAKEOFF" in command:
            self.takeoff()
        elif "LAND" in command:
            self.land()
        elif "EMERGENCY" in command:
            self.emergency_landing()
    
    def control_loop(self, event):
        """Main control loop - runs at 50Hz"""
        try:
            # Get AI decision for current scenario
            ai_decision = self.get_ai_flight_decision()
            
            # Apply AI decision to state machine
            if ai_decision == "TAKEOFF" and self.current_state == FlightState.GROUNDED:
                self.current_state = FlightState.TAKEOFF
            elif ai_decision == "EMERGENCY_LAND":
                self.current_state = FlightState.EMERGENCY
            elif ai_decision == "LAND":
                self.current_state = FlightState.LANDING
            
            # State machine for flight control
            if self.current_state == FlightState.GROUNDED:
                self.grounded_control()
            elif self.current_state == FlightState.TAKEOFF:
                self.takeoff_control()
            elif self.current_state == FlightState.HOVER:
                self.hover_control()
            elif self.current_state == FlightState.NAVIGATING:
                self.navigation_control()
            elif self.current_state == FlightState.LANDING:
                self.landing_control()
            elif self.current_state == FlightState.EMERGENCY:
                self.emergency_control()
            
            # Publish status to swarm
            self.publish_status()
            
        except Exception as e:
            rospy.logerr(f"🚁 Control loop error: {e}")
            self.emergency_landing()
    
    def grounded_control(self):
        """Control when quadcopter is on the ground"""
        # Wait for takeoff command
        pass
    
    def takeoff_control(self):
        """Control during takeoff phase"""
        target_height = self.hover_height
        current_height = self.current_position[2]
        
        if current_height < target_height:
            # Continue ascending
            velocity_cmd = self.calculate_velocity_command(
                target=np.array([self.current_position[0], self.current_position[1], target_height])
            )
            self.publish_velocity_command(velocity_cmd)
        else:
            # Transition to hover
            self.current_state = FlightState.HOVER
            rospy.loginfo(f"🚁 Quad {self.quad_id} reached hover height")
    
    def hover_control(self):
        """Control during hover phase"""
        # Maintain position with small corrections
        velocity_cmd = self.calculate_velocity_command(self.current_position)
        self.publish_velocity_command(velocity_cmd)
    
    def navigation_control(self):
        """Control during navigation phase"""
        # Navigate to target position
        velocity_cmd = self.calculate_velocity_command(self.target_position)
        self.publish_velocity_command(velocity_cmd)
        
        # Check if target reached
        distance = np.linalg.norm(self.current_position - self.target_position)
        if distance < 1.0:  # Within 1 meter
            self.current_state = FlightState.HOVER
            rospy.loginfo(f"🚁 Quad {self.quad_id} reached target")
    
    def landing_control(self):
        """Control during landing phase"""
        target_height = 0.1  # Near ground
        current_height = self.current_position[2]
        
        if current_height > target_height:
            # Continue descending
            velocity_cmd = self.calculate_velocity_command(
                target=np.array([self.current_position[0], self.current_position[1], target_height])
            )
            self.publish_velocity_command(velocity_cmd)
        else:
            # Landed
            self.current_state = FlightState.GROUNDED
            rospy.loginfo(f"🚁 Quad {self.quad_id} landed successfully")
    
    def emergency_control(self):
        """Emergency control - immediate landing"""
        self.landing_control()
    
    def calculate_velocity_command(self, target):
        """Calculate velocity command using PID control"""
        error = target - self.current_position
        velocity_error = np.array([0.0, 0.0, 0.0]) - self.current_velocity
        
        # Simple PID control
        velocity_cmd = (
            self.position_gain * error + 
            self.velocity_gain * velocity_error
        )
        
        # Limit velocity
        velocity_cmd = np.clip(velocity_cmd, -self.max_velocity, self.max_velocity)
        
        return velocity_cmd
    
    def publish_velocity_command(self, velocity_cmd):
        """Publish velocity command to quadcopter"""
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.header.frame_id = f"quad_{self.quad_id}_base_link"
        
        cmd_msg.twist.linear.x = velocity_cmd[0]
        cmd_msg.twist.linear.y = velocity_cmd[1]
        cmd_msg.twist.linear.z = velocity_cmd[2]
        
        self.cmd_vel_pub.publish(cmd_msg)
    
    def takeoff(self):
        """Initiate takeoff sequence"""
        if self.current_state == FlightState.GROUNDED:
            self.current_state = FlightState.TAKEOFF
            rospy.loginfo(f"🚁 Quad {self.quad_id} starting takeoff")
    
    def land(self):
        """Initiate landing sequence"""
        if self.current_state != FlightState.GROUNDED:
            self.current_state = FlightState.LANDING
            rospy.loginfo(f"🚁 Quad {self.quad_id} starting landing")
    
    def emergency_landing(self):
        """Emergency landing"""
        self.current_state = FlightState.EMERGENCY
        rospy.logwarn(f"🚁 Quad {self.quad_id} emergency landing!")
    
    def set_target_position(self, x, y, z):
        """Set target position for navigation"""
        self.target_position = np.array([x, y, z])
        if self.current_state == FlightState.HOVER:
            self.current_state = FlightState.NAVIGATING
            rospy.loginfo(f"🚁 Quad {self.quad_id} navigating to {self.target_position}")
    
    def get_ai_flight_decision(self):
        """Get AI flight decision from SMOLLM service"""
        try:
            # Create scenario description
            scenario = self._create_ai_scenario()
            
            # Check cache first
            if scenario in self.ai_decision_cache:
                return self.ai_decision_cache[scenario]
            
            # Call AI service
            response = requests.post(
                f"{self.ai_service_url}/flight_decision",
                json={
                    "quad_id": self.quad_id,
                    "scenario": scenario
                },
                timeout=self.ai_timeout
            )
            
            if response.status_code == 200:
                result = response.json()
                decision = result.get("decision", "HOVER")
                self.ai_decision_cache[scenario] = decision
                self.last_ai_decision = decision
                return decision
            else:
                rospy.logwarn(f"🚁 AI service error: {response.status_code}")
                return self.last_ai_decision
                
        except Exception as e:
            rospy.logwarn(f"🚁 AI decision error: {e}")
            return self.last_ai_decision
    
    def _create_ai_scenario(self):
        """Create scenario description for AI decision making"""
        scenario = f"""Quadcopter {self.quad_id} at position {self.current_position.tolist()}
Current state: {self.current_state.value}
Battery: 85%  # Simulated
Target position: {self.target_position.tolist()}
Velocity: {self.current_velocity.tolist()}"""
        
        # Add obstacle information (simulated)
        if np.linalg.norm(self.current_position - self.target_position) < 5.0:
            scenario += "\nTarget nearby"
        
        # Add emergency conditions
        if self.current_state == FlightState.EMERGENCY:
            scenario += "\nEmergency situation detected"
        
        return scenario
    
    def publish_status(self):
        """Publish status to swarm"""
        status_msg = rospy.String()
        status_msg.data = f"{self.quad_id}:{self.current_state.value}:{self.current_position.tolist()}"
        self.swarm_status_pub.publish(status_msg)

if __name__ == "__main__":
    try:
        # Example usage
        quad_id = rospy.get_param("~quad_id", "quad_001")
        initial_pos = rospy.get_param("~initial_position", [0, 0, 0])
        
        controller = AIFlightController(quad_id, initial_pos)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass 