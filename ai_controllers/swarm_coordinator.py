#!/usr/bin/env python3
"""
AI Swarm Coordinator for Quadcopter Swarm
Manages distributed decision making, mission planning, and swarm coordination
Integrates with SMOLLM AI for real-time swarm decisions
"""

import rospy
import numpy as np
import json
import threading
import requests
import time
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from enum import Enum
import math
from collections import defaultdict

class MissionType(Enum):
    SEARCH_AND_RESCUE = "search_and_rescue"
    FORMATION_FLYING = "formation_flying"
    SURVEILLANCE = "surveillance"
    EMERGENCY_RESPONSE = "emergency_response"

class SwarmState(Enum):
    IDLE = "idle"
    DEPLOYING = "deploying"
    MISSION_ACTIVE = "mission_active"
    RETURNING = "returning"
    EMERGENCY = "emergency"

class AISwarmCoordinator:
    """
    Advanced AI swarm coordinator for quadcopter swarm
    Handles distributed decision making, mission planning, and coordination
    """
    
    def __init__(self, swarm_size=5):
        self.swarm_size = swarm_size
        self.node_name = "swarm_coordinator"
        
        # Initialize ROS node
        rospy.init_node(self.node_name, anonymous=True)
        
        # Swarm state and mission
        self.swarm_state = SwarmState.IDLE
        self.current_mission = MissionType.SEARCH_AND_RESCUE
        self.mission_active = False
        
        # Quadcopter tracking
        self.quadcopters = {}
        self.quad_positions = {}
        self.quad_states = {}
        self.quad_batteries = {}
        
        # Mission parameters
        self.search_area = {
            'center': [0, 0, 15],  # Paris center coordinates
            'radius': 100,  # meters
            'height_range': [10, 50]  # meters
        }
        
        self.formation_config = {
            'type': 'v_formation',
            'spacing': 10,  # meters
            'height_spacing': 5  # meters
        }
        
        # AI coordination parameters
        self.coordination_rate = 10  # Hz
        self.communication_range = 200  # meters
        self.safety_distance = 5  # meters
        
        # SMOLLM AI Integration
        self.ai_service_url = "http://ai_decision_service:5000"
        self.ai_timeout = 3.0  # seconds
        self.last_ai_coordination = {"action": "maintain_formation", "targets": []}
        
        # Mission targets and objectives
        self.mission_targets = []
        self.discovered_targets = []
        self.mission_progress = 0.0
        
        # Setup ROS communication
        self.setup_ros_communication()
        
        # Start coordination loop
        self.coordination_timer = rospy.Timer(
            rospy.Duration(1.0/self.coordination_rate),
            self.coordination_loop
        )
        
        # Initialize quadcopters
        self.initialize_swarm()
        
        rospy.loginfo(f"🤖 AI Swarm Coordinator initialized with {swarm_size} quadcopters")
    
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers for swarm coordination"""
        
        # Publishers
        self.swarm_command_pub = rospy.Publisher(
            "/swarm/commands",
            String,
            queue_size=10
        )
        
        self.mission_status_pub = rospy.Publisher(
            "/swarm/mission_status",
            String,
            queue_size=10
        )
        
        self.formation_targets_pub = rospy.Publisher(
            "/swarm/formation_targets",
            String,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            "/swarm/status",
            String,
            self.swarm_status_callback
        )
        
        rospy.Subscriber(
            "/mission/commands",
            String,
            self.mission_command_callback
        )
        
        # Mission control
        rospy.Subscriber(
            "/mission/start",
            String,
            self.start_mission_callback
        )
        
        rospy.Subscriber(
            "/mission/stop",
            String,
            self.stop_mission_callback
        )
    
    def initialize_swarm(self):
        """Initialize the quadcopter swarm"""
        base_positions = [
            [0, 0, 0],      # Center
            [10, 0, 0],     # Right
            [-10, 0, 0],    # Left
            [0, 10, 0],     # Front
            [0, -10, 0]     # Back
        ]
        
        for i in range(self.swarm_size):
            quad_id = f"quad_{i+1:03d}"
            self.quadcopters[quad_id] = {
                'id': quad_id,
                'position': np.array(base_positions[i % len(base_positions)]),
                'state': 'grounded',
                'battery': 100.0,
                'assigned_target': None,
                'formation_position': i
            }
            self.quad_positions[quad_id] = np.array(base_positions[i % len(base_positions)])
            self.quad_states[quad_id] = 'grounded'
            self.quad_batteries[quad_id] = 100.0
        
        rospy.loginfo(f"🤖 Swarm initialized with {self.swarm_size} quadcopters")
    
    def swarm_status_callback(self, msg):
        """Handle status updates from individual quadcopters"""
        try:
            data = msg.data.split(':')
            if len(data) >= 3:
                quad_id = data[0]
                state = data[1]
                position_str = data[2]
                
                # Parse position
                position = eval(position_str)  # Convert string to list
                self.quad_positions[quad_id] = np.array(position)
                self.quad_states[quad_id] = state
                
                # Update quadcopter data
                if quad_id in self.quadcopters:
                    self.quadcopters[quad_id]['position'] = np.array(position)
                    self.quadcopters[quad_id]['state'] = state
                    
        except Exception as e:
            rospy.logwarn(f"🤖 Error parsing swarm status: {e}")
    
    def mission_command_callback(self, msg):
        """Handle mission-level commands"""
        command = msg.data
        rospy.loginfo(f"🤖 Received mission command: {command}")
        
        if "TAKEOFF_ALL" in command:
            self.takeoff_swarm()
        elif "LAND_ALL" in command:
            self.land_swarm()
        elif "FORMATION" in command:
            self.activate_formation_flying()
        elif "SEARCH_MODE" in command:
            self.activate_search_mode()
        elif "EMERGENCY" in command:
            self.emergency_protocol()
    
    def start_mission_callback(self, msg):
        """Start a new mission"""
        mission_data = json.loads(msg.data)
        mission_type = mission_data.get('type', 'search_and_rescue')
        search_area = mission_data.get('search_area', self.search_area)
        
        self.current_mission = MissionType(mission_type)
        self.search_area.update(search_area)
        
        rospy.loginfo(f"🤖 Starting mission: {self.current_mission.value}")
        self.start_mission()
    
    def stop_mission_callback(self, msg):
        """Stop current mission"""
        rospy.loginfo("🤖 Stopping mission")
        self.stop_mission()
    
    def coordination_loop(self, event):
        """Main coordination loop - runs at 10Hz"""
        try:
            # Get AI coordination decision
            ai_coordination = self.get_ai_swarm_coordination()
            
            # Apply AI coordination decision
            if ai_coordination["action"] == "expand_search":
                self.current_mission = MissionType.SEARCH_AND_RESCUE
            elif ai_coordination["action"] == "maintain_formation":
                self.current_mission = MissionType.FORMATION_FLYING
            elif ai_coordination["action"] == "emergency_protocol":
                self.emergency_protocol()
            
            # Update swarm state based on current mission
            if self.mission_active:
                if self.current_mission == MissionType.SEARCH_AND_RESCUE:
                    self.coordinate_search_and_rescue()
                elif self.current_mission == MissionType.FORMATION_FLYING:
                    self.coordinate_formation_flying()
                elif self.current_mission == MissionType.SURVEILLANCE:
                    self.coordinate_surveillance()
                elif self.current_mission == MissionType.EMERGENCY_RESPONSE:
                    self.coordinate_emergency_response()
            
            # Check for collisions and safety
            self.check_safety_distances()
            
            # Publish mission status
            self.publish_mission_status()
            
        except Exception as e:
            rospy.logerr(f"🤖 Coordination loop error: {e}")
    
    def coordinate_search_and_rescue(self):
        """Coordinate search and rescue mission"""
        # Generate search patterns
        search_patterns = self.generate_search_patterns()
        
        # Assign quadcopters to search areas
        for quad_id, pattern in search_patterns.items():
            if quad_id in self.quadcopters:
                target_pos = self.calculate_search_target(pattern)
                self.assign_target_to_quad(quad_id, target_pos)
        
        # Check for discovered targets
        self.process_discovered_targets()
        
        # Update mission progress
        self.update_mission_progress()
    
    def coordinate_formation_flying(self):
        """Coordinate formation flying"""
        formation_targets = self.calculate_formation_positions()
        
        # Assign formation positions to quadcopters
        for quad_id, target_pos in formation_targets.items():
            if quad_id in self.quadcopters:
                self.assign_target_to_quad(quad_id, target_pos)
    
    def coordinate_surveillance(self):
        """Coordinate surveillance mission"""
        # Implement surveillance coordination logic
        pass
    
    def coordinate_emergency_response(self):
        """Coordinate emergency response"""
        # Implement emergency response coordination
        pass
    
    def generate_search_patterns(self):
        """Generate search patterns for the swarm"""
        patterns = {}
        
        # Spiral search pattern
        center = np.array(self.search_area['center'])
        radius = self.search_area['radius']
        
        for i, quad_id in enumerate(self.quadcopters.keys()):
            angle = (2 * math.pi * i) / self.swarm_size
            distance = radius * (0.3 + 0.7 * (i / self.swarm_size))
            
            x = center[0] + distance * math.cos(angle)
            y = center[1] + distance * math.sin(angle)
            z = center[2] + 10 * (i % 3)  # Staggered heights
            
            patterns[quad_id] = {
                'type': 'spiral',
                'center': [x, y, z],
                'radius': radius / self.swarm_size,
                'angle': angle
            }
        
        return patterns
    
    def calculate_search_target(self, pattern):
        """Calculate target position for search pattern"""
        center = np.array(pattern['center'])
        radius = pattern['radius']
        angle = pattern['angle']
        
        # Dynamic search movement
        time_factor = rospy.Time.now().to_sec() * 0.1
        dynamic_angle = angle + time_factor
        
        x = center[0] + radius * math.cos(dynamic_angle)
        y = center[1] + radius * math.sin(dynamic_angle)
        z = center[2]
        
        return np.array([x, y, z])
    
    def calculate_formation_positions(self):
        """Calculate formation positions for the swarm"""
        formation_targets = {}
        
        # V-formation calculation
        leader_pos = np.array([0, 0, 20])  # Leader position
        spacing = self.formation_config['spacing']
        height_spacing = self.formation_config['height_spacing']
        
        for i, quad_id in enumerate(self.quadcopters.keys()):
            if i == 0:  # Leader
                formation_targets[quad_id] = leader_pos
            else:
                # Calculate wing positions
                wing_offset = spacing * (i // 2 + 1)
                height_offset = height_spacing * (i % 2)
                
                if i % 2 == 1:  # Right wing
                    x = leader_pos[0] + wing_offset
                    y = leader_pos[1] + wing_offset
                else:  # Left wing
                    x = leader_pos[0] - wing_offset
                    y = leader_pos[1] + wing_offset
                
                z = leader_pos[2] + height_offset
                formation_targets[quad_id] = np.array([x, y, z])
        
        return formation_targets
    
    def assign_target_to_quad(self, quad_id, target_position):
        """Assign target position to specific quadcopter"""
        if quad_id in self.quadcopters:
            self.quadcopters[quad_id]['assigned_target'] = target_position
            
            # Send target to quadcopter
            target_msg = String()
            target_msg.data = json.dumps({
                'quad_id': quad_id,
                'target': target_position.tolist(),
                'mission_type': self.current_mission.value
            })
            
            # Publish to specific quadcopter
            rospy.Publisher(f"/quad_{quad_id}/target", String, queue_size=1).publish(target_msg)
    
    def check_safety_distances(self):
        """Check and maintain safety distances between quadcopters"""
        quad_ids = list(self.quad_positions.keys())
        
        for i, quad1_id in enumerate(quad_ids):
            for j, quad2_id in enumerate(quad_ids[i+1:], i+1):
                pos1 = self.quad_positions[quad1_id]
                pos2 = self.quad_positions[quad2_id]
                
                distance = np.linalg.norm(pos1 - pos2)
                
                if distance < self.safety_distance:
                    rospy.logwarn(f"🤖 Safety distance violation: {quad1_id} and {quad2_id} at {distance:.2f}m")
                    self.adjust_for_safety_collision(quad1_id, quad2_id)
    
    def adjust_for_safety_collision(self, quad1_id, quad2_id):
        """Adjust positions to maintain safety distance"""
        pos1 = self.quad_positions[quad1_id]
        pos2 = self.quad_positions[quad2_id]
        
        # Calculate separation vector
        separation_vector = pos1 - pos2
        separation_vector = separation_vector / np.linalg.norm(separation_vector)
        
        # Adjust positions
        adjustment = separation_vector * (self.safety_distance - np.linalg.norm(pos1 - pos2)) / 2
        
        new_pos1 = pos1 + adjustment
        new_pos2 = pos2 - adjustment
        
        # Assign new positions
        self.assign_target_to_quad(quad1_id, new_pos1)
        self.assign_target_to_quad(quad2_id, new_pos2)
    
    def process_discovered_targets(self):
        """Process targets discovered during search"""
        # Simulate target discovery
        if rospy.Time.now().to_sec() % 30 < 1:  # Every 30 seconds
            # Simulate finding a target
            target_pos = np.random.uniform(-50, 50, 3)
            target_pos[2] = 0  # Ground level
            
            self.discovered_targets.append({
                'position': target_pos,
                'time': rospy.Time.now().to_sec(),
                'assigned_quad': None
            })
            
            rospy.loginfo(f"🤖 Target discovered at {target_pos}")
    
    def update_mission_progress(self):
        """Update mission progress based on search coverage"""
        # Calculate search coverage
        total_area = math.pi * self.search_area['radius'] ** 2
        covered_area = len(self.discovered_targets) * 100  # Simplified
        
        self.mission_progress = min(covered_area / total_area, 1.0)
    
    def takeoff_swarm(self):
        """Initiate takeoff for entire swarm"""
        rospy.loginfo("🤖 Initiating swarm takeoff")
        
        # Send takeoff command to all quadcopters
        takeoff_msg = String()
        takeoff_msg.data = "TAKEOFF_ALL"
        self.swarm_command_pub.publish(takeoff_msg)
        
        self.swarm_state = SwarmState.DEPLOYING
    
    def land_swarm(self):
        """Initiate landing for entire swarm"""
        rospy.loginfo("🤖 Initiating swarm landing")
        
        # Send landing command to all quadcopters
        land_msg = String()
        land_msg.data = "LAND_ALL"
        self.swarm_command_pub.publish(land_msg)
        
        self.swarm_state = SwarmState.RETURNING
    
    def activate_formation_flying(self):
        """Activate formation flying mode"""
        rospy.loginfo("🤖 Activating formation flying")
        self.current_mission = MissionType.FORMATION_FLYING
    
    def activate_search_mode(self):
        """Activate search and rescue mode"""
        rospy.loginfo("🤖 Activating search and rescue mode")
        self.current_mission = MissionType.SEARCH_AND_RESCUE
    
    def emergency_protocol(self):
        """Activate emergency protocol"""
        rospy.logwarn("🤖 Emergency protocol activated!")
        
        # Send emergency command to all quadcopters
        emergency_msg = String()
        emergency_msg.data = "EMERGENCY_LANDING"
        self.swarm_command_pub.publish(emergency_msg)
        
        self.swarm_state = SwarmState.EMERGENCY
    
    def start_mission(self):
        """Start the current mission"""
        self.mission_active = True
        self.swarm_state = SwarmState.MISSION_ACTIVE
        rospy.loginfo(f"🤖 Mission started: {self.current_mission.value}")
    
    def stop_mission(self):
        """Stop the current mission"""
        self.mission_active = False
        self.swarm_state = SwarmState.IDLE
        rospy.loginfo("🤖 Mission stopped")
    
    def get_ai_swarm_coordination(self):
        """Get AI swarm coordination decision from SMOLLM service"""
        try:
            # Create swarm state for AI
            swarm_state = {
                'swarm_state': self.swarm_state.value,
                'mission_type': self.current_mission.value,
                'mission_active': self.mission_active,
                'mission_progress': self.mission_progress,
                'quadcopter_count': len(self.quadcopters),
                'discovered_targets': len(self.discovered_targets),
                'quadcopter_positions': self.quad_positions
            }
            
            # Call AI service
            response = requests.post(
                f"{self.ai_service_url}/swarm_coordination",
                json={
                    "swarm_state": swarm_state
                },
                timeout=self.ai_timeout
            )
            
            if response.status_code == 200:
                result = response.json()
                coordination = result.get("coordination", {})
                self.last_ai_coordination = coordination
                return coordination
            else:
                rospy.logwarn(f"🤖 AI coordination service error: {response.status_code}")
                return self.last_ai_coordination
                
        except Exception as e:
            rospy.logwarn(f"🤖 AI coordination error: {e}")
            return self.last_ai_coordination
    
    def publish_mission_status(self):
        """Publish current mission status"""
        status_data = {
            'swarm_state': self.swarm_state.value,
            'mission_type': self.current_mission.value,
            'mission_active': self.mission_active,
            'mission_progress': self.mission_progress,
            'quadcopter_count': len(self.quadcopters),
            'discovered_targets': len(self.discovered_targets),
            'timestamp': rospy.Time.now().to_sec()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.mission_status_pub.publish(status_msg)

if __name__ == "__main__":
    try:
        swarm_size = rospy.get_param("~swarm_size", 5)
        coordinator = AISwarmCoordinator(swarm_size)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass 