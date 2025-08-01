#!/usr/bin/env python3
"""
Communication Logger for Paris Quadcopter Swarm
Tracks and logs all swarm communication and coordination
"""

import rospy
import json
import time
import os
from std_msgs.msg import String
from datetime import datetime

class CommunicationLogger:
    """
    Logs all communication between quadcopters and swarm coordinator
    """
    
    def __init__(self):
        self.node_name = "communication_logger"
        
        # Initialize ROS node
        rospy.init_node(self.node_name, anonymous=True)
        
        # Logging parameters
        self.log_file = rospy.get_param("~log_file", "swarm_communication.log")
        self.log_level = rospy.get_param("~log_level", "info")
        
        # Create logs directory if it doesn't exist
        log_dir = os.path.dirname(self.log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Communication tracking
        self.communication_count = 0
        self.quadcopter_messages = {}
        self.swarm_messages = []
        
        # Setup ROS subscribers
        self.setup_subscribers()
        
        # Start logging
        self.start_logging()
        
        rospy.loginfo(f"📝 Communication Logger initialized - logging to {self.log_file}")
    
    def setup_subscribers(self):
        """Setup ROS subscribers for communication tracking"""
        
        # Swarm status messages
        rospy.Subscriber('/swarm/status', String, self.swarm_status_callback)
        
        # Mission status messages
        rospy.Subscriber('/swarm/mission_status', String, self.mission_status_callback)
        
        # Swarm commands
        rospy.Subscriber('/swarm/commands', String, self.swarm_commands_callback)
        
        # Individual quadcopter messages
        for i in range(5):  # Track 5 quadcopters
            quad_id = f"quad_{i+1:03d}"
            rospy.Subscriber(f'/quad_{quad_id}/status', String, 
                           lambda msg, qid=quad_id: self.quadcopter_status_callback(msg, qid))
    
    def swarm_status_callback(self, msg):
        """Handle swarm status messages"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # Parse message
            data = msg.data.split(':')
            if len(data) >= 3:
                quad_id = data[0]
                state = data[1]
                position = data[2]
                
                log_entry = {
                    'timestamp': timestamp,
                    'type': 'swarm_status',
                    'quad_id': quad_id,
                    'state': state,
                    'position': position,
                    'message': msg.data
                }
                
                self.log_communication(log_entry)
                
        except Exception as e:
            rospy.logwarn(f"📝 Error parsing swarm status: {e}")
    
    def mission_status_callback(self, msg):
        """Handle mission status messages"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # Parse JSON message
            status_data = json.loads(msg.data)
            
            log_entry = {
                'timestamp': timestamp,
                'type': 'mission_status',
                'swarm_state': status_data.get('swarm_state', 'unknown'),
                'mission_type': status_data.get('mission_type', 'unknown'),
                'mission_progress': status_data.get('mission_progress', 0.0),
                'quadcopter_count': status_data.get('quadcopter_count', 0),
                'discovered_targets': len(status_data.get('discovered_targets', [])),
                'message': msg.data
            }
            
            self.log_communication(log_entry)
            
        except Exception as e:
            rospy.logwarn(f"📝 Error parsing mission status: {e}")
    
    def swarm_commands_callback(self, msg):
        """Handle swarm command messages"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            log_entry = {
                'timestamp': timestamp,
                'type': 'swarm_command',
                'command': msg.data,
                'message': msg.data
            }
            
            self.log_communication(log_entry)
            
        except Exception as e:
            rospy.logwarn(f"📝 Error parsing swarm command: {e}")
    
    def quadcopter_status_callback(self, msg, quad_id):
        """Handle individual quadcopter status messages"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            log_entry = {
                'timestamp': timestamp,
                'type': 'quadcopter_status',
                'quad_id': quad_id,
                'message': msg.data
            }
            
            self.log_communication(log_entry)
            
        except Exception as e:
            rospy.logwarn(f"📝 Error parsing quadcopter status: {e}")
    
    def log_communication(self, log_entry):
        """Log communication entry to file"""
        try:
            # Increment communication count
            self.communication_count += 1
            
            # Add to swarm messages
            self.swarm_messages.append(log_entry)
            
            # Write to log file
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
            
            # Log to console based on level
            if self.log_level == 'debug':
                rospy.loginfo(f"📝 Communication: {log_entry['type']} - {log_entry.get('message', '')}")
            elif self.log_level == 'info' and log_entry['type'] in ['swarm_command', 'mission_status']:
                rospy.loginfo(f"📝 {log_entry['type']}: {log_entry.get('message', '')}")
            
        except Exception as e:
            rospy.logerr(f"📝 Error writing to log file: {e}")
    
    def start_logging(self):
        """Start the communication logging"""
        # Write header to log file
        header = {
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'type': 'log_header',
            'message': 'Paris Swarm Communication Log Started',
            'log_level': self.log_level
        }
        
        try:
            with open(self.log_file, 'w') as f:
                f.write(json.dumps(header) + '\n')
            
            rospy.loginfo(f"📝 Communication logging started - {self.log_file}")
            
        except Exception as e:
            rospy.logerr(f"📝 Error creating log file: {e}")
    
    def get_communication_stats(self):
        """Get communication statistics"""
        stats = {
            'total_messages': self.communication_count,
            'message_types': {},
            'quadcopter_messages': len(self.quadcopter_messages),
            'swarm_messages': len(self.swarm_messages)
        }
        
        # Count message types
        for msg in self.swarm_messages:
            msg_type = msg.get('type', 'unknown')
            stats['message_types'][msg_type] = stats['message_types'].get(msg_type, 0) + 1
        
        return stats
    
    def print_stats(self):
        """Print communication statistics"""
        stats = self.get_communication_stats()
        
        rospy.loginfo("📊 Communication Statistics:")
        rospy.loginfo(f"   Total Messages: {stats['total_messages']}")
        rospy.loginfo(f"   Swarm Messages: {stats['swarm_messages']}")
        rospy.loginfo(f"   Quadcopter Messages: {stats['quadcopter_messages']}")
        
        rospy.loginfo("   Message Types:")
        for msg_type, count in stats['message_types'].items():
            rospy.loginfo(f"     {msg_type}: {count}")

if __name__ == "__main__":
    try:
        logger = CommunicationLogger()
        
        # Print stats every 30 seconds
        rate = rospy.Rate(1/30)  # 30 second interval
        
        while not rospy.is_shutdown():
            logger.print_stats()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass 