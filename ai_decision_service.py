#!/usr/bin/env python3
"""
AI Decision Service for Paris Quadcopter Swarm
Integrates SMOLLM for real-time AI decision making
"""

import os
import json
import time
import urllib.request
import urllib.error
from typing import Dict, List, Tuple
from flask import Flask, request, jsonify
from flask_cors import CORS

class SmollmAI:
    """SMOLLM AI integration for quadcopter decision making"""
    
    def __init__(self, ollama_url: str = "http://ollama:11434"):
        """Initialize SMOLLM AI system"""
        self.ollama_url = ollama_url
        self.model = "smollm:135m"  # Fast, lightweight model
        self.cache = {}
        self.timeout = 3.0  # 3 second timeout for real-time robotics
        
        print("🚁 SMOLLM AI initialized for Paris Swarm")
        print(f"🤖 Model: {self.model}")
        print(f"⏱️  Timeout: {self.timeout}s")
        print(f"📊 Model size: 135M parameters (fast!)")
    
    def get_flight_decision(self, scenario: str) -> str:
        """Get AI flight decision using SMOLLM"""
        start_time = time.time()
        
        # Check cache first (instant)
        if scenario in self.cache:
            return self.cache[scenario]
        
        # Try AI with intelligent fallback
        try:
            decision = self._get_intelligent_flight_decision(scenario)
            self.cache[scenario] = decision
            response_time = time.time() - start_time
            print(f"🚁 Flight Decision: {decision} (time: {response_time:.3f}s)")
            return decision
        except Exception as e:
            print(f"❌ AI Error: {e}")
            # Use intelligent fallback
            decision = self._get_intelligent_fallback(scenario)
            self.cache[scenario] = decision
            return decision
    
    def _get_intelligent_flight_decision(self, scenario: str) -> str:
        """Get flight decision using SMOLLM + intelligent analysis"""
        # First, try to get AI response
        try:
            response = self._call_smollm_api(scenario)
            print(f"   📝 SMOLLM Response: '{response[:50]}...'")
            
            # Extract command from AI response
            command = self._extract_flight_command_from_ai(response)
            if command != "HOVER":  # If AI gave a specific command
                return command
                
        except Exception as e:
            print(f"   ⚠️  SMOLLM call failed: {e}")
        
        # Fallback to intelligent analysis
        return self._get_intelligent_fallback(scenario)
    
    def _call_smollm_api(self, scenario: str) -> str:
        """Call SMOLLM with optimized parameters for flight decisions"""
        prompt = f"""Flight Scenario: {scenario}
You are a quadcopter in Paris. Reply with ONLY: TAKEOFF, HOVER, MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, LAND, or EMERGENCY_LAND"""
        
        url = f"{self.ollama_url}/api/chat"
        
        data = {
            "model": self.model,
            "messages": [{"role": "user", "content": prompt}],
            "stream": False,
            "options": {
                "temperature": 0,
                "num_predict": 5,
                "top_k": 1,
                "top_p": 0.1,
            }
        }
        
        json_data = json.dumps(data).encode('utf-8')
        req = urllib.request.Request(
            url,
            data=json_data,
            headers={'Content-Type': 'application/json'}
        )
        
        with urllib.request.urlopen(req, timeout=self.timeout) as response:
            result = json.loads(response.read().decode('utf-8'))
            return result["message"]["content"].strip()
    
    def _extract_flight_command_from_ai(self, response: str) -> str:
        """Extract flight command from SMOLLM response"""
        response_upper = response.upper()
        
        # Flight commands for quadcopters
        flight_commands = [
            "TAKEOFF", "HOVER", "MOVE_FORWARD", "TURN_LEFT", 
            "TURN_RIGHT", "LAND", "EMERGENCY_LAND", "ASCEND", 
            "DESCEND", "FORWARD", "BACKWARD", "LEFT", "RIGHT"
        ]
        
        for command in flight_commands:
            if command in response_upper:
                return command
        
        return "HOVER"  # Default safe action
    
    def _get_intelligent_fallback(self, scenario: str) -> str:
        """Intelligent fallback for flight decisions"""
        scenario_lower = scenario.lower()
        
        # Flight-specific rules for quadcopters
        if "target" in scenario_lower and "ahead" in scenario_lower:
            return "MOVE_FORWARD"
        elif "obstacle" in scenario_lower or "building" in scenario_lower:
            return "TURN_LEFT"
        elif "search" in scenario_lower or "look" in scenario_lower:
            return "HOVER"
        elif "land" in scenario_lower or "ground" in scenario_lower:
            return "LAND"
        elif "emergency" in scenario_lower or "danger" in scenario_lower:
            return "EMERGENCY_LAND"
        elif "takeoff" in scenario_lower or "launch" in scenario_lower:
            return "TAKEOFF"
        elif "formation" in scenario_lower:
            return "HOVER"
        else:
            return "HOVER"  # Safe default for quadcopters
    
    def get_swarm_coordination_decision(self, swarm_state: Dict) -> Dict:
        """Get swarm coordination decision using SMOLLM"""
        try:
            # Create swarm scenario description
            scenario = f"""Swarm State: {swarm_state}
Quadcopters: {swarm_state.get('quadcopter_count', 0)}
Mission: {swarm_state.get('mission_type', 'unknown')}
Progress: {swarm_state.get('mission_progress', 0.0)}
Targets Found: {len(swarm_state.get('discovered_targets', []))}"""
            
            response = self._call_smollm_api(scenario)
            
            # Parse coordination decision
            decision = self._parse_coordination_decision(response)
            return decision
            
        except Exception as e:
            print(f"❌ Swarm coordination error: {e}")
            return {"action": "maintain_formation", "targets": []}
    
    def _parse_coordination_decision(self, response: str) -> Dict:
        """Parse coordination decision from SMOLLM response"""
        response_lower = response.lower()
        
        if "search" in response_lower:
            return {"action": "expand_search", "targets": []}
        elif "formation" in response_lower:
            return {"action": "maintain_formation", "targets": []}
        elif "target" in response_lower:
            return {"action": "investigate_target", "targets": ["new_target"]}
        elif "emergency" in response_lower:
            return {"action": "emergency_protocol", "targets": []}
        else:
            return {"action": "continue_mission", "targets": []}
    
    def get_stats(self) -> Dict:
        """Get AI performance statistics"""
        return {
            "model": self.model,
            "cache_size": len(self.cache),
            "timeout": self.timeout,
            "status": "operational"
        }

class AIFlightController:
    """AI-powered flight controller for individual quadcopters"""
    
    def __init__(self, quad_id: str):
        self.quad_id = quad_id
        self.ai = SmollmAI()
        self.current_state = "GROUNDED"
        self.position = [0, 0, 0]
        self.target = None
        
        print(f"🚁 AI Flight Controller {quad_id} initialized")
    
    def get_flight_command(self, sensor_data: Dict) -> str:
        """Get flight command based on sensor data"""
        # Create scenario description
        scenario = self._create_scenario_description(sensor_data)
        
        # Get AI decision
        command = self.ai.get_flight_decision(scenario)
        
        # Update state based on command
        self._update_state(command)
        
        return command
    
    def _create_scenario_description(self, sensor_data: Dict) -> str:
        """Create scenario description for AI decision making"""
        position = sensor_data.get('position', self.position)
        obstacles = sensor_data.get('obstacles', [])
        targets = sensor_data.get('targets', [])
        battery = sensor_data.get('battery', 100)
        
        scenario = f"""Quadcopter {self.quad_id} at position {position}
Current state: {self.current_state}
Battery: {battery}%
Obstacles detected: {len(obstacles)}
Targets visible: {len(targets)}"""
        
        if targets:
            scenario += f"\nNearest target at: {targets[0]}"
        if obstacles:
            scenario += f"\nNearest obstacle at: {obstacles[0]}"
        if battery < 20:
            scenario += "\nLow battery warning!"
        
        return scenario
    
    def _update_state(self, command: str):
        """Update quadcopter state based on command"""
        if command == "TAKEOFF":
            self.current_state = "TAKEOFF"
        elif command == "HOVER":
            self.current_state = "HOVER"
        elif command == "MOVE_FORWARD":
            self.current_state = "NAVIGATING"
        elif command == "LAND":
            self.current_state = "LANDING"
        elif command == "EMERGENCY_LAND":
            self.current_state = "EMERGENCY"

class AISwarmCoordinator:
    """AI-powered swarm coordinator using SMOLLM"""
    
    def __init__(self, swarm_size: int = 5):
        self.swarm_size = swarm_size
        self.ai = SmollmAI()
        self.quadcopters = {}
        
        # Initialize quadcopters
        for i in range(swarm_size):
            quad_id = f"quad_{i+1:03d}"
            self.quadcopters[quad_id] = AIFlightController(quad_id)
        
        print(f"🤖 AI Swarm Coordinator initialized with {swarm_size} quadcopters")
    
    def coordinate_swarm(self, swarm_state: Dict) -> Dict:
        """Coordinate swarm using SMOLLM AI"""
        # Get AI coordination decision
        coordination_decision = self.ai.get_swarm_coordination_decision(swarm_state)
        
        # Apply coordination to individual quadcopters
        individual_commands = {}
        for quad_id, controller in self.quadcopters.items():
            # Create individual scenario
            individual_state = {
                'position': swarm_state.get('quadcopter_positions', {}).get(quad_id, [0, 0, 0]),
                'state': controller.current_state,
                'coordination_action': coordination_decision['action']
            }
            
            command = controller.get_flight_command(individual_state)
            individual_commands[quad_id] = command
        
        return {
            'coordination': coordination_decision,
            'individual_commands': individual_commands
        }

# Flask API for AI decisions
app = Flask(__name__)
CORS(app)

# Global AI instances
smollm_ai = SmollmAI()
swarm_coordinator = AISwarmCoordinator()

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        "status": "healthy",
        "service": "paris_swarm_ai",
        "model": "smollm:135m"
    })

@app.route('/flight_decision', methods=['POST'])
def get_flight_decision():
    """Get flight decision for individual quadcopter"""
    try:
        data = request.get_json()
        scenario = data.get('scenario', '')
        quad_id = data.get('quad_id', 'unknown')
        
        decision = smollm_ai.get_flight_decision(scenario)
        
        return jsonify({
            "quad_id": quad_id,
            "decision": decision,
            "timestamp": time.time()
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/swarm_coordination', methods=['POST'])
def get_swarm_coordination():
    """Get swarm coordination decision"""
    try:
        data = request.get_json()
        swarm_state = data.get('swarm_state', {})
        
        coordination = swarm_coordinator.coordinate_swarm(swarm_state)
        
        return jsonify({
            "coordination": coordination,
            "timestamp": time.time()
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/stats', methods=['GET'])
def get_stats():
    """Get AI statistics"""
    return jsonify(smollm_ai.get_stats())

if __name__ == "__main__":
    print("🚁 Paris Swarm AI Decision Service Starting...")
    print("🤖 SMOLLM:135m Integration Active")
    print("🌍 Paris Environment Ready")
    print("🚀 AI Decision Service Running on port 5000")
    
    app.run(host='0.0.0.0', port=5000, debug=False) 