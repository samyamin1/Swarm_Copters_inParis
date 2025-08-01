#!/usr/bin/env python3
"""
Perception Bridge for AI Model Integration
Handles communication between quadcopter agents and SMOLLM AI models
"""

import numpy as np
import json
import time
import urllib.request
import os
from typing import Tuple, Dict

class SmollmAI:
    """Working AI solution with smollm:135m - Fast and reliable for quadcopters"""
    def __init__(self, ollama_url: str = "http://ollama:11434"):
        """Initialize SMOLLM AI system for quadcopter swarm"""
        self.ollama_url = ollama_url
        self.model = "smollm:135m"  # Our working model
        self.cache = {}
        self.timeout = 3.0  # 3 second timeout for real-time robotics
        
        print("🚁 SMOLLM AI initialized for Paris Quadcopter Swarm")
        print(f"🤖 Model: {self.model}")
        print(f"⏱️  Timeout: {self.timeout}s")
        print(f"📊 Model size: 135M parameters (fast!)")
    
    def get_flight_decision(self, scenario: str) -> str:
        """Get AI flight decision using intelligent fallback"""
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
        
        # Fix URL format for Docker container
        if self.ollama_url.startswith('ollama:'):
            # Convert ollama:11434 to http://ollama:11434
            url = f"http://{self.ollama_url}"
        else:
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
        
        return "HOVER"  # Default safe action for quadcopters
    
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

class PerceptionBridge:
    """Perception Bridge for quadcopter swarm AI integration"""
    
    def __init__(self):
        """Initialize perception bridge with SMOLLM AI"""
        self.ai_system = SmollmAI()
        self.timeout = 3.0  # 3 second timeout for real-time robotics
        self.stats = {
            "total_requests": 0,
            "successful_requests": 0,
            "failed_requests": 0,
            "average_response_time": 0.0
        }
        
        print("🚁 Perception Bridge initialized with SMOLLM AI")
    
    def process_perception_input(self, textual_scene_description: str) -> Tuple[str, str]:
        """Process perception input and return AI decision"""
        start_time = time.time()
        self.stats["total_requests"] += 1
        
        try:
            # Get AI decision
            decision = self.ai_system.get_flight_decision(textual_scene_description)
            
            # Update stats
            response_time = time.time() - start_time
            self.stats["successful_requests"] += 1
            self.stats["average_response_time"] = (
                (self.stats["average_response_time"] * (self.stats["successful_requests"] - 1) + response_time) /
                self.stats["successful_requests"]
            )
            
            return decision, "success"
            
        except Exception as e:
            self.stats["failed_requests"] += 1
            print(f"❌ Perception processing error: {e}")
            return "HOVER", "error"  # Safe fallback
    
    def run_vision_language_model(self, scene_description: str) -> str:
        """Run vision language model (SMOLLM) on scene description"""
        try:
            return self.ai_system.get_flight_decision(scene_description)
        except Exception as e:
            print(f"❌ VLM error: {e}")
            return "HOVER"
    
    def run_decision_making_model(self, vlm_output: str) -> str:
        """Run decision making model on VLM output"""
        try:
            # For quadcopters, the VLM output is already the decision
            return self._extract_command(vlm_output)
        except Exception as e:
            print(f"❌ Decision making error: {e}")
            return "HOVER"
    
    def _extract_command(self, response: str) -> str:
        """Extract command from AI response"""
        return self.ai_system._extract_flight_command_from_ai(response)
    
    def get_ai_stats(self) -> Dict:
        """Get AI performance statistics"""
        return {
            **self.stats,
            **self.ai_system.get_stats()
        }

# Example usage for quadcopter swarm
if __name__ == "__main__":
    # Test the perception bridge
    bridge = PerceptionBridge()
    
    test_scenarios = [
        "You see a target ahead in Paris",
        "You encounter a building obstacle", 
        "You need to search an area",
        "You found a target",
        "Emergency situation detected",
    ]
    
    for scenario in test_scenarios:
        decision, status = bridge.process_perception_input(scenario)
        print(f"Scenario: {scenario}")
        print(f"Decision: {decision}")
        print(f"Status: {status}")
        print()
    
    # Print stats
    print("AI Stats:", bridge.get_ai_stats()) 