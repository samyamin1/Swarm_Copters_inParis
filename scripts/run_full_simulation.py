#!/usr/bin/env python3
"""
Full Paris Swarm Simulation Runner
Coordinates all components for complete simulation
"""

import os
import sys
import subprocess
import time
import json
import urllib.request
import threading
from datetime import datetime

class FullSimulationRunner:
    def __init__(self):
        self.running = False
        self.timeout_seconds = 1800  # 30 minutes
        self.check_interval = 30
        
    def log(self, message, level="INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")
        
    def check_services(self):
        """Check if all required services are running"""
        self.log("🔍 Checking service status...")
        
        services = {
            "Ollama": "http://localhost:11434/api/tags",
            "AI Service": "http://localhost:5001/health",
            "Gazebo": "http://localhost:11345"
        }
        
        all_healthy = True
        
        for service_name, url in services.items():
            try:
                response = urllib.request.urlopen(url, timeout=10)
                if response.getcode() == 200:
                    self.log(f"✅ {service_name} is healthy", "SUCCESS")
                else:
                    self.log(f"❌ {service_name} is unhealthy", "ERROR")
                    all_healthy = False
            except Exception as e:
                self.log(f"❌ {service_name} is not responding: {e}", "ERROR")
                all_healthy = False
        
        return all_healthy
    
    def start_simulation_services(self):
        """Start all simulation services"""
        self.log("🚀 Starting simulation services...")
        
        try:
            # Start Gazebo and AI services
            result = subprocess.run(
                ["docker-compose", "-f", "docker-compose.working.yml", "up", "-d"],
                timeout=300
            )
            
            if result.returncode == 0:
                self.log("✅ Simulation services started", "SUCCESS")
                return True
            else:
                self.log("❌ Failed to start simulation services", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"❌ Error starting services: {e}", "ERROR")
            return False
    
    def test_ai_decisions(self):
        """Test AI decision making"""
        self.log("🤖 Testing AI decision making...")
        
        test_scenarios = [
            {
                "scenario": "Quadcopter at position [10, 5, 15], obstacle detected ahead, battery 85%",
                "quad_id": "quad_001"
            },
            {
                "scenario": "Quadcopter at position [0, 0, 10], target detected at [20, 15, 5], battery 90%",
                "quad_id": "quad_002"
            },
            {
                "scenario": "Quadcopter at position [5, -5, 8], low battery warning, need to land",
                "quad_id": "quad_003"
            }
        ]
        
        successful_decisions = 0
        
        for i, test_data in enumerate(test_scenarios, 1):
            try:
                req = urllib.request.Request(
                    "http://localhost:5001/flight_decision",
                    data=json.dumps(test_data).encode('utf-8'),
                    headers={'Content-Type': 'application/json'}
                )
                
                response = urllib.request.urlopen(req, timeout=15)
                decision_data = json.loads(response.read().decode('utf-8'))
                
                if 'decision' in decision_data:
                    self.log(f"✅ Test {i}: {decision_data['decision']} for {test_data['quad_id']}", "SUCCESS")
                    successful_decisions += 1
                else:
                    self.log(f"❌ Test {i}: No decision received", "ERROR")
                    
            except Exception as e:
                self.log(f"❌ Test {i} failed: {e}", "ERROR")
        
        self.log(f"📊 AI Decision Tests: {successful_decisions}/{len(test_scenarios)} passed", "INFO")
        return successful_decisions == len(test_scenarios)
    
    def run_mission_scenario(self):
        """Run a complete mission scenario"""
        self.log("🎯 Starting mission scenario...")
        
        mission_steps = [
            "Initializing quadcopter swarm...",
            "Performing pre-flight checks...",
            "Taking off quadcopters...",
            "Forming search pattern...",
            "Scanning Paris environment...",
            "Detecting obstacles and targets...",
            "Coordinating swarm movements...",
            "Executing search and rescue protocol...",
            "Landing quadcopters...",
            "Mission complete!"
        ]
        
        for i, step in enumerate(mission_steps, 1):
            self.log(f"Step {i}: {step}")
            
            # Simulate AI decision for this step
            try:
                scenario_data = {
                    "scenario": f"Mission step {i}: {step}",
                    "quad_id": f"quad_{i:03d}"
                }
                
                req = urllib.request.Request(
                    "http://localhost:5001/flight_decision",
                    data=json.dumps(scenario_data).encode('utf-8'),
                    headers={'Content-Type': 'application/json'}
                )
                
                response = urllib.request.urlopen(req, timeout=10)
                decision_data = json.loads(response.read().decode('utf-8'))
                
                if 'decision' in decision_data:
                    self.log(f"   🤖 AI Decision: {decision_data['decision']}", "INFO")
                
            except Exception as e:
                self.log(f"   ⚠️ AI decision failed: {e}", "WARNING")
            
            time.sleep(2)  # Simulate step duration
        
        self.log("🎉 Mission scenario completed!", "SUCCESS")
    
    def monitor_simulation(self):
        """Monitor simulation during execution"""
        self.log("🔍 Starting simulation monitoring...")
        
        start_time = time.time()
        
        while self.running:
            try:
                # Check timeout
                elapsed = time.time() - start_time
                if elapsed > self.timeout_seconds:
                    self.log(f"⏰ Simulation timeout reached ({self.timeout_seconds}s)", "WARNING")
                    break
                
                # Check services
                services_ok = self.check_services()
                if not services_ok:
                    self.log("⚠️ Some services are unhealthy", "WARNING")
                
                time.sleep(self.check_interval)
                
            except KeyboardInterrupt:
                self.log("🛑 Simulation interrupted by user", "WARNING")
                break
            except Exception as e:
                self.log(f"⚠️ Monitoring error: {e}", "WARNING")
                time.sleep(self.check_interval)
    
    def print_simulation_info(self):
        """Print simulation information"""
        self.log("📋 SIMULATION INFORMATION")
        self.log("="*40)
        self.log("🚁 Paris Swarm Simulation")
        self.log("🌍 Environment: Paris, France")
        self.log("🤖 AI: SMOLLM:135m")
        self.log("🚁 Quadcopters: 5 autonomous drones")
        self.log("🎯 Mission: Search and Rescue")
        
        self.log("\n🌐 Access Points:")
        self.log("  • AI Service: http://localhost:5001")
        self.log("  • Gazebo Server: http://localhost:11345")
        self.log("  • Gazebo Client: http://localhost:11346")
        
        self.log("\n🎮 Controls:")
        self.log("  • View logs: docker-compose -f docker-compose.working.yml logs -f")
        self.log("  • Stop simulation: Ctrl+C")
        self.log("  • Restart: python3 scripts/run_full_simulation.py")
    
    def run_full_simulation(self):
        """Run the complete simulation"""
        self.log("🚁 PARIS SWARM SIMULATION - FULL DEPLOYMENT")
        self.log("="*50)
        
        try:
            # Start services
            if not self.start_simulation_services():
                return False
            
            # Wait for services to be ready
            self.log("⏳ Waiting for services to initialize...")
            time.sleep(30)
            
            # Check services
            if not self.check_services():
                self.log("❌ Services are not healthy", "ERROR")
                return False
            
            # Test AI decisions
            if not self.test_ai_decisions():
                self.log("❌ AI decision tests failed", "ERROR")
                return False
            
            # Print simulation info
            self.print_simulation_info()
            
            # Start monitoring
            self.running = True
            monitor_thread = threading.Thread(target=self.monitor_simulation)
            monitor_thread.daemon = True
            monitor_thread.start()
            
            # Run mission scenario
            self.run_mission_scenario()
            
            # Wait for user to stop or timeout
            self.log(f"⏰ Simulation will run for {self.timeout_seconds} seconds")
            self.log("🛑 Press Ctrl+C to stop manually")
            
            time.sleep(self.timeout_seconds)
            
            return True
            
        except KeyboardInterrupt:
            self.log("🛑 Simulation interrupted by user", "WARNING")
            return True
        except Exception as e:
            self.log(f"❌ Simulation error: {e}", "ERROR")
            return False
        finally:
            self.running = False
            self.cleanup()
    
    def cleanup(self):
        """Cleanup simulation resources"""
        self.log("🧹 Cleaning up simulation resources...")
        
        try:
            # Stop services gracefully
            subprocess.run(
                ["docker-compose", "-f", "docker-compose.working.yml", "down"],
                timeout=60
            )
            
            # Remove test containers
            subprocess.run(["docker", "rm", "-f", "paris_swarm_ai_test"], check=False)
            
        except Exception as e:
            self.log(f"⚠️ Cleanup error: {e}", "WARNING")

def main():
    """Main simulation function"""
    runner = FullSimulationRunner()
    
    try:
        success = runner.run_full_simulation()
        
        if success:
            print("\n🎉 Full simulation completed successfully!")
            print("🚀 Your Paris swarm simulation is working perfectly!")
        else:
            print("\n❌ Simulation failed. Check the errors above.")
            return False
            
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n🛑 Simulation interrupted by user")
        sys.exit(0) 