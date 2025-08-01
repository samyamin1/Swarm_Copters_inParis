#!/usr/bin/env python3
"""
Comprehensive Simulation Test for Paris Swarm
Tests all components with safety timers and detailed monitoring
"""

import os
import sys
import subprocess
import time
import threading
import json
import urllib.request
import urllib.error
from datetime import datetime

class ComprehensiveSimulationTest:
    def __init__(self):
        self.test_results = {}
        self.running = False
        self.timeout_seconds = 600  # 10 minutes for comprehensive test
        self.check_interval = 30
        
    def log(self, message, level="INFO"):
        """Log message with timestamp and level"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")
        
    def test_ollama_service(self):
        """Test Ollama service and SMOLLM model"""
        self.log("🔍 Testing Ollama Service...")
        
        try:
            # Test Ollama API
            response = urllib.request.urlopen(
                "http://localhost:11434/api/tags",
                timeout=10
            )
            data = json.loads(response.read().decode('utf-8'))
            
            # Check if SMOLLM model is available
            models = [model['name'] for model in data.get('models', [])]
            if 'smollm:135m' in models:
                self.log("✅ Ollama service running with SMOLLM model", "SUCCESS")
                self.test_results['ollama'] = "PASS"
                return True
            else:
                self.log("❌ SMOLLM model not found", "ERROR")
                self.test_results['ollama'] = "FAIL"
                return False
                
        except Exception as e:
            self.log(f"❌ Ollama service test failed: {e}", "ERROR")
            self.test_results['ollama'] = "FAIL"
            return False
    
    def test_ai_service(self):
        """Test AI Decision Service"""
        self.log("🤖 Testing AI Decision Service...")
        
        try:
            # Start AI service
            self.log("🚀 Starting AI service container...")
            subprocess.run([
                "docker", "run", "-d", "--name", "test_ai_service", 
                "-p", "5001:5000", "paris_swarm_ai"
            ], check=True)
            
            # Wait for service to start
            time.sleep(10)
            
            # Test health endpoint
            response = urllib.request.urlopen(
                "http://localhost:5001/health",
                timeout=10
            )
            health_data = json.loads(response.read().decode('utf-8'))
            
            if health_data.get('status') == 'healthy':
                self.log("✅ AI service health check passed", "SUCCESS")
                
                # Test flight decision
                flight_data = {
                    "scenario": "Quadcopter at position [10, 5, 15], obstacle detected ahead",
                    "quad_id": "quad_001"
                }
                
                req = urllib.request.Request(
                    "http://localhost:5001/flight_decision",
                    data=json.dumps(flight_data).encode('utf-8'),
                    headers={'Content-Type': 'application/json'}
                )
                
                response = urllib.request.urlopen(req, timeout=10)
                decision_data = json.loads(response.read().decode('utf-8'))
                
                if 'decision' in decision_data:
                    self.log(f"✅ AI flight decision: {decision_data['decision']}", "SUCCESS")
                    self.test_results['ai_service'] = "PASS"
                    return True
                else:
                    self.log("❌ AI flight decision failed", "ERROR")
                    self.test_results['ai_service'] = "FAIL"
                    return False
            else:
                self.log("❌ AI service health check failed", "ERROR")
                self.test_results['ai_service'] = "FAIL"
                return False
                
        except Exception as e:
            self.log(f"❌ AI service test failed: {e}", "ERROR")
            self.test_results['ai_service'] = "FAIL"
            return False
        finally:
            # Cleanup AI service
            try:
                subprocess.run(["docker", "rm", "-f", "test_ai_service"], check=False)
            except:
                pass
    
    def test_docker_compose(self):
        """Test Docker Compose services"""
        self.log("🐳 Testing Docker Compose services...")
        
        try:
            # Start services
            self.log("🚀 Starting Docker Compose services...")
            subprocess.run(["docker-compose", "up", "-d"], check=True)
            
            # Wait for services to start
            time.sleep(30)
            
            # Check service status
            result = subprocess.run(
                ["docker-compose", "ps"],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                self.log("✅ Docker Compose services started", "SUCCESS")
                print(result.stdout)
                self.test_results['docker_compose'] = "PASS"
                return True
            else:
                self.log("❌ Docker Compose services failed", "ERROR")
                print(result.stderr)
                self.test_results['docker_compose'] = "FAIL"
                return False
                
        except Exception as e:
            self.log(f"❌ Docker Compose test failed: {e}", "ERROR")
            self.test_results['docker_compose'] = "FAIL"
            return False
    
    def test_simulation_components(self):
        """Test individual simulation components"""
        self.log("🧪 Testing simulation components...")
        
        components = [
            ("ai_controllers", "AI Controllers"),
            ("quadcopter_models", "Quadcopter Models"),
            ("paris_environment", "Paris Environment"),
            ("simulation", "Simulation Launch Files"),
            ("control_panel", "Control Panel"),
            ("swarm_agents", "Swarm Agents")
        ]
        
        all_passed = True
        
        for component_dir, component_name in components:
            if os.path.exists(component_dir):
                self.log(f"✅ {component_name} directory exists", "SUCCESS")
                self.test_results[f"{component_dir}"] = "PASS"
            else:
                self.log(f"❌ {component_name} directory missing", "ERROR")
                self.test_results[f"{component_dir}"] = "FAIL"
                all_passed = False
        
        return all_passed
    
    def test_launch_files(self):
        """Test ROS launch files"""
        self.log("🚀 Testing launch files...")
        
        launch_files = [
            "simulation/paris_swarm_simulation.launch",
            "control_panel/launch/mission_control.launch"
        ]
        
        all_passed = True
        
        for launch_file in launch_files:
            if os.path.exists(launch_file):
                self.log(f"✅ Launch file exists: {launch_file}", "SUCCESS")
                self.test_results[f"launch_{launch_file}"] = "PASS"
            else:
                self.log(f"❌ Launch file missing: {launch_file}", "ERROR")
                self.test_results[f"launch_{launch_file}"] = "FAIL"
                all_passed = False
        
        return all_passed
    
    def monitor_simulation(self):
        """Monitor simulation during test"""
        self.log("🔍 Starting simulation monitoring...")
        
        start_time = time.time()
        
        while self.running:
            try:
                # Check timeout
                elapsed = time.time() - start_time
                if elapsed > self.timeout_seconds:
                    self.log(f"⏰ Test timeout reached ({self.timeout_seconds}s)", "WARNING")
                    break
                
                # Check Docker services
                try:
                    result = subprocess.run(
                        ["docker-compose", "ps"],
                        capture_output=True,
                        text=True,
                        timeout=10
                    )
                    if result.returncode == 0:
                        self.log("✅ Docker services running", "INFO")
                    else:
                        self.log("⚠️ Docker services may have issues", "WARNING")
                except Exception as e:
                    self.log(f"⚠️ Docker check failed: {e}", "WARNING")
                
                time.sleep(self.check_interval)
                
            except KeyboardInterrupt:
                self.log("🛑 Test interrupted by user", "WARNING")
                break
            except Exception as e:
                self.log(f"⚠️ Monitoring error: {e}", "WARNING")
                time.sleep(self.check_interval)
    
    def run_comprehensive_test(self):
        """Run comprehensive simulation test"""
        self.log("🚁 Starting Comprehensive Paris Swarm Simulation Test")
        self.log("="*60)
        
        try:
            # Test 1: Ollama Service
            if not self.test_ollama_service():
                self.log("❌ Ollama service test failed", "ERROR")
                return False
            
            # Test 2: AI Service
            if not self.test_ai_service():
                self.log("❌ AI service test failed", "ERROR")
                return False
            
            # Test 3: Simulation Components
            if not self.test_simulation_components():
                self.log("❌ Simulation components test failed", "ERROR")
                return False
            
            # Test 4: Launch Files
            if not self.test_launch_files():
                self.log("❌ Launch files test failed", "ERROR")
                return False
            
            # Test 5: Docker Compose (with monitoring)
            self.running = True
            monitor_thread = threading.Thread(target=self.monitor_simulation)
            monitor_thread.daemon = True
            monitor_thread.start()
            
            if not self.test_docker_compose():
                self.log("❌ Docker Compose test failed", "ERROR")
                return False
            
            # Wait for monitoring
            self.log(f"⏰ Running comprehensive test for {self.timeout_seconds} seconds")
            time.sleep(self.timeout_seconds)
            
            return True
            
        except Exception as e:
            self.log(f"❌ Comprehensive test failed: {e}", "ERROR")
            return False
        finally:
            self.running = False
            self.cleanup()
    
    def cleanup(self):
        """Cleanup test resources"""
        self.log("🧹 Cleaning up test resources...")
        
        try:
            # Stop Docker Compose
            subprocess.run(["docker-compose", "down"], timeout=60)
            
            # Remove test containers
            subprocess.run(["docker", "rm", "-f", "test_ai_service"], check=False)
            
        except Exception as e:
            self.log(f"⚠️ Cleanup error: {e}", "WARNING")
    
    def print_results(self):
        """Print test results"""
        self.log("📊 Test Results Summary")
        self.log("="*40)
        
        passed = 0
        failed = 0
        
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result == "PASS" else "❌ FAIL"
            self.log(f"{test_name}: {status}")
            
            if result == "PASS":
                passed += 1
            else:
                failed += 1
        
        self.log("="*40)
        self.log(f"Total Tests: {len(self.test_results)}")
        self.log(f"Passed: {passed}")
        self.log(f"Failed: {failed}")
        
        if failed == 0:
            self.log("🎉 All tests passed! Simulation is ready to run.", "SUCCESS")
        else:
            self.log(f"⚠️ {failed} test(s) failed. Check logs above.", "WARNING")

def main():
    """Main test function"""
    print("🚁 PARIS SWARM SIMULATION - COMPREHENSIVE TEST")
    print("="*60)
    print("🧪 Testing Components:")
    print("  • Ollama Service & SMOLLM Model")
    print("  • AI Decision Service")
    print("  • Docker Compose Services")
    print("  • Simulation Components")
    print("  • Launch Files")
    print("  • Real-time Monitoring")
    print("="*60)
    
    tester = ComprehensiveSimulationTest()
    
    try:
        success = tester.run_comprehensive_test()
        tester.print_results()
        
        if success:
            print("\n🎉 Comprehensive test completed successfully!")
            return True
        else:
            print("\n❌ Comprehensive test failed. Check results above.")
            return False
            
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
        return False
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False
    finally:
        tester.cleanup()

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
        sys.exit(0) 