#!/usr/bin/env python3
"""
Paris Swarm Simulation Status Report
Shows what's working and provides solutions for issues
"""

import os
import subprocess
import json
import urllib.request
from datetime import datetime

class SimulationStatusReport:
    def __init__(self):
        self.status = {}
        
    def log(self, message, level="INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")
        
    def check_ollama(self):
        """Check Ollama service status"""
        try:
            response = urllib.request.urlopen("http://localhost:11434/api/tags", timeout=5)
            data = json.loads(response.read().decode('utf-8'))
            models = [model['name'] for model in data.get('models', [])]
            
            if 'smollm:135m' in models:
                self.status['ollama'] = "✅ WORKING - SMOLLM:135m available"
                return True
            else:
                self.status['ollama'] = "❌ FAILED - SMOLLM model not found"
                return False
        except:
            self.status['ollama'] = "❌ FAILED - Ollama service not responding"
            return False
    
    def check_ai_service(self):
        """Check AI service status"""
        try:
            # Start AI service temporarily
            subprocess.run([
                "docker", "run", "-d", "--name", "status_ai_test", 
                "-p", "5002:5000", "paris_swarm_ai"
            ], check=True)
            
            time.sleep(5)
            
            response = urllib.request.urlopen("http://localhost:5002/health", timeout=5)
            data = json.loads(response.read().decode('utf-8'))
            
            if data.get('status') == 'healthy':
                self.status['ai_service'] = "✅ WORKING - AI decisions functional"
                return True
            else:
                self.status['ai_service'] = "❌ FAILED - AI service unhealthy"
                return False
                
        except Exception as e:
            self.status['ai_service'] = f"❌ FAILED - {str(e)}"
            return False
        finally:
            # Cleanup
            subprocess.run(["docker", "rm", "-f", "status_ai_test"], check=False)
    
    def check_components(self):
        """Check simulation components"""
        components = [
            ("ai_controllers", "AI Controllers"),
            ("quadcopter_models", "Quadcopter Models"),
            ("paris_environment", "Paris Environment"),
            ("simulation", "Simulation Files"),
            ("control_panel", "Control Panel"),
            ("swarm_agents", "Swarm Agents")
        ]
        
        for dir_name, display_name in components:
            if os.path.exists(dir_name):
                self.status[dir_name] = "✅ READY - Directory exists"
            else:
                self.status[dir_name] = "❌ MISSING - Directory not found"
    
    def check_launch_files(self):
        """Check launch files"""
        launch_files = [
            ("simulation/paris_swarm_simulation.launch", "Main Simulation"),
            ("control_panel/launch/mission_control.launch", "Mission Control")
        ]
        
        for file_path, display_name in launch_files:
            if os.path.exists(file_path):
                self.status[f"launch_{display_name}"] = "✅ READY - Launch file exists"
            else:
                self.status[f"launch_{display_name}"] = "❌ MISSING - Launch file not found"
    
    def check_docker_images(self):
        """Check Docker image compatibility"""
        try:
            result = subprocess.run(
                ["docker", "images", "gazebo/gzserver11-focal"],
                capture_output=True,
                text=True
            )
            
            if "gzserver11-focal" in result.stdout:
                self.status['gazebo_image'] = "✅ AVAILABLE - Gazebo image present"
            else:
                self.status['gazebo_image'] = "❌ MISSING - Gazebo image not found"
                
        except Exception as e:
            self.status['gazebo_image'] = f"❌ ERROR - {str(e)}"
    
    def generate_report(self):
        """Generate comprehensive status report"""
        self.log("🚁 PARIS SWARM SIMULATION - STATUS REPORT")
        self.log("="*60)
        
        # Check all components
        self.check_ollama()
        self.check_ai_service()
        self.check_components()
        self.check_launch_files()
        self.check_docker_images()
        
        # Print status
        self.log("📊 COMPONENT STATUS:")
        self.log("="*40)
        
        working = 0
        total = len(self.status)
        
        for component, status in self.status.items():
            self.log(f"{component}: {status}")
            if "✅" in status:
                working += 1
        
        self.log("="*40)
        self.log(f"Working Components: {working}/{total}")
        
        # Provide recommendations
        self.log("\n💡 RECOMMENDATIONS:")
        self.log("="*40)
        
        if working >= total - 1:  # All but one working
            self.log("🎉 EXCELLENT! Most components are working.")
            self.log("🚀 You can run the simulation with these options:")
            self.log("   1. Use native ROS2/Gazebo installation")
            self.log("   2. Use Docker with platform emulation")
            self.log("   3. Use cloud-based simulation")
            
        elif working >= total * 0.8:  # 80% working
            self.log("✅ GOOD! Most components are working.")
            self.log("🔧 Minor fixes needed for full functionality.")
            
        else:
            self.log("⚠️ NEEDS ATTENTION! Several components need fixing.")
            self.log("🔧 Please check the failed components above.")
        
        # ARM64 specific recommendations
        self.log("\n🍎 APPLE SILICON (ARM64) SOLUTIONS:")
        self.log("="*40)
        self.log("Since you're on Apple Silicon, try these options:")
        self.log("1. Install ROS2/Gazebo natively (recommended)")
        self.log("2. Use Docker with platform emulation:")
        self.log("   docker run --platform linux/amd64 gazebo/gzserver11-focal")
        self.log("3. Use cloud-based simulation services")
        self.log("4. Use ROS2 Docker images with ARM64 support")
        
        return working, total

def main():
    """Main function"""
    import time
    
    reporter = SimulationStatusReport()
    
    try:
        working, total = reporter.generate_report()
        
        print(f"\n🎯 SUMMARY: {working}/{total} components working")
        
        if working >= total - 1:
            print("🎉 SIMULATION IS READY TO RUN!")
            print("🚀 You can start the simulation with safety timers.")
        else:
            print("⚠️ Some components need attention before running.")
            
    except Exception as e:
        print(f"❌ Error generating report: {e}")

if __name__ == "__main__":
    main() 