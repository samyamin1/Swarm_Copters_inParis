#!/usr/bin/env python3
"""
Safe Simulation Runner for Paris Swarm
Runs the simulation with safety timers and monitoring
"""

import os
import sys
import subprocess
import time
import signal
import threading
import requests
from datetime import datetime

class SafeSimulationRunner:
    def __init__(self):
        self.processes = []
        self.running = False
        self.timeout_seconds = 300  # 5 minutes timeout
        self.check_interval = 30    # Check every 30 seconds
        
    def log(self, message):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] {message}")
        
    def cleanup(self):
        """Clean up all running processes"""
        self.log("🧹 Cleaning up processes...")
        for process in self.processes:
            try:
                if process.poll() is None:  # Process is still running
                    process.terminate()
                    time.sleep(2)
                    if process.poll() is None:
                        process.kill()
            except Exception as e:
                self.log(f"⚠️ Error cleaning up process: {e}")
        self.processes.clear()
        
    def check_service_health(self, service_name, url, timeout=10):
        """Check if a service is healthy"""
        try:
            response = requests.get(url, timeout=timeout)
            return response.status_code == 200
        except:
            return False
            
    def monitor_simulation(self):
        """Monitor simulation health and services"""
        self.log("🔍 Starting simulation monitoring...")
        
        start_time = time.time()
        
        while self.running:
            try:
                # Check if we've exceeded timeout
                elapsed = time.time() - start_time
                if elapsed > self.timeout_seconds:
                    self.log(f"⏰ Timeout reached ({self.timeout_seconds}s). Stopping simulation...")
                    self.stop_simulation()
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
                        self.log("✅ Docker services running")
                    else:
                        self.log("⚠️ Docker services may have issues")
                except Exception as e:
                    self.log(f"⚠️ Docker check failed: {e}")
                
                # Check AI service
                if self.check_service_health("AI Service", "http://localhost:5000/health"):
                    self.log("✅ AI Decision Service healthy")
                else:
                    self.log("⚠️ AI Decision Service not responding")
                    
                # Check control panel
                if self.check_service_health("Control Panel", "http://localhost:8080"):
                    self.log("✅ Control Panel accessible")
                else:
                    self.log("⚠️ Control Panel not accessible")
                    
                time.sleep(self.check_interval)
                
            except KeyboardInterrupt:
                self.log("🛑 Interrupted by user")
                self.stop_simulation()
                break
            except Exception as e:
                self.log(f"⚠️ Monitoring error: {e}")
                time.sleep(self.check_interval)
                
    def start_simulation(self):
        """Start the simulation with safety measures"""
        self.log("🚀 Starting Paris Swarm Simulation with safety timers...")
        
        try:
            # Start Docker services
            self.log("🐳 Starting Docker services...")
            docker_process = subprocess.Popen(
                ["docker-compose", "up", "-d"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(docker_process)
            
            # Wait for Docker services to start
            self.log("⏳ Waiting for Docker services to initialize...")
            time.sleep(30)
            
            # Check if services started successfully
            result = subprocess.run(
                ["docker-compose", "ps"], 
                capture_output=True, 
                text=True, 
                timeout=30
            )
            
            if result.returncode == 0:
                self.log("✅ Docker services started successfully")
                print(result.stdout)
            else:
                self.log("❌ Docker services failed to start")
                print(result.stderr)
                return False
                
            # Start monitoring in background
            self.running = True
            monitor_thread = threading.Thread(target=self.monitor_simulation)
            monitor_thread.daemon = True
            monitor_thread.start()
            
            # Wait for user to stop or timeout
            self.log(f"⏰ Simulation will auto-stop after {self.timeout_seconds} seconds")
            self.log("🎮 Access Control Panel at: http://localhost:8080")
            self.log("🤖 Access AI Service at: http://localhost:5000")
            self.log("📊 Check logs with: docker-compose logs -f")
            self.log("🛑 Press Ctrl+C to stop manually")
            
            # Wait for timeout or user interruption
            time.sleep(self.timeout_seconds)
            
            self.log("⏰ Timeout reached, stopping simulation...")
            self.stop_simulation()
            
            return True
            
        except KeyboardInterrupt:
            self.log("🛑 User interrupted simulation")
            self.stop_simulation()
            return True
        except Exception as e:
            self.log(f"❌ Error starting simulation: {e}")
            self.stop_simulation()
            return False
            
    def stop_simulation(self):
        """Stop the simulation and cleanup"""
        self.log("🛑 Stopping simulation...")
        self.running = False
        
        try:
            # Stop Docker services
            self.log("🐳 Stopping Docker services...")
            subprocess.run(
                ["docker-compose", "down"],
                timeout=60
            )
            
            # Cleanup processes
            self.cleanup()
            
            self.log("✅ Simulation stopped successfully")
            
        except Exception as e:
            self.log(f"⚠️ Error stopping simulation: {e}")
            
    def print_status(self):
        """Print current simulation status"""
        self.log("📊 Current Status:")
        
        try:
            # Check Docker services
            result = subprocess.run(
                ["docker-compose", "ps"], 
                capture_output=True, 
                text=True, 
                timeout=10
            )
            if result.returncode == 0:
                print("🐳 Docker Services:")
                print(result.stdout)
            else:
                print("❌ Docker services not responding")
                
            # Check service health
            print("\n🔍 Service Health:")
            services = [
                ("AI Decision Service", "http://localhost:5000/health"),
                ("Control Panel", "http://localhost:8080"),
                ("Gazebo", "http://localhost:11345")
            ]
            
            for name, url in services:
                if self.check_service_health(name, url):
                    print(f"✅ {name}: Healthy")
                else:
                    print(f"❌ {name}: Not responding")
                    
        except Exception as e:
            self.log(f"⚠️ Error checking status: {e}")

def main():
    """Main function"""
    print("🚁 PARIS SWARM SIMULATION - SAFE RUNNER")
    print("="*50)
    print("🛡️ Safety Features:")
    print("  • 5-minute timeout to prevent hanging")
    print("  • Service health monitoring")
    print("  • Automatic cleanup on exit")
    print("  • Graceful shutdown handling")
    print("="*50)
    
    runner = SafeSimulationRunner()
    
    try:
        # Check if we're in the right directory
        if not os.path.exists("docker-compose.yml"):
            print("❌ docker-compose.yml not found. Please run from project root.")
            return False
            
        # Start simulation
        success = runner.start_simulation()
        
        if success:
            print("\n🎉 Simulation completed successfully!")
        else:
            print("\n❌ Simulation failed. Check logs above.")
            return False
            
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return False
    finally:
        runner.stop_simulation()
        
    return True

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
        sys.exit(0) 