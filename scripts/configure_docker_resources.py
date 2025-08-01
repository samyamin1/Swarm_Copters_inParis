#!/usr/bin/env python3
"""
Docker Resources Configuration for Paris Swarm
Ensures SMOLLM has adequate resources to work properly
"""

import os
import sys
import subprocess
import time
import json
import urllib.request
from datetime import datetime

class DockerResourceConfig:
    def __init__(self):
        self.min_memory_gb = 8
        self.min_cpus = 4
        self.recommended_memory_gb = 12
        
    def log(self, message, level="INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")
        
    def check_current_resources(self):
        """Check current Docker resource allocation"""
        self.log("🔍 Checking current Docker resources...")
        
        try:
            # Get Docker system info
            result = subprocess.run(
                ["docker", "system", "info"],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                info = result.stdout
                
                # Extract CPU and memory info
                cpu_line = [line for line in info.split('\n') if 'CPUs:' in line]
                memory_line = [line for line in info.split('\n') if 'Total Memory:' in line]
                
                if cpu_line:
                    cpus = int(cpu_line[0].split(':')[1].strip())
                    self.log(f"📊 Current CPUs: {cpus}")
                else:
                    cpus = 0
                    self.log("⚠️ Could not determine CPU count")
                
                if memory_line:
                    memory_str = memory_line[0].split(':')[1].strip()
                    # Convert to GB
                    if 'GiB' in memory_str:
                        memory_gb = float(memory_str.replace('GiB', ''))
                    elif 'MiB' in memory_str:
                        memory_gb = float(memory_str.replace('MiB', '')) / 1024
                    else:
                        memory_gb = 0
                    
                    self.log(f"📊 Current Memory: {memory_gb:.1f}GB")
                else:
                    memory_gb = 0
                    self.log("⚠️ Could not determine memory allocation")
                
                return cpus, memory_gb
            else:
                self.log("❌ Failed to get Docker system info")
                return 0, 0
                
        except Exception as e:
            self.log(f"❌ Error checking resources: {e}")
            return 0, 0
    
    def check_smollm_requirements(self):
        """Check if current resources meet SMOLLM requirements"""
        self.log("🤖 Checking SMOLLM resource requirements...")
        
        cpus, memory_gb = self.check_current_resources()
        
        # SMOLLM requirements
        cpu_ok = cpus >= self.min_cpus
        memory_ok = memory_gb >= self.min_memory_gb
        
        self.log(f"📋 SMOLLM Requirements:")
        self.log(f"   • CPUs: {cpus}/{self.min_cpus} {'✅' if cpu_ok else '❌'}")
        self.log(f"   • Memory: {memory_gb:.1f}GB/{self.min_memory_gb}GB {'✅' if memory_ok else '❌'}")
        
        if cpu_ok and memory_ok:
            self.log("✅ Resources meet SMOLLM requirements", "SUCCESS")
            return True
        else:
            self.log("❌ Resources insufficient for SMOLLM", "ERROR")
            return False
    
    def test_smollm_with_resources(self):
        """Test SMOLLM with current resource allocation"""
        self.log("🧪 Testing SMOLLM with current resources...")
        
        try:
            # Check if Ollama is running
            response = urllib.request.urlopen(
                "http://localhost:11434/api/tags",
                timeout=10
            )
            data = json.loads(response.read().decode('utf-8'))
            
            models = [model['name'] for model in data.get('models', [])]
            if 'smollm:135m' not in models:
                self.log("❌ SMOLLM model not available", "ERROR")
                return False
            
            # Test SMOLLM with a simple query
            self.log("🚀 Testing SMOLLM inference...")
            
            test_data = {
                "model": "smollm:135m",
                "messages": [{"role": "user", "content": "Say 'Hello from SMOLLM'"}],
                "stream": False,
                "options": {
                    "temperature": 0,
                    "num_predict": 10,
                    "top_k": 1,
                    "top_p": 0.1,
                }
            }
            
            req = urllib.request.Request(
                "http://localhost:11434/api/chat",
                data=json.dumps(test_data).encode('utf-8'),
                headers={'Content-Type': 'application/json'}
            )
            
            start_time = time.time()
            response = urllib.request.urlopen(req, timeout=30)
            end_time = time.time()
            
            result = json.loads(response.read().decode('utf-8'))
            response_time = end_time - start_time
            
            if 'message' in result and 'content' in result['message']:
                self.log(f"✅ SMOLLM test successful! Response: '{result['message']['content']}'", "SUCCESS")
                self.log(f"⏱️ Response time: {response_time:.2f} seconds")
                return True
            else:
                self.log("❌ SMOLLM test failed - no valid response", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"❌ SMOLLM test failed: {e}", "ERROR")
            return False
    
    def provide_resource_instructions(self):
        """Provide instructions for configuring Docker resources"""
        self.log("📋 DOCKER RESOURCE CONFIGURATION INSTRUCTIONS")
        self.log("="*50)
        
        self.log("🔧 To configure Docker resources properly:")
        self.log("1. Open Docker Desktop")
        self.log("2. Go to Settings/Preferences")
        self.log("3. Navigate to Resources")
        self.log("4. Set the following values:")
        self.log(f"   • Memory: {self.recommended_memory_gb}GB (minimum {self.min_memory_gb}GB)")
        self.log(f"   • CPUs: {self.min_cpus} or more")
        self.log("   • Swap: 2GB")
        self.log("   • Disk image size: 64GB")
        
        self.log("\n🍎 For Apple Silicon (M1/M2):")
        self.log("• Use 'Use the new Virtualization framework' if available")
        self.log("• Allocate more memory if you have 16GB+ RAM")
        
        self.log("\n⚡ After changing resources:")
        self.log("1. Click 'Apply & Restart'")
        self.log("2. Wait for Docker to restart")
        self.log("3. Run this script again to verify")
        
        self.log("\n💡 Recommended settings for SMOLLM:")
        self.log(f"• Memory: {self.recommended_memory_gb}GB")
        self.log(f"• CPUs: {self.min_cpus}")
        self.log("• Swap: 2GB")
        self.log("• Disk: 64GB")
    
    def run_resource_check(self):
        """Run comprehensive resource check"""
        self.log("🚁 PARIS SWARM - DOCKER RESOURCE CHECK")
        self.log("="*50)
        
        # Check current resources
        cpus, memory_gb = self.check_current_resources()
        
        # Check if resources meet requirements
        resources_ok = self.check_smollm_requirements()
        
        # Test SMOLLM if resources are adequate
        if resources_ok:
            smollm_ok = self.test_smollm_with_resources()
            if smollm_ok:
                self.log("🎉 SMOLLM is working properly with current resources!", "SUCCESS")
                return True
            else:
                self.log("⚠️ Resources are adequate but SMOLLM test failed", "WARNING")
                return False
        else:
            self.log("❌ Resources are insufficient for SMOLLM", "ERROR")
            self.provide_resource_instructions()
            return False

def main():
    """Main function"""
    config = DockerResourceConfig()
    
    try:
        success = config.run_resource_check()
        
        if success:
            print("\n🎉 Docker resources are properly configured!")
            print("🚀 You can now run the simulation with SMOLLM support.")
        else:
            print("\n⚠️ Please configure Docker resources as shown above.")
            print("🔄 Run this script again after configuring resources.")
            
    except Exception as e:
        print(f"❌ Error during resource check: {e}")

if __name__ == "__main__":
    main() 