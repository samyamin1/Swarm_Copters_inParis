#!/usr/bin/env python3
"""
Gazebo Deployment Script
Automatically selects the best configuration for reusability and maintainability
"""

import os
import sys
import subprocess
import platform
import json
from datetime import datetime

class GazeboDeployment:
    def __init__(self):
        self.configs = {
            'apple_silicon': 'config/apple_silicon.env',
            'amd64': 'config/amd64.env',
            'headless': 'config/headless.env'
        }
        
    def log(self, message, level="INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")
        
    def detect_platform(self):
        """Detect the best platform configuration"""
        self.log("🔍 Detecting platform and architecture...")
        
        # Get system info
        system = platform.system()
        machine = platform.machine()
        processor = platform.processor()
        
        self.log(f"System: {system}")
        self.log(f"Architecture: {machine}")
        self.log(f"Processor: {processor}")
        
        # Check if running on Apple Silicon
        if system == "Darwin" and machine == "arm64":
            self.log("🍎 Apple Silicon (M1/M2) detected", "SUCCESS")
            return 'apple_silicon'
        elif system == "Darwin" and machine == "x86_64":
            self.log("🍎 Intel Mac detected", "INFO")
            return 'amd64'
        elif system == "Linux":
            self.log("🐧 Linux detected", "INFO")
            return 'amd64'
        else:
            self.log("⚠️ Unknown platform, using AMD64 fallback", "WARNING")
            return 'amd64'
    
    def check_docker_resources(self):
        """Check if Docker has adequate resources"""
        self.log("🔍 Checking Docker resources...")
        
        try:
            result = subprocess.run(
                ["docker", "system", "info"],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                info = result.stdout
                
                # Extract memory info
                memory_line = [line for line in info.split('\n') if 'Total Memory:' in line]
                if memory_line:
                    memory_str = memory_line[0].split(':')[1].strip()
                    if 'GiB' in memory_str:
                        memory_gb = float(memory_str.replace('GiB', ''))
                    else:
                        memory_gb = 0
                    
                    self.log(f"📊 Docker Memory: {memory_gb:.1f}GB")
                    
                    if memory_gb >= 8:
                        self.log("✅ Docker has adequate resources", "SUCCESS")
                        return True
                    else:
                        self.log("⚠️ Docker memory may be insufficient", "WARNING")
                        return False
                else:
                    self.log("⚠️ Could not determine Docker memory", "WARNING")
                    return False
            else:
                self.log("❌ Could not get Docker system info", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"❌ Error checking Docker resources: {e}", "ERROR")
            return False
    
    def test_gazebo_images(self):
        """Test available Gazebo images"""
        self.log("🧪 Testing Gazebo image availability...")
        
        images_to_test = [
            "arm64v8/gazebo:gzserver11-focal",
            "gazebo:gzserver11-focal",
            "amd64/gazebo:gzserver11-focal"
        ]
        
        available_images = []
        
        for image in images_to_test:
            try:
                result = subprocess.run(
                    ["docker", "pull", image],
                    capture_output=True,
                    text=True,
                    timeout=60
                )
                
                if result.returncode == 0:
                    self.log(f"✅ {image} is available", "SUCCESS")
                    available_images.append(image)
                else:
                    self.log(f"❌ {image} is not available", "ERROR")
                    
            except Exception as e:
                self.log(f"❌ Error testing {image}: {e}", "ERROR")
        
        return available_images
    
    def select_best_config(self):
        """Select the best configuration based on platform and resources"""
        self.log("🎯 Selecting optimal Gazebo configuration...")
        
        # Detect platform
        platform_type = self.detect_platform()
        
        # Check Docker resources
        resources_ok = self.check_docker_resources()
        
        # Test available images
        available_images = self.test_gazebo_images()
        
        # Select configuration
        if platform_type == 'apple_silicon':
            if 'arm64v8/gazebo:gzserver11-focal' in available_images:
                self.log("✅ Using native ARM64 Gazebo image", "SUCCESS")
                config_file = self.configs['apple_silicon']
            else:
                self.log("⚠️ ARM64 image not available, using AMD64 with emulation", "WARNING")
                config_file = self.configs['amd64']
        else:
            self.log("✅ Using AMD64 Gazebo image", "SUCCESS")
            config_file = self.configs['amd64']
        
        # Check if headless mode is requested
        if '--headless' in sys.argv:
            self.log("🎮 Headless mode requested", "INFO")
            config_file = self.configs['headless']
        
        return config_file, platform_type
    
    def deploy_gazebo(self, config_file, platform_type):
        """Deploy Gazebo with the selected configuration"""
        self.log(f"🚀 Deploying Gazebo with {platform_type} configuration...")
        
        try:
            # Load environment variables
            env_vars = {}
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith('#') and '=' in line:
                            key, value = line.split('=', 1)
                            env_vars[key] = value
            
            # Set environment variables
            for key, value in env_vars.items():
                os.environ[key] = value
            
            # Deploy with docker-compose
            self.log("🐳 Starting Gazebo services...")
            
            cmd = [
                "docker-compose", 
                "--env-file", config_file,
                "up", "-d"
            ]
            
            result = subprocess.run(cmd, timeout=300)
            
            if result.returncode == 0:
                self.log("✅ Gazebo deployment successful!", "SUCCESS")
                return True
            else:
                self.log("❌ Gazebo deployment failed", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"❌ Deployment error: {e}", "ERROR")
            return False
    
    def verify_deployment(self):
        """Verify that Gazebo is running properly"""
        self.log("🔍 Verifying Gazebo deployment...")
        
        try:
            # Check if containers are running
            result = subprocess.run(
                ["docker-compose", "ps"],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                self.log("✅ Docker services are running", "SUCCESS")
                print(result.stdout)
                
                # Check Gazebo health
                try:
                    import urllib.request
                    response = urllib.request.urlopen("http://localhost:11345", timeout=10)
                    self.log("✅ Gazebo server is responding", "SUCCESS")
                    return True
                except:
                    self.log("⚠️ Gazebo server may not be ready yet", "WARNING")
                    return False
            else:
                self.log("❌ Docker services are not running", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"❌ Verification error: {e}", "ERROR")
            return False
    
    def print_deployment_info(self, config_file, platform_type):
        """Print deployment information"""
        self.log("📋 DEPLOYMENT INFORMATION")
        self.log("="*40)
        self.log(f"Configuration: {config_file}")
        self.log(f"Platform: {platform_type}")
        self.log(f"Gazebo Server: http://localhost:11345")
        self.log(f"Gazebo Client: http://localhost:11346")
        self.log(f"Control Panel: http://localhost:8080")
        self.log(f"AI Service: http://localhost:5000")
        
        self.log("\n🎮 Usage:")
        self.log("• View logs: docker-compose logs -f")
        self.log("• Stop services: docker-compose down")
        self.log("• Restart: docker-compose restart")
        self.log("• Health check: docker-compose ps")

def main():
    """Main deployment function"""
    print("🚁 PARIS SWARM - GAZEBO DEPLOYMENT")
    print("="*50)
    print("🎯 Optimizing for reusability and maintainability...")
    
    deployer = GazeboDeployment()
    
    try:
        # Select best configuration
        config_file, platform_type = deployer.select_best_config()
        
        # Deploy Gazebo
        success = deployer.deploy_gazebo(config_file, platform_type)
        
        if success:
            # Verify deployment
            verify_ok = deployer.verify_deployment()
            
            if verify_ok:
                deployer.print_deployment_info(config_file, platform_type)
                print("\n🎉 Gazebo deployment completed successfully!")
                print("🚀 Your simulation is ready to run!")
            else:
                print("\n⚠️ Deployment completed but verification failed.")
                print("🔄 Please check logs: docker-compose logs")
        else:
            print("\n❌ Deployment failed. Check the errors above.")
            return False
            
    except Exception as e:
        print(f"\n❌ Deployment error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n🛑 Deployment interrupted by user")
        sys.exit(0) 