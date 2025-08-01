#!/usr/bin/env python3
"""
Simple Gazebo Deployment
Optimized for current Docker resources and reusability
"""

import os
import sys
import subprocess
import platform
from datetime import datetime

def log(message, level="INFO"):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] {message}")

def check_docker_resources():
    """Check current Docker resources"""
    log("🔍 Checking Docker resources...")
    
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
                
                log(f"📊 Docker Memory: {memory_gb:.1f}GB")
                
                if memory_gb >= 8:
                    log("✅ Docker has adequate resources", "SUCCESS")
                    return True, memory_gb
                else:
                    log("⚠️ Docker memory may be insufficient", "WARNING")
                    return False, memory_gb
            else:
                log("⚠️ Could not determine Docker memory", "WARNING")
                return False, 0
        else:
            log("❌ Could not get Docker system info", "ERROR")
            return False, 0
            
    except Exception as e:
        log(f"❌ Error checking Docker resources: {e}", "ERROR")
        return False, 0

def get_best_gazebo_config():
    """Get the best Gazebo configuration for current setup"""
    log("🎯 Selecting best Gazebo configuration...")
    
    # Detect platform
    system = platform.system()
    machine = platform.machine()
    
    log(f"System: {system}")
    log(f"Architecture: {machine}")
    
    # Check Docker resources
    resources_ok, memory_gb = check_docker_resources()
    
    # Select configuration based on resources and platform
    if memory_gb >= 8:
        if system == "Darwin" and machine == "arm64":
            log("🍎 Using Apple Silicon configuration", "SUCCESS")
            return "apple_silicon"
        else:
            log("🖥️ Using AMD64 configuration", "SUCCESS")
            return "amd64"
    else:
        log("🖥️ Using minimal configuration for limited resources", "WARNING")
        return "minimal"

def deploy_gazebo(config_type):
    """Deploy Gazebo with the selected configuration"""
    log(f"🚀 Deploying Gazebo with {config_type} configuration...")
    
    try:
        # Create minimal docker-compose override
        override_content = f"""
version: '3.8'

services:
  gazebo:
    image: gazebo:gzserver11-focal
    platform: linux/amd64
    environment:
      - DISPLAY=:0
      - QT_X11_NO_MITSHM=1
    volumes:
      - ./paris_environment:/workspace/paris_environment:ro
      - ./quadcopter_models:/workspace/quadcopter_models:ro
      - ./simulation:/workspace/simulation:ro
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    ports:
      - "11345:11345"
      - "11346:11346"
    deploy:
      resources:
        limits:
          memory: 2G
          cpus: '1.0'
        reservations:
          memory: 1G
          cpus: '0.5'
    networks:
      - paris_swarm_network
    restart: unless-stopped

  ai_decision_service:
    build:
      context: .
      dockerfile: Dockerfile.ai
    environment:
      - OLLAMA_HOST=ollama:11434
      - MODEL_NAME=smollm:135m
    volumes:
      - ./ai_controllers:/workspace/ai_controllers:ro
      - ./swarm_agents:/workspace/swarm_agents:ro
    ports:
      - "5000:5000"
    deploy:
      resources:
        limits:
          memory: 1G
          cpus: '1.0'
        reservations:
          memory: 512M
          cpus: '0.5'
    depends_on:
      - ollama
    networks:
      - paris_swarm_network
    restart: unless-stopped

networks:
  paris_swarm_network:
    external: true
"""
        
        # Write override file
        with open("docker-compose.override.yml", "w") as f:
            f.write(override_content)
        
        log("✅ Created docker-compose override", "SUCCESS")
        
        # Start services
        log("🐳 Starting Gazebo services...")
        
        # First, ensure network exists
        subprocess.run(["docker", "network", "create", "paris_swarm_network"], check=False)
        
        # Start services
        result = subprocess.run(
            ["docker-compose", "up", "-d", "gazebo", "ai_decision_service"],
            timeout=300
        )
        
        if result.returncode == 0:
            log("✅ Gazebo deployment successful!", "SUCCESS")
            return True
        else:
            log("❌ Gazebo deployment failed", "ERROR")
            return False
            
    except Exception as e:
        log(f"❌ Deployment error: {e}", "ERROR")
        return False

def verify_deployment():
    """Verify that Gazebo is running properly"""
    log("🔍 Verifying Gazebo deployment...")
    
    try:
        # Check if containers are running
        result = subprocess.run(
            ["docker-compose", "ps"],
            capture_output=True,
            text=True,
            timeout=30
        )
        
        if result.returncode == 0:
            log("✅ Docker services are running", "SUCCESS")
            print(result.stdout)
            return True
        else:
            log("❌ Docker services are not running", "ERROR")
            return False
            
    except Exception as e:
        log(f"❌ Verification error: {e}", "ERROR")
        return False

def print_deployment_info(config_type):
    """Print deployment information"""
    log("📋 DEPLOYMENT INFORMATION")
    log("="*40)
    log(f"Configuration: {config_type}")
    log(f"Gazebo Server: http://localhost:11345")
    log(f"Gazebo Client: http://localhost:11346")
    log(f"AI Service: http://localhost:5000")
    
    log("\n🎮 Usage:")
    log("• View logs: docker-compose logs -f")
    log("• Stop services: docker-compose down")
    log("• Restart: docker-compose restart")
    log("• Health check: docker-compose ps")

def main():
    """Main deployment function"""
    print("🚁 PARIS SWARM - SIMPLE GAZEBO DEPLOYMENT")
    print("="*50)
    print("🎯 Optimizing for reusability and maintainability...")
    
    try:
        # Get best configuration
        config_type = get_best_gazebo_config()
        
        # Deploy Gazebo
        success = deploy_gazebo(config_type)
        
        if success:
            # Verify deployment
            verify_ok = verify_deployment()
            
            if verify_ok:
                print_deployment_info(config_type)
                print("\n🎉 Gazebo deployment completed successfully!")
                print("🚀 Your simulation is ready to run!")
                print("\n💡 Next steps:")
                print("1. Configure Docker resources to 12GB for better performance")
                print("2. Run: python3 scripts/test_smollm_after_config.py")
                print("3. Access AI service at: http://localhost:5000")
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