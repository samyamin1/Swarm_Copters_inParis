#!/usr/bin/env python3
"""
Working Gazebo Deployment
Focuses on working components for reusability and maintainability
"""

import os
import sys
import subprocess
import platform
from datetime import datetime

def log(message, level="INFO"):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] {message}")

def deploy_working_services():
    """Deploy only the working services"""
    log("🚀 Deploying working services...")
    
    try:
        # Create a working docker-compose file
        working_compose = """
version: '3.8'

services:
  # Ollama Service (already running)
  ollama:
    image: ollama/ollama:latest
    container_name: paris_swarm_ollama
    ports:
      - "11434:11434"
    volumes:
      - ollama_data:/root/.ollama
    environment:
      - OLLAMA_HOST=0.0.0.0
    deploy:
      resources:
        limits:
          memory: 4G
          cpus: '2.0'
    networks:
      - paris_swarm_network
    restart: unless-stopped

  # AI Decision Service (working)
  ai_decision_service:
    build:
      context: .
      dockerfile: Dockerfile.ai
    container_name: paris_swarm_ai
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
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:5000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 30s

  # Simple Gazebo (working)
  gazebo:
    image: gazebo:gzserver11-focal
    platform: linux/amd64
    container_name: paris_swarm_gazebo
    environment:
      - DISPLAY=:0
      - QT_X11_NO_MITSHM=1
      - GAZEBO_MODEL_PATH=/workspace/quadcopter_models:/workspace/paris_environment
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
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:11345"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s

volumes:
  ollama_data:

networks:
  paris_swarm_network:
    driver: bridge
"""
        
        # Write working compose file
        with open("docker-compose.working.yml", "w") as f:
            f.write(working_compose)
        
        log("✅ Created working docker-compose file", "SUCCESS")
        
        # Start services
        log("🐳 Starting working services...")
        
        result = subprocess.run(
            ["docker-compose", "-f", "docker-compose.working.yml", "up", "-d"],
            timeout=300
        )
        
        if result.returncode == 0:
            log("✅ Working services deployment successful!", "SUCCESS")
            return True
        else:
            log("❌ Working services deployment failed", "ERROR")
            return False
            
    except Exception as e:
        log(f"❌ Deployment error: {e}", "ERROR")
        return False

def verify_services():
    """Verify that services are running properly"""
    log("🔍 Verifying services...")
    
    try:
        # Check if containers are running
        result = subprocess.run(
            ["docker-compose", "-f", "docker-compose.working.yml", "ps"],
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

def test_ai_service():
    """Test the AI service"""
    log("🤖 Testing AI service...")
    
    try:
        import urllib.request
        import json
        import time
        
        # Wait for service to start
        time.sleep(10)
        
        # Test health endpoint
        response = urllib.request.urlopen("http://localhost:5000/health", timeout=10)
        health_data = json.loads(response.read().decode('utf-8'))
        
        if health_data.get('status') == 'healthy':
            log("✅ AI service health check passed", "SUCCESS")
            
            # Test flight decision
            flight_data = {
                "scenario": "Quadcopter at position [10, 5, 15], obstacle detected ahead",
                "quad_id": "quad_001"
            }
            
            req = urllib.request.Request(
                "http://localhost:5000/flight_decision",
                data=json.dumps(flight_data).encode('utf-8'),
                headers={'Content-Type': 'application/json'}
            )
            
            response = urllib.request.urlopen(req, timeout=15)
            decision_data = json.loads(response.read().decode('utf-8'))
            
            if 'decision' in decision_data:
                log(f"✅ AI flight decision: {decision_data['decision']}", "SUCCESS")
                return True
            else:
                log("❌ AI flight decision failed", "ERROR")
                return False
        else:
            log("❌ AI service health check failed", "ERROR")
            return False
            
    except Exception as e:
        log(f"❌ AI service test failed: {e}", "ERROR")
        return False

def print_deployment_info():
    """Print deployment information"""
    log("📋 WORKING DEPLOYMENT INFORMATION")
    log("="*40)
    log("✅ Services Deployed:")
    log("  • Ollama Service (SMOLLM)")
    log("  • AI Decision Service")
    log("  • Gazebo Simulation")
    
    log("\n🌐 Access Points:")
    log("  • AI Service: http://localhost:5000")
    log("  • Gazebo Server: http://localhost:11345")
    log("  • Gazebo Client: http://localhost:11346")
    
    log("\n🎮 Usage:")
    log("• View logs: docker-compose -f docker-compose.working.yml logs -f")
    log("• Stop services: docker-compose -f docker-compose.working.yml down")
    log("• Restart: docker-compose -f docker-compose.working.yml restart")
    log("• Health check: docker-compose -f docker-compose.working.yml ps")

def main():
    """Main deployment function"""
    print("🚁 PARIS SWARM - WORKING GAZEBO DEPLOYMENT")
    print("="*50)
    print("🎯 Deploying working components for reusability...")
    
    try:
        # Deploy working services
        success = deploy_working_services()
        
        if success:
            # Verify services
            verify_ok = verify_services()
            
            if verify_ok:
                # Test AI service
                ai_ok = test_ai_service()
                
                if ai_ok:
                    print_deployment_info()
                    print("\n🎉 Working deployment completed successfully!")
                    print("🚀 Your AI-powered simulation is ready!")
                    print("\n💡 Next steps:")
                    print("1. Configure Docker resources to 12GB for better performance")
                    print("2. Test AI decisions: curl -X POST http://localhost:5000/flight_decision")
                    print("3. Access Gazebo at: http://localhost:11345")
                else:
                    print("\n⚠️ Services deployed but AI test failed.")
                    print("🔄 Check logs: docker-compose -f docker-compose.working.yml logs ai_decision_service")
            else:
                print("\n⚠️ Deployment completed but verification failed.")
                print("🔄 Please check logs: docker-compose -f docker-compose.working.yml logs")
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