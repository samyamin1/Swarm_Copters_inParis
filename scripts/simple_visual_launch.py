#!/usr/bin/env python3
"""
Simple Visual Simulation Launcher
Launches visual simulation using available gazebo image
"""

import subprocess
import time
import os
import json

def main():
    print("🚁 PARIS SWARM - SIMPLE VISUAL LAUNCH")
    print("="*40)
    
    # Check if services are running
    print("🔍 Checking existing services...")
    
    try:
        # Check if Gazebo is running
        result = subprocess.run(
            ["docker", "ps", "--filter", "name=paris_swarm_gazebo"],
            capture_output=True,
            text=True
        )
        
        if "paris_swarm_gazebo" in result.stdout:
            print("✅ Gazebo server is running")
        else:
            print("❌ Gazebo server not found")
            return False
        
        # Check if AI service is running
        result = subprocess.run(
            ["docker", "ps", "--filter", "name=paris_swarm_ai"],
            capture_output=True,
            text=True
        )
        
        if "paris_swarm_ai" in result.stdout:
            print("✅ AI service is running")
        else:
            print("❌ AI service not found")
            return False
        
        print("\n🎮 LAUNCHING VISUAL SIMULATION...")
        print("="*40)
        
        # Create simple visual world with communication visualization
        world_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="paris_visual_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Paris Buildings -->
    <model name="eiffel_tower">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 10 50</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 10 50</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Communication Tower -->
    <model name="comm_tower">
      <static>true</static>
      <pose>20 20 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>2</radius>
              <length>30</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>2</radius>
              <length>30</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Quadcopters with Communication Links -->
    <model name="quad_001">
      <static>false</static>
      <pose>0 0 10 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="quad_002">
      <static>false</static>
      <pose>10 0 12 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="quad_003">
      <static>false</static>
      <pose>-10 0 11 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Communication Visualization -->
    <model name="comm_link_1_2">
      <static>true</static>
      <pose>5 0 11 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 0 0.5</ambient>
            <diffuse>1 1 0 0.5</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <model name="comm_link_2_3">
      <static>true</static>
      <pose>0 0 11.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 1 0.5</ambient>
            <diffuse>0 1 1 0.5</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>
    
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 0 100 0 1.5708 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>"""
        
        # Create world file
        os.makedirs("paris_environment/worlds", exist_ok=True)
        with open("paris_environment/worlds/paris_visual_world.world", "w") as f:
            f.write(world_content)
        
        print("✅ Created visual world file with communication links")
        
        # Launch Gazebo GUI using available image
        print("🚀 Launching Gazebo GUI...")
        
        # Create visual compose with available image
        visual_compose = """version: '3.8'

services:
  gazebo_gui:
    image: gazebo:latest
    platform: linux/amd64
    container_name: paris_swarm_gazebo_gui
    environment:
      - DISPLAY=:0
      - QT_X11_NO_MITSHM=1
      - GAZEBO_MODEL_PATH=/workspace/quadcopter_models:/workspace/paris_environment
    volumes:
      - ./paris_environment:/workspace/paris_environment:ro
      - ./quadcopter_models:/workspace/quadcopter_models:ro
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    ports:
      - "11345:11345"
      - "11346:11346"
    deploy:
      resources:
        limits:
          memory: 4G
          cpus: '2.0'
    networks:
      - paris_swarm_network
    restart: unless-stopped
    command: gazebo --verbose /workspace/paris_environment/worlds/paris_visual_world.world

networks:
  paris_swarm_network:
    external: true"""
        
        with open("docker-compose.visual.yml", "w") as f:
            f.write(visual_compose)
        
        # Start visual simulation
        result = subprocess.run(
            ["docker-compose", "-f", "docker-compose.visual.yml", "up", "-d"],
            timeout=300
        )
        
        if result.returncode == 0:
            print("✅ Gazebo GUI launched successfully!")
            
            # Start communication simulation
            print("📡 Starting communication simulation...")
            start_communication_simulation()
            
            print("\n🎉 VISUAL SIMULATION IS READY!")
            print("="*40)
            print("🚁 What you'll see:")
            print("  • 3D Paris environment with Eiffel Tower")
            print("  • 3 quadcopters (green, red, blue)")
            print("  • Communication links (yellow/cyan lines)")
            print("  • Real-time AI decision making")
            print("  • Search and rescue missions")
            
            print("\n🌐 Access points:")
            print("  • Gazebo GUI: http://localhost:11346")
            print("  • AI Service: http://localhost:5002")
            
            print("\n🎮 Controls:")
            print("  • Mouse: Rotate camera")
            print("  • Scroll: Zoom in/out")
            print("  • Right-click: Pan camera")
            print("  • Space: Pause/Resume")
            
            print("\n🤖 Test AI decisions:")
            print("  curl -X POST http://localhost:5002/flight_decision \\")
            print("    -H 'Content-Type: application/json' \\")
            print("    -d '{\"scenario\": \"Quadcopter flying in Paris\", \"quad_id\": \"quad_001\"}'")
            
            return True
        else:
            print("❌ Failed to launch Gazebo GUI")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def start_communication_simulation():
    """Simulate communication between quadcopters"""
    import threading
    import time
    
    def simulate_communications():
        while True:
            try:
                # Simulate communication between quadcopters
                scenarios = [
                    {"scenario": "Quadcopter 001 reporting position to swarm", "quad_id": "quad_001"},
                    {"scenario": "Quadcopter 002 detecting obstacle, warning others", "quad_id": "quad_002"},
                    {"scenario": "Quadcopter 003 coordinating formation flight", "quad_id": "quad_003"},
                    {"scenario": "All quadcopters synchronizing for search pattern", "quad_id": "quad_001"},
                    {"scenario": "Emergency landing protocol activated", "quad_id": "quad_002"}
                ]
                
                for scenario in scenarios:
                    response = subprocess.run([
                        "curl", "-s", "-X", "POST", 
                        "http://localhost:5002/flight_decision",
                        "-H", "Content-Type: application/json",
                        "-d", json.dumps(scenario)
                    ], capture_output=True, text=True)
                    
                    if response.returncode == 0:
                        try:
                            result = json.loads(response.stdout)
                            print(f"📡 {result['quad_id']}: {result['decision']}")
                        except:
                            pass
                    
                    time.sleep(5)  # Wait 5 seconds between communications
                    
            except Exception as e:
                print(f"Communication simulation error: {e}")
                time.sleep(10)
    
    # Start communication simulation in background
    comm_thread = threading.Thread(target=simulate_communications, daemon=True)
    comm_thread.start()
    print("✅ Communication simulation started")

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            print("\n💡 Try running the full visual simulation script:")
            print("   python3 scripts/enable_visual_simulation.py")
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user") 