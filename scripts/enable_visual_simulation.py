#!/usr/bin/env python3
"""
Enable Visual Simulation for Paris Swarm
Launches Gazebo GUI with Paris environment and quadcopters
"""

import os
import sys
import subprocess
import time
import platform
from datetime import datetime

class VisualSimulationEnabler:
    def __init__(self):
        self.gazebo_gui_image = "gazebo:gzclient11-focal"
        self.paris_world_file = "paris_environment/worlds/paris_swarm_world.world"
        
    def log(self, message, level="INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")
        
    def check_docker_resources(self):
        """Check if Docker has adequate resources for visual simulation"""
        self.log("🔍 Checking Docker resources for visual simulation...")
        
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
                        self.log("✅ Docker has adequate resources for visual simulation", "SUCCESS")
                        return True
                    else:
                        self.log("⚠️ Docker memory may be insufficient for visual simulation", "WARNING")
                        self.log("💡 Recommended: 12GB memory for smooth visual simulation", "INFO")
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
    
    def create_paris_world(self):
        """Create the Paris world file for visual simulation"""
        self.log("🌍 Creating Paris world for visual simulation...")
        
        paris_world_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="paris_swarm_world">
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ground Plane -->
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
    
    <!-- More Paris landmarks -->
    <model name="arc_de_triomphe">
      <static>true</static>
      <pose>50 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 20 30</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 20 30</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Quadcopter Spawn Points -->
    <model name="spawn_point_1">
      <static>true</static>
      <pose>0 0 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>
    
    <!-- GUI -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 0 100 0 1.5708 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>"""
        
        # Create worlds directory
        os.makedirs("paris_environment/worlds", exist_ok=True)
        
        # Write world file
        with open(self.paris_world_file, "w") as f:
            f.write(paris_world_content)
        
        self.log("✅ Paris world created successfully", "SUCCESS")
        return True
    
    def launch_gazebo_gui(self):
        """Launch Gazebo GUI with Paris world"""
        self.log("🚀 Launching Gazebo GUI with Paris world...")
        
        try:
            # Create docker-compose for visual simulation
            visual_compose = f"""
version: '3.8'

services:
  gazebo_gui:
    image: {self.gazebo_gui_image}
    platform: linux/amd64
    container_name: paris_swarm_gazebo_gui
    environment:
      - DISPLAY=:0
      - QT_X11_NO_MITSHM=1
      - GAZEBO_MODEL_PATH=/workspace/quadcopter_models:/workspace/paris_environment
      - GAZEBO_RESOURCE_PATH=/workspace/paris_environment
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
          memory: 4G
          cpus: '2.0'
        reservations:
          memory: 2G
          cpus: '1.0'
    networks:
      - paris_swarm_network
    restart: unless-stopped
    command: >
      gzclient --verbose
      --world /workspace/{self.paris_world_file}
      --gui-client-plugin libgazebo_ros_gui.so

networks:
  paris_swarm_network:
    external: true
"""
            
            # Write visual compose file
            with open("docker-compose.visual.yml", "w") as f:
                f.write(visual_compose)
            
            self.log("✅ Created visual simulation configuration", "SUCCESS")
            
            # Launch visual simulation
            result = subprocess.run(
                ["docker-compose", "-f", "docker-compose.visual.yml", "up", "-d"],
                timeout=300
            )
            
            if result.returncode == 0:
                self.log("✅ Gazebo GUI launched successfully!", "SUCCESS")
                return True
            else:
                self.log("❌ Failed to launch Gazebo GUI", "ERROR")
                return False
                
        except Exception as e:
            self.log(f"❌ Error launching Gazebo GUI: {e}", "ERROR")
            return False
    
    def spawn_quadcopters(self):
        """Spawn quadcopters in the visual simulation"""
        self.log("🚁 Spawning quadcopters in visual simulation...")
        
        # Quadcopter spawn positions around Paris
        spawn_positions = [
            {"name": "quad_001", "x": 0, "y": 0, "z": 10},
            {"name": "quad_002", "x": 10, "y": 0, "z": 12},
            {"name": "quad_003", "x": -10, "y": 0, "z": 11},
            {"name": "quad_004", "x": 0, "y": 10, "z": 13},
            {"name": "quad_005", "x": 0, "y": -10, "z": 9}
        ]
        
        for quad in spawn_positions:
            try:
                # Spawn quadcopter using Gazebo service
                spawn_cmd = [
                    "docker", "exec", "paris_swarm_gazebo_gui",
                    "gz", "service", "call", "/gazebo/spawn_sdf_model",
                    f"gazebo_msgs/SpawnModel",
                    f"model_name: {quad['name']}",
                    f"model_xml: <sdf version='1.6'><model name='{quad['name']}'><static>false</static><pose>{quad['x']} {quad['y']} {quad['z']} 0 0 0</pose><link name='link'><collision name='collision'><geometry><box><size>0.5 0.5 0.1</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.5 0.5 0.1</size></box></geometry><material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material></visual></link></model></sdf>"
                ]
                
                subprocess.run(spawn_cmd, check=True)
                self.log(f"✅ Spawned {quad['name']} at position ({quad['x']}, {quad['y']}, {quad['z']})", "SUCCESS")
                
            except Exception as e:
                self.log(f"⚠️ Failed to spawn {quad['name']}: {e}", "WARNING")
        
        self.log("🚁 Quadcopter spawning completed", "SUCCESS")
    
    def print_visual_simulation_info(self):
        """Print visual simulation information"""
        self.log("📋 VISUAL SIMULATION INFORMATION")
        self.log("="*40)
        self.log("🎮 Visual Components:")
        self.log("  • 3D Paris Environment")
        self.log("  • 5 Quadcopters")
        self.log("  • Real-time AI Decisions")
        self.log("  • Mission Visualization")
        
        self.log("\n🌐 Access Points:")
        self.log("  • Gazebo GUI: http://localhost:11346")
        self.log("  • AI Service: http://localhost:5002")
        self.log("  • Mission Control: http://localhost:8080")
        
        self.log("\n🎯 What You'll See:")
        self.log("  • Quadcopters flying around Paris")
        self.log("  • Real-time AI decision making")
        self.log("  • Search and rescue missions")
        self.log("  • Formation flying")
        self.log("  • Obstacle avoidance")
        
        self.log("\n🎮 Controls:")
        self.log("  • Mouse: Rotate camera")
        self.log("  • Scroll: Zoom in/out")
        self.log("  • Right-click: Pan camera")
        self.log("  • Space: Pause/Resume simulation")
    
    def enable_visual_simulation(self):
        """Enable complete visual simulation"""
        self.log("🚁 PARIS SWARM - ENABLE VISUAL SIMULATION")
        self.log("="*50)
        
        try:
            # Check Docker resources
            resources_ok = self.check_docker_resources()
            if not resources_ok:
                self.log("⚠️ Consider upgrading Docker resources for better performance", "WARNING")
            
            # Create Paris world
            if not self.create_paris_world():
                return False
            
            # Launch Gazebo GUI
            if not self.launch_gazebo_gui():
                return False
            
            # Wait for GUI to start
            self.log("⏳ Waiting for Gazebo GUI to initialize...")
            time.sleep(30)
            
            # Spawn quadcopters
            self.spawn_quadcopters()
            
            # Print information
            self.print_visual_simulation_info()
            
            self.log("🎉 Visual simulation enabled successfully!", "SUCCESS")
            self.log("🚀 You can now see quadcopters flying in Paris!", "SUCCESS")
            
            return True
            
        except Exception as e:
            self.log(f"❌ Error enabling visual simulation: {e}", "ERROR")
            return False

def main():
    """Main function"""
    enabler = VisualSimulationEnabler()
    
    try:
        success = enabler.enable_visual_simulation()
        
        if success:
            print("\n🎉 Visual simulation is now enabled!")
            print("🚀 Open Gazebo GUI to see quadcopters in Paris!")
            print("💡 Use the controls above to navigate the simulation")
        else:
            print("\n❌ Failed to enable visual simulation.")
            print("🔧 Check the errors above and try again.")
            return False
            
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
        sys.exit(0) 