#!/usr/bin/env python3
"""
Quick Start Script for Paris Swarm Simulation
Automatically sets up and launches the complete simulation
"""

import os
import sys
import subprocess
import time
import rospy
from std_msgs.msg import String

def check_dependencies():
    """Check if all required dependencies are installed"""
    print("🔍 Checking dependencies...")
    
    required_packages = [
        'rospy', 'numpy', 'matplotlib', 'tkinter'
    ]
    
    missing_packages = []
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"✅ {package}")
        except ImportError:
            missing_packages.append(package)
            print(f"❌ {package} - MISSING")
    
    if missing_packages:
        print(f"\n❌ Missing packages: {', '.join(missing_packages)}")
        print("Please install missing packages:")
        print("pip install -r requirements.txt")
        return False
    
    print("✅ All dependencies satisfied!")
    return True

def check_ros_environment():
    """Check if ROS environment is properly set up"""
    print("\n🔍 Checking ROS environment...")
    
    # Check if ROS_MASTER_URI is set
    if not os.environ.get('ROS_MASTER_URI'):
        print("❌ ROS_MASTER_URI not set")
        print("Please run: source /opt/ros/noetic/setup.bash")
        return False
    
    # Check if roscore is running
    try:
        import rospy
        rospy.init_node('quick_start_check', anonymous=True)
        print("✅ ROS environment ready")
        return True
    except Exception as e:
        print(f"❌ ROS environment error: {e}")
        return False

def generate_world_file():
    """Generate the Paris world file"""
    print("\n🌍 Generating Paris world...")
    
    try:
        from paris_environment.paris_world import generate_paris_world
        
        world_content = generate_paris_world()
        
        # Create worlds directory if it doesn't exist
        worlds_dir = "paris_environment/worlds"
        os.makedirs(worlds_dir, exist_ok=True)
        
        # Write world file
        world_file = os.path.join(worlds_dir, "paris_swarm_world.world")
        with open(world_file, 'w') as f:
            f.write(world_content)
        
        print(f"✅ Paris world generated: {world_file}")
        return True
        
    except Exception as e:
        print(f"❌ Error generating world: {e}")
        return False

def generate_quadcopter_models():
    """Generate quadcopter URDF models"""
    print("\n🚁 Generating quadcopter models...")
    
    try:
        from quadcopter_models.quadcopter_urdf import generate_quadcopter_urdf
        
        # Create models directory if it doesn't exist
        models_dir = "quadcopter_models/models"
        os.makedirs(models_dir, exist_ok=True)
        
        # Generate models for each quadcopter
        for i in range(1, 6):
            quad_id = f"quad_{i:03d}"
            urdf_content = generate_quadcopter_urdf(quad_id)
            
            urdf_file = os.path.join(models_dir, f"{quad_id}.urdf")
            with open(urdf_file, 'w') as f:
                f.write(urdf_content)
            
            print(f"✅ Generated {quad_id} model")
        
        return True
        
    except Exception as e:
        print(f"❌ Error generating models: {e}")
        return False

def launch_simulation():
    """Launch the Paris swarm simulation"""
    print("\n🚀 Launching Paris swarm simulation...")
    
    try:
        # Launch the simulation
        launch_cmd = [
            'roslaunch', 'simulation', 'paris_swarm_simulation.launch',
            'swarm_size:=5',
            'gui:=true'
        ]
        
        print("Starting Gazebo with Paris world...")
        print("Starting AI swarm coordinator...")
        print("Starting quadcopter flight controllers...")
        print("Starting mission control interface...")
        
        # Launch the simulation
        process = subprocess.Popen(launch_cmd)
        
        print("\n🎉 Simulation launched successfully!")
        print("📊 Mission Control Interface should open automatically")
        print("🎮 Use the GUI to control the swarm")
        print("📝 Check the communication logs for detailed information")
        
        return process
        
    except Exception as e:
        print(f"❌ Error launching simulation: {e}")
        return None

def print_usage_instructions():
    """Print usage instructions"""
    print("\n" + "="*60)
    print("🚁 PARIS SWARM SIMULATION - USAGE INSTRUCTIONS")
    print("="*60)
    
    print("\n📋 Available Commands:")
    print("  🚀 Start Mission: Click 'Start Mission' in the GUI")
    print("  🛫 Takeoff All: Click 'Takeoff All' to launch quadcopters")
    print("  🛬 Land All: Click 'Land All' to land quadcopters")
    print("  🚨 Emergency: Click 'Emergency' for emergency landing")
    print("  ⏹️ Stop Mission: Click 'Stop Mission' to end")
    
    print("\n🎯 Mission Types:")
    print("  🔍 Search & Rescue: Locate targets in Paris")
    print("  ✈️ Formation Flying: Coordinated swarm patterns")
    print("  👁️ Surveillance: Monitor specific areas")
    print("  🚨 Emergency Response: Rapid deployment")
    
    print("\n📊 Monitoring:")
    print("  📍 Map View: Real-time quadcopter positions")
    print("  📝 Communication Log: Swarm coordination messages")
    print("  📈 Performance: System performance metrics")
    print("  ⚙️ Settings: Adjust simulation parameters")
    
    print("\n🔧 Troubleshooting:")
    print("  • If Gazebo doesn't start: Check ROS installation")
    print("  • If GUI doesn't appear: Check tkinter installation")
    print("  • If quadcopters don't move: Check AI controllers")
    print("  • For detailed logs: Check scripts/logs/ directory")
    
    print("\n📚 Documentation:")
    print("  • README.md: Project overview and setup")
    print("  • docs/: Detailed documentation")
    print("  • scripts/logs/: Communication and performance logs")

def main():
    """Main quick start function"""
    print("🚁 PARIS SWARM SIMULATION - QUICK START")
    print("="*50)
    
    # Check dependencies
    if not check_dependencies():
        print("\n❌ Please install missing dependencies and try again.")
        return False
    
    # Check ROS environment
    if not check_ros_environment():
        print("\n❌ Please set up ROS environment and try again.")
        return False
    
    # Generate world file
    if not generate_world_file():
        print("\n❌ Failed to generate Paris world.")
        return False
    
    # Generate quadcopter models
    if not generate_quadcopter_models():
        print("\n❌ Failed to generate quadcopter models.")
        return False
    
    # Launch simulation
    process = launch_simulation()
    if not process:
        print("\n❌ Failed to launch simulation.")
        return False
    
    # Print usage instructions
    print_usage_instructions()
    
    try:
        # Wait for user to stop
        print("\n⏳ Simulation running... Press Ctrl+C to stop")
        process.wait()
    except KeyboardInterrupt:
        print("\n🛑 Stopping simulation...")
        process.terminate()
        process.wait()
        print("✅ Simulation stopped")
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        if success:
            print("\n🎉 Quick start completed successfully!")
        else:
            print("\n❌ Quick start failed. Please check the errors above.")
            sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        sys.exit(1) 