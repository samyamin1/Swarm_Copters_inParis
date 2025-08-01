# SWARM_Copters_In_Paris 🚁

## Mission Overview
Advanced AI-powered quadcopter swarm simulation for search and rescue missions in Paris. This project elevates swarm robotics from ground rovers to sophisticated aerial vehicles with real flight dynamics and autonomous AI coordination.

## 🎯 Key Features

### 🚁 Quadcopter Architecture
- **Real Flight Controller AI**: Advanced AI-based flight control systems
- **High-Fidelity Flight Dynamics**: Realistic physics simulation with aerodynamic modeling
- **Autonomous Navigation**: AI-powered path planning and obstacle avoidance
- **Swarm Coordination**: Distributed decision making and formation flying

### 🌍 Paris Environment
- **Real Earth Topology**: Accurate Paris terrain and building models
- **Search & Rescue Scenarios**: Mission-based simulation environments
- **Dynamic Weather**: Realistic atmospheric conditions affecting flight

### 🤖 AI Swarm Intelligence
- **SMOLLM Integration**: Real-time AI decisions with 3-second timeout
- **Distributed AI**: Each quadcopter has independent AI decision making
- **Swarm Communication**: Real-time coordination and information sharing
- **Mission Planning**: Autonomous task allocation and execution
- **Adaptive Behavior**: Learning and optimization during missions
- **Intelligent Fallbacks**: Rule-based systems for reliable operation

### 🎮 Simulation & Control
- **Gazebo + ROS Integration**: High-fidelity physics simulation
- **Real-time Visualization**: Live quadcopter movement and communication
- **Control Panel**: Mission monitoring and swarm management interface
- **Communication Logs**: Real-time swarm coordination tracking

## 🏗️ Architecture

```
SWARM_Copters_In_Paris/
├── ai_controllers/          # AI flight controllers and swarm logic
├── quadcopter_models/       # Quadcopter URDF models and dynamics
├── paris_environment/       # Paris world models and terrain
├── simulation/             # Gazebo simulation launch files
├── control_panel/          # Mission control interface
├── swarm_agents/           # SMOLLM AI agents and perception bridge
├── scripts/               # Utility scripts and tools
├── docker-compose.yml      # Complete containerized setup
└── docs/                  # Documentation and blueprints
```

## 🚀 Getting Started

### Prerequisites
- **Docker & Docker Compose**: Containerized development environment
- **ROS2 Humble**: Robot Operating System
- **Gazebo**: Physics simulation engine
- **Python 3.8+**: AI and coordination logic
- **Ollama**: Local LLM serving with SMOLLM:135m

### Quick Start with Docker
```bash
# Clone the repository
git clone https://github.com/your-username/SWARM_Copters_In_Paris.git
cd SWARM_Copters_In_Paris

# Start all services (SMOLLM, Gazebo, ROS2, Control Panel)
docker-compose up -d

# Check services
docker-compose ps

# View logs
docker-compose logs -f
```

### Manual Installation
```bash
# Install dependencies
pip install -r requirements.txt

# Pull SMOLLM model
ollama pull smollm:135m

# Build the workspace
catkin_make
source devel/setup.bash
```

### Launch Simulation
```bash
# Launch the Paris environment with quadcopter swarm
roslaunch simulation paris_swarm_simulation.launch

# Launch control panel
roslaunch control_panel mission_control.launch

# Access AI Decision Service
curl http://localhost:5000/health
```

## 🎯 Mission Scenarios

1. **Search & Rescue**: Locate and assist simulated victims in Paris
2. **Formation Flying**: Coordinated swarm movements and patterns
3. **Obstacle Avoidance**: Dynamic navigation through urban environment
4. **Emergency Response**: Rapid deployment and coordination scenarios

## 🤝 Contributing
This project builds upon advanced swarm robotics concepts and AI coordination. Contributions are welcome for:
- Enhanced flight dynamics models
- Improved AI decision making algorithms
- Additional Paris environment scenarios
- Performance optimizations

## 📄 License
MIT License - See LICENSE file for details

---

**Built with ❤️ for advancing swarm robotics and AI coordination** 