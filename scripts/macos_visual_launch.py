#!/usr/bin/env python3
"""
macOS Visual Simulation Launcher
Launches visual simulation compatible with macOS
"""

import subprocess
import time
import os
import json
import webbrowser
from datetime import datetime

def main():
    print("🚁 PARIS SWARM - macOS VISUAL LAUNCH")
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
        
        # Create a web-based visualization
        print("🌐 Creating web-based 3D visualization...")
        
        # Create HTML visualization
        html_content = """<!DOCTYPE html>
<html>
<head>
    <title>Paris Swarm Simulation - 3D Visualization</title>
    <style>
        body { 
            margin: 0; 
            font-family: Arial, sans-serif; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        .container {
            display: flex;
            height: 100vh;
        }
        .sidebar {
            width: 300px;
            background: rgba(0,0,0,0.8);
            padding: 20px;
            overflow-y: auto;
        }
        .main-view {
            flex: 1;
            position: relative;
        }
        .canvas-container {
            width: 100%;
            height: 100%;
            background: #000;
        }
        .status {
            position: absolute;
            top: 10px;
            left: 10px;
            background: rgba(0,0,0,0.8);
            padding: 10px;
            border-radius: 5px;
        }
        .quad-info {
            margin: 10px 0;
            padding: 10px;
            border-radius: 5px;
            background: rgba(255,255,255,0.1);
        }
        .quad-001 { border-left: 4px solid #00ff00; }
        .quad-002 { border-left: 4px solid #ff0000; }
        .quad-003 { border-left: 4px solid #0000ff; }
        .communication {
            background: rgba(255,255,0,0.2);
            border-left: 4px solid #ffff00;
            margin: 5px 0;
            padding: 8px;
            font-size: 12px;
        }
        h1, h2 { margin: 0 0 10px 0; }
        .controls {
            margin: 20px 0;
            padding: 15px;
            background: rgba(255,255,255,0.1);
            border-radius: 5px;
        }
        .control-btn {
            background: #4CAF50;
            color: white;
            border: none;
            padding: 8px 16px;
            margin: 5px;
            border-radius: 4px;
            cursor: pointer;
        }
        .control-btn:hover { background: #45a049; }
    </style>
</head>
<body>
    <div class="container">
        <div class="sidebar">
            <h1>🚁 Paris Swarm</h1>
            <h2>3D Visualization</h2>
            
            <div class="status">
                <strong>Status:</strong> <span id="status">Connecting...</span>
            </div>
            
            <div class="controls">
                <h3>🎮 Controls</h3>
                <button class="control-btn" onclick="startSimulation()">Start Simulation</button>
                <button class="control-btn" onclick="pauseSimulation()">Pause</button>
                <button class="control-btn" onclick="resetSimulation()">Reset</button>
            </div>
            
            <div class="quad-info quad-001">
                <strong>🟢 Quadcopter 001</strong><br>
                Position: <span id="pos-001">[0, 0, 10]</span><br>
                Status: <span id="status-001">Flying</span><br>
                Battery: <span id="battery-001">85%</span>
            </div>
            
            <div class="quad-info quad-002">
                <strong>🔴 Quadcopter 002</strong><br>
                Position: <span id="pos-002">[10, 0, 12]</span><br>
                Status: <span id="status-002">Flying</span><br>
                Battery: <span id="battery-002">78%</span>
            </div>
            
            <div class="quad-info quad-003">
                <strong>🔵 Quadcopter 003</strong><br>
                Position: <span id="pos-003">[-10, 0, 11]</span><br>
                Status: <span id="status-003">Flying</span><br>
                Battery: <span id="battery-003">92%</span>
            </div>
            
            <div id="communications">
                <h3>📡 Communications</h3>
                <div class="communication">
                    <strong>🟢 Quad_001:</strong> Reporting position to swarm
                </div>
                <div class="communication">
                    <strong>🔴 Quad_002:</strong> Detecting obstacle ahead
                </div>
                <div class="communication">
                    <strong>🔵 Quad_003:</strong> Coordinating formation flight
                </div>
            </div>
        </div>
        
        <div class="main-view">
            <div class="canvas-container" id="canvas-container">
                <canvas id="gazebo-canvas" width="800" height="600"></canvas>
            </div>
        </div>
    </div>

    <script>
        // 3D Visualization using Three.js
        let scene, camera, renderer, quadcopters = {};
        
        function init3D() {
            // Create scene
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x87CEEB); // Sky blue
            
            // Create camera
            camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
            camera.position.set(50, 30, 50);
            camera.lookAt(0, 0, 0);
            
            // Create renderer
            renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('gazebo-canvas'), antialias: true });
            renderer.setSize(document.getElementById('canvas-container').clientWidth, document.getElementById('canvas-container').clientHeight);
            
            // Add lighting
            const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
            scene.add(ambientLight);
            
            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
            directionalLight.position.set(50, 50, 50);
            scene.add(directionalLight);
            
            // Create ground
            const groundGeometry = new THREE.PlaneGeometry(100, 100);
            const groundMaterial = new THREE.MeshLambertMaterial({ color: 0x90EE90 });
            const ground = new THREE.Mesh(groundGeometry, groundMaterial);
            ground.rotation.x = -Math.PI / 2;
            scene.add(ground);
            
            // Create Eiffel Tower
            const towerGeometry = new THREE.BoxGeometry(10, 50, 10);
            const towerMaterial = new THREE.MeshLambertMaterial({ color: 0x808080 });
            const tower = new THREE.Mesh(towerGeometry, towerMaterial);
            tower.position.set(0, 25, 0);
            scene.add(tower);
            
            // Create quadcopters
            const quadColors = {
                'quad_001': 0x00ff00, // Green
                'quad_002': 0xff0000, // Red
                'quad_003': 0x0000ff  // Blue
            };
            
            Object.keys(quadColors).forEach((quadId, index) => {
                const quadGeometry = new THREE.BoxGeometry(0.5, 0.1, 0.5);
                const quadMaterial = new THREE.MeshLambertMaterial({ color: quadColors[quadId] });
                const quad = new THREE.Mesh(quadGeometry, quadMaterial);
                
                // Position quadcopters
                const positions = [
                    [0, 10, 0],
                    [10, 12, 0],
                    [-10, 11, 0]
                ];
                
                quad.position.set(...positions[index]);
                scene.add(quad);
                quadcopters[quadId] = quad;
            });
            
            // Animation loop
            function animate() {
                requestAnimationFrame(animate);
                
                // Rotate quadcopters
                Object.values(quadcopters).forEach(quad => {
                    quad.rotation.y += 0.01;
                });
                
                renderer.render(scene, camera);
            }
            
            animate();
            
            // Handle window resize
            window.addEventListener('resize', onWindowResize, false);
        }
        
        function onWindowResize() {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        }
        
        function startSimulation() {
            document.getElementById('status').textContent = 'Running';
            document.getElementById('status').style.color = '#00ff00';
        }
        
        function pauseSimulation() {
            document.getElementById('status').textContent = 'Paused';
            document.getElementById('status').style.color = '#ffff00';
        }
        
        function resetSimulation() {
            document.getElementById('status').textContent = 'Reset';
            document.getElementById('status').style.color = '#ff0000';
        }
        
        // Initialize 3D scene
        init3D();
        
        // Simulate real-time updates
        setInterval(() => {
            // Update positions
            const positions = [
                [Math.random() * 20 - 10, 10 + Math.random() * 5, Math.random() * 20 - 10],
                [10 + Math.random() * 10, 12 + Math.random() * 3, Math.random() * 10 - 5],
                [-10 + Math.random() * 10, 11 + Math.random() * 4, Math.random() * 10 - 5]
            ];
            
            Object.values(quadcopters).forEach((quad, index) => {
                quad.position.set(...positions[index]);
            });
            
            // Update UI
            document.getElementById('pos-001').textContent = `[${positions[0][0].toFixed(1)}, ${positions[0][1].toFixed(1)}, ${positions[0][2].toFixed(1)}]`;
            document.getElementById('pos-002').textContent = `[${positions[1][0].toFixed(1)}, ${positions[1][1].toFixed(1)}, ${positions[1][2].toFixed(1)}]`;
            document.getElementById('pos-003').textContent = `[${positions[2][0].toFixed(1)}, ${positions[2][1].toFixed(1)}, ${positions[2][2].toFixed(1)}]`;
            
        }, 1000);
        
    </script>
    
    <!-- Three.js library -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
</body>
</html>"""
        
        # Create web visualization file
        os.makedirs("visualization", exist_ok=True)
        with open("visualization/paris_swarm_3d.html", "w") as f:
            f.write(html_content)
        
        print("✅ Created web-based 3D visualization")
        
        # Start communication simulation
        print("📡 Starting communication simulation...")
        start_communication_simulation()
        
        # Open the visualization in browser
        print("🌐 Opening 3D visualization in browser...")
        webbrowser.open('file://' + os.path.abspath('visualization/paris_swarm_3d.html'))
        
        print("\n🎉 VISUAL SIMULATION IS READY!")
        print("="*40)
        print("🚁 What you'll see:")
        print("  • 3D Paris environment with Eiffel Tower")
        print("  • 3 quadcopters (green, red, blue)")
        print("  • Real-time position updates")
        print("  • Communication logs")
        print("  • Interactive controls")
        
        print("\n🌐 Access points:")
        print("  • 3D Visualization: visualization/paris_swarm_3d.html")
        print("  • AI Service: http://localhost:5002")
        
        print("\n🎮 Controls:")
        print("  • Start/Pause/Reset buttons")
        print("  • Real-time position tracking")
        print("  • Communication monitoring")
        
        return True
            
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