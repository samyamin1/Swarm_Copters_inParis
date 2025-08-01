#!/usr/bin/env python3
"""
Show Communications Between Quadcopters
Displays real-time communication between swarm members
"""

import subprocess
import json
import time
import threading
from datetime import datetime

def main():
    print("🚁 PARIS SWARM - COMMUNICATION MONITOR")
    print("="*50)
    print("📡 Monitoring communications between quadcopters...")
    print("="*50)
    
    def simulate_communications():
        while True:
            try:
                # Different communication scenarios
                scenarios = [
                    {
                        "scenario": "Quadcopter 001 reporting position to swarm - currently at coordinates [10, 5, 15]",
                        "quad_id": "quad_001"
                    },
                    {
                        "scenario": "Quadcopter 002 detecting obstacle ahead, warning others to change formation",
                        "quad_id": "quad_002"
                    },
                    {
                        "scenario": "Quadcopter 003 coordinating formation flight - maintaining 5m spacing",
                        "quad_id": "quad_003"
                    },
                    {
                        "scenario": "All quadcopters synchronizing for search pattern - expanding coverage area",
                        "quad_id": "quad_001"
                    },
                    {
                        "scenario": "Emergency landing protocol activated - all units return to base",
                        "quad_id": "quad_002"
                    },
                    {
                        "scenario": "Quadcopter 001 detecting thermal updraft, adjusting altitude for efficiency",
                        "quad_id": "quad_001"
                    },
                    {
                        "scenario": "Quadcopter 003 reporting battery at 75% - requesting rotation",
                        "quad_id": "quad_003"
                    },
                    {
                        "scenario": "Formation flight successful - maintaining visual contact with all units",
                        "quad_id": "quad_002"
                    }
                ]
                
                for scenario in scenarios:
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    
                    # Make API call to AI service
                    response = subprocess.run([
                        "curl", "-s", "-X", "POST", 
                        "http://localhost:5002/flight_decision",
                        "-H", "Content-Type: application/json",
                        "-d", json.dumps(scenario)
                    ], capture_output=True, text=True)
                    
                    if response.returncode == 0:
                        try:
                            result = json.loads(response.stdout)
                            
                            # Color coding based on quadcopter
                            colors = {
                                "quad_001": "🟢",  # Green
                                "quad_002": "🔴",  # Red  
                                "quad_003": "🔵"   # Blue
                            }
                            
                            color = colors.get(result["quad_id"], "⚪")
                            
                            print(f"[{timestamp}] {color} {result['quad_id']}: {result['decision']}")
                            print(f"    📡 Message: {scenario['scenario']}")
                            print()
                            
                        except json.JSONDecodeError:
                            print(f"[{timestamp}] ❌ Error parsing response")
                    else:
                        print(f"[{timestamp}] ❌ Communication failed")
                    
                    time.sleep(4)  # Wait 4 seconds between communications
                    
            except Exception as e:
                print(f"❌ Communication error: {e}")
                time.sleep(5)
    
    # Start communication simulation
    comm_thread = threading.Thread(target=simulate_communications, daemon=True)
    comm_thread.start()
    
    print("✅ Communication monitoring started!")
    print("🎮 Visual simulation is running at: http://localhost:11348")
    print("🤖 AI Service is available at: http://localhost:5002")
    print("="*50)
    print("📡 Real-time communications:")
    print("="*50)
    
    # Keep the main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Communication monitoring stopped")

if __name__ == "__main__":
    main() 