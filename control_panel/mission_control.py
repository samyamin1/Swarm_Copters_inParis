#!/usr/bin/env python3
"""
Mission Control Interface for Paris Quadcopter Swarm
Provides GUI for swarm management and mission monitoring
"""

import tkinter as tk
from tkinter import ttk, messagebox
import rospy
import json
import threading
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class MissionControlInterface:
    """
    Mission Control Interface for Paris Quadcopter Swarm
    Provides real-time monitoring and control capabilities
    """
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Paris Swarm Mission Control")
        self.root.geometry("1200x800")
        
        # ROS initialization
        rospy.init_node('mission_control_interface', anonymous=True)
        
        # Mission state
        self.mission_active = False
        self.swarm_size = 5
        self.quadcopter_positions = {}
        self.quadcopter_states = {}
        self.mission_progress = 0.0
        self.discovered_targets = []
        
        # ROS publishers and subscribers
        self.setup_ros_communication()
        
        # Create GUI
        self.create_gui()
        
        # Start update thread
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()
    
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        
        # Publishers
        self.mission_command_pub = rospy.Publisher('/mission/commands', String, queue_size=10)
        self.mission_start_pub = rospy.Publisher('/mission/start', String, queue_size=10)
        self.mission_stop_pub = rospy.Publisher('/mission/stop', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/swarm/mission_status', String, self.mission_status_callback)
        rospy.Subscriber('/swarm/status', String, self.swarm_status_callback)
        
    def create_gui(self):
        """Create the mission control GUI"""
        
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title
        title_label = ttk.Label(main_frame, text="🚁 Paris Swarm Mission Control", 
                               font=('Arial', 16, 'bold'))
        title_label.pack(pady=(0, 20))
        
        # Create notebook for tabs
        notebook = ttk.Notebook(main_frame)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # Mission Control Tab
        self.create_mission_control_tab(notebook)
        
        # Swarm Status Tab
        self.create_swarm_status_tab(notebook)
        
        # Communication Log Tab
        self.create_communication_log_tab(notebook)
        
        # Map View Tab
        self.create_map_view_tab(notebook)
        
        # Settings Tab
        self.create_settings_tab(notebook)
    
    def create_mission_control_tab(self, notebook):
        """Create mission control tab"""
        mission_frame = ttk.Frame(notebook)
        notebook.add(mission_frame, text="Mission Control")
        
        # Mission status frame
        status_frame = ttk.LabelFrame(mission_frame, text="Mission Status", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Mission status labels
        self.mission_status_label = ttk.Label(status_frame, text="Status: IDLE", 
                                             font=('Arial', 12))
        self.mission_status_label.pack(anchor=tk.W)
        
        self.mission_progress_label = ttk.Label(status_frame, text="Progress: 0%", 
                                               font=('Arial', 12))
        self.mission_progress_label.pack(anchor=tk.W)
        
        self.targets_found_label = ttk.Label(status_frame, text="Targets Found: 0", 
                                            font=('Arial', 12))
        self.targets_found_label.pack(anchor=tk.W)
        
        # Mission control buttons
        control_frame = ttk.LabelFrame(mission_frame, text="Mission Controls", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Mission type selection
        mission_type_frame = ttk.Frame(control_frame)
        mission_type_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(mission_type_frame, text="Mission Type:").pack(side=tk.LEFT)
        self.mission_type_var = tk.StringVar(value="search_and_rescue")
        mission_type_combo = ttk.Combobox(mission_type_frame, 
                                         textvariable=self.mission_type_var,
                                         values=["search_and_rescue", "formation_flying", 
                                                "surveillance", "emergency_response"],
                                         state="readonly")
        mission_type_combo.pack(side=tk.LEFT, padx=(10, 0))
        
        # Mission parameters
        params_frame = ttk.Frame(control_frame)
        params_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(params_frame, text="Search Area Radius (m):").pack(side=tk.LEFT)
        self.search_radius_var = tk.StringVar(value="100")
        radius_entry = ttk.Entry(params_frame, textvariable=self.search_radius_var, width=10)
        radius_entry.pack(side=tk.LEFT, padx=(10, 0))
        
        ttk.Label(params_frame, text="Swarm Size:").pack(side=tk.LEFT, padx=(20, 0))
        self.swarm_size_var = tk.StringVar(value="5")
        size_entry = ttk.Entry(params_frame, textvariable=self.swarm_size_var, width=5)
        size_entry.pack(side=tk.LEFT, padx=(10, 0))
        
        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X)
        
        self.start_button = ttk.Button(button_frame, text="🚀 Start Mission", 
                                      command=self.start_mission)
        self.start_button.pack(side=tk.LEFT, padx=(0, 10))
        
        self.stop_button = ttk.Button(button_frame, text="⏹️ Stop Mission", 
                                     command=self.stop_mission)
        self.stop_button.pack(side=tk.LEFT, padx=(0, 10))
        
        self.takeoff_button = ttk.Button(button_frame, text="🛫 Takeoff All", 
                                        command=self.takeoff_swarm)
        self.takeoff_button.pack(side=tk.LEFT, padx=(0, 10))
        
        self.land_button = ttk.Button(button_frame, text="🛬 Land All", 
                                     command=self.land_swarm)
        self.land_button.pack(side=tk.LEFT, padx=(0, 10))
        
        self.emergency_button = ttk.Button(button_frame, text="🚨 Emergency", 
                                          command=self.emergency_protocol,
                                          style="Emergency.TButton")
        self.emergency_button.pack(side=tk.LEFT, padx=(0, 10))
        
        # Mission log
        log_frame = ttk.LabelFrame(mission_frame, text="Mission Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.mission_log = tk.Text(log_frame, height=10, wrap=tk.WORD)
        mission_log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, 
                                             command=self.mission_log.yview)
        self.mission_log.configure(yscrollcommand=mission_log_scrollbar.set)
        
        self.mission_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        mission_log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    
    def create_swarm_status_tab(self, notebook):
        """Create swarm status tab"""
        status_frame = ttk.Frame(notebook)
        notebook.add(status_frame, text="Swarm Status")
        
        # Quadcopter status frame
        quad_frame = ttk.LabelFrame(status_frame, text="Quadcopter Status", padding=10)
        quad_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create treeview for quadcopter status
        columns = ('ID', 'State', 'Position', 'Battery', 'Target')
        self.quad_tree = ttk.Treeview(quad_frame, columns=columns, show='headings')
        
        # Define headings
        for col in columns:
            self.quad_tree.heading(col, text=col)
            self.quad_tree.column(col, width=100)
        
        # Add scrollbar
        quad_scrollbar = ttk.Scrollbar(quad_frame, orient=tk.VERTICAL, 
                                       command=self.quad_tree.yview)
        self.quad_tree.configure(yscrollcommand=quad_scrollbar.set)
        
        self.quad_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        quad_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Initialize quadcopter entries
        for i in range(self.swarm_size):
            quad_id = f"quad_{i+1:03d}"
            self.quad_tree.insert('', 'end', values=(quad_id, 'Unknown', 'N/A', 'N/A', 'N/A'))
    
    def create_communication_log_tab(self, notebook):
        """Create communication log tab"""
        comm_frame = ttk.Frame(notebook)
        notebook.add(comm_frame, text="Communication Log")
        
        # Communication log frame
        log_frame = ttk.LabelFrame(comm_frame, text="Swarm Communication", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        # Communication log text widget
        self.comm_log = tk.Text(log_frame, wrap=tk.WORD)
        comm_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, 
                                       command=self.comm_log.yview)
        self.comm_log.configure(yscrollcommand=comm_scrollbar.set)
        
        self.comm_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        comm_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Add sample communication entries
        self.comm_log.insert(tk.END, "🚁 Swarm Communication Log\n")
        self.comm_log.insert(tk.END, "=" * 50 + "\n\n")
        self.comm_log.insert(tk.END, "[INFO] Swarm coordinator initialized\n")
        self.comm_log.insert(tk.END, "[INFO] All quadcopters online\n")
        self.comm_log.insert(tk.END, "[INFO] Mission parameters loaded\n")
    
    def create_map_view_tab(self, notebook):
        """Create map view tab with Paris visualization"""
        map_frame = ttk.Frame(notebook)
        notebook.add(map_frame, text="Map View")
        
        # Create matplotlib figure for Paris map
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, map_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Initialize map
        self.update_map()
    
    def create_settings_tab(self, notebook):
        """Create settings tab"""
        settings_frame = ttk.Frame(notebook)
        notebook.add(settings_frame, text="Settings")
        
        # Settings frame
        settings_label_frame = ttk.LabelFrame(settings_frame, text="Simulation Settings", padding=10)
        settings_label_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Simulation parameters
        param_frame = ttk.Frame(settings_label_frame)
        param_frame.pack(fill=tk.X)
        
        # Real-time factor
        ttk.Label(param_frame, text="Real-time Factor:").grid(row=0, column=0, sticky=tk.W)
        self.rtf_var = tk.StringVar(value="1.0")
        rtf_entry = ttk.Entry(param_frame, textvariable=self.rtf_var, width=10)
        rtf_entry.grid(row=0, column=1, padx=(10, 0))
        
        # Update rate
        ttk.Label(param_frame, text="Update Rate (Hz):").grid(row=1, column=0, sticky=tk.W)
        self.update_rate_var = tk.StringVar(value="50")
        rate_entry = ttk.Entry(param_frame, textvariable=self.update_rate_var, width=10)
        rate_entry.grid(row=1, column=1, padx=(10, 0))
        
        # Safety distance
        ttk.Label(param_frame, text="Safety Distance (m):").grid(row=2, column=0, sticky=tk.W)
        self.safety_distance_var = tk.StringVar(value="5.0")
        safety_entry = ttk.Entry(param_frame, textvariable=self.safety_distance_var, width=10)
        safety_entry.grid(row=2, column=1, padx=(10, 0))
        
        # Apply settings button
        apply_button = ttk.Button(settings_label_frame, text="Apply Settings", 
                                 command=self.apply_settings)
        apply_button.pack(pady=(10, 0))
    
    def start_mission(self):
        """Start the current mission"""
        try:
            mission_data = {
                'type': self.mission_type_var.get(),
                'search_area': {
                    'center': [0, 0, 15],
                    'radius': float(self.search_radius_var.get()),
                    'height_range': [10, 50]
                },
                'swarm_size': int(self.swarm_size_var.get())
            }
            
            mission_msg = String()
            mission_msg.data = json.dumps(mission_data)
            self.mission_start_pub.publish(mission_msg)
            
            self.mission_active = True
            self.update_mission_status("Mission started")
            self.log_message(f"🚀 Mission started: {mission_data['type']}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start mission: {e}")
    
    def stop_mission(self):
        """Stop the current mission"""
        try:
            stop_msg = String()
            stop_msg.data = "STOP"
            self.mission_stop_pub.publish(stop_msg)
            
            self.mission_active = False
            self.update_mission_status("Mission stopped")
            self.log_message("⏹️ Mission stopped")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to stop mission: {e}")
    
    def takeoff_swarm(self):
        """Initiate takeoff for all quadcopters"""
        try:
            takeoff_msg = String()
            takeoff_msg.data = "TAKEOFF_ALL"
            self.mission_command_pub.publish(takeoff_msg)
            
            self.log_message("🛫 Swarm takeoff initiated")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to initiate takeoff: {e}")
    
    def land_swarm(self):
        """Initiate landing for all quadcopters"""
        try:
            land_msg = String()
            land_msg.data = "LAND_ALL"
            self.mission_command_pub.publish(land_msg)
            
            self.log_message("🛬 Swarm landing initiated")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to initiate landing: {e}")
    
    def emergency_protocol(self):
        """Activate emergency protocol"""
        try:
            emergency_msg = String()
            emergency_msg.data = "EMERGENCY"
            self.mission_command_pub.publish(emergency_msg)
            
            self.log_message("🚨 Emergency protocol activated!")
            messagebox.showwarning("Emergency", "Emergency protocol activated!")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to activate emergency protocol: {e}")
    
    def apply_settings(self):
        """Apply simulation settings"""
        try:
            # Update simulation parameters
            rospy.set_param('/simulation/real_time_factor', float(self.rtf_var.get()))
            rospy.set_param('/simulation/update_rate', float(self.update_rate_var.get()))
            rospy.set_param('/simulation/safety_distance', float(self.safety_distance_var.get()))
            
            self.log_message("⚙️ Settings applied")
            messagebox.showinfo("Settings", "Settings applied successfully")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to apply settings: {e}")
    
    def mission_status_callback(self, msg):
        """Handle mission status updates"""
        try:
            status_data = json.loads(msg.data)
            
            # Update mission status
            self.mission_active = status_data.get('mission_active', False)
            self.mission_progress = status_data.get('mission_progress', 0.0)
            self.discovered_targets = status_data.get('discovered_targets', [])
            
            # Update GUI
            self.root.after(0, self.update_mission_display, status_data)
            
        except Exception as e:
            rospy.logwarn(f"Error parsing mission status: {e}")
    
    def swarm_status_callback(self, msg):
        """Handle swarm status updates"""
        try:
            data = msg.data.split(':')
            if len(data) >= 3:
                quad_id = data[0]
                state = data[1]
                position_str = data[2]
                
                # Parse position
                position = eval(position_str)
                
                # Update quadcopter data
                self.quadcopter_positions[quad_id] = position
                self.quadcopter_states[quad_id] = state
                
                # Update GUI
                self.root.after(0, self.update_quadcopter_display, quad_id, state, position)
                
        except Exception as e:
            rospy.logwarn(f"Error parsing swarm status: {e}")
    
    def update_mission_display(self, status_data):
        """Update mission status display"""
        status = status_data.get('swarm_state', 'unknown')
        progress = status_data.get('mission_progress', 0.0)
        targets = len(status_data.get('discovered_targets', []))
        
        self.mission_status_label.config(text=f"Status: {status.upper()}")
        self.mission_progress_label.config(text=f"Progress: {progress:.1%}")
        self.targets_found_label.config(text=f"Targets Found: {targets}")
    
    def update_quadcopter_display(self, quad_id, state, position):
        """Update quadcopter status display"""
        # Find the item in the treeview
        for item in self.quad_tree.get_children():
            if self.quad_tree.item(item)['values'][0] == quad_id:
                # Update the values
                self.quad_tree.item(item, values=(
                    quad_id, 
                    state, 
                    f"[{position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}]",
                    "85%",  # Simulated battery
                    "Searching"  # Simulated target
                ))
                break
    
    def update_map(self):
        """Update the Paris map visualization"""
        self.ax.clear()
        
        # Draw Paris landmarks
        landmarks = {
            'Eiffel Tower': [0, 0],
            'Arc de Triomphe': [50, 0],
            'Louvre': [-50, 30],
            'Notre Dame': [-30, -40],
            'Sacré Cœur': [40, -30]
        }
        
        for name, pos in landmarks.items():
            self.ax.plot(pos[0], pos[1], 'ro', markersize=10)
            self.ax.annotate(name, (pos[0], pos[1]), xytext=(5, 5), 
                            textcoords='offset points', fontsize=8)
        
        # Draw quadcopter positions
        for quad_id, position in self.quadcopter_positions.items():
            self.ax.plot(position[0], position[1], 'bo', markersize=8)
            self.ax.annotate(quad_id, (position[0], position[1]), xytext=(5, 5),
                            textcoords='offset points', fontsize=8)
        
        # Draw search targets
        for i, target in enumerate(self.discovered_targets):
            pos = target.get('position', [0, 0, 0])
            self.ax.plot(pos[0], pos[1], 'go', markersize=6)
            self.ax.annotate(f"Target {i+1}", (pos[0], pos[1]), xytext=(5, 5),
                            textcoords='offset points', fontsize=8)
        
        # Set map limits and labels
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Paris Swarm Map')
        self.ax.grid(True)
        
        self.canvas.draw()
    
    def log_message(self, message):
        """Add message to mission log"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        
        self.mission_log.insert(tk.END, log_entry)
        self.mission_log.see(tk.END)
    
    def update_loop(self):
        """Main update loop"""
        while not rospy.is_shutdown():
            try:
                # Update map every second
                self.root.after(0, self.update_map)
                time.sleep(1.0)
                
            except Exception as e:
                rospy.logerr(f"Update loop error: {e}")
    
    def run(self):
        """Run the mission control interface"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            rospy.loginfo("Mission control interface stopped")

if __name__ == "__main__":
    try:
        interface = MissionControlInterface()
        interface.run()
    except rospy.ROSInterruptException:
        pass 