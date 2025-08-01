#!/usr/bin/env python3
"""
Quadcopter URDF Model Generator
Creates high-fidelity quadcopter models with realistic flight dynamics
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import math

class QuadcopterURDFGenerator:
    """
    Generates URDF models for quadcopters with realistic flight dynamics
    """
    
    def __init__(self, quad_id="quad_001"):
        self.quad_id = quad_id
        self.model_name = f"quadcopter_{quad_id}"
        
        # Physical dimensions
        self.body_length = 0.5  # meters
        self.body_width = 0.3   # meters
        self.body_height = 0.1  # meters
        self.arm_length = 0.25  # meters
        self.propeller_radius = 0.15  # meters
        
        # Mass properties
        self.body_mass = 1.5  # kg
        self.motor_mass = 0.2  # kg each
        self.propeller_mass = 0.05  # kg each
        
        # Flight dynamics parameters
        self.max_thrust = 20.0  # N per motor
        self.max_torque = 2.0   # Nm per motor
        self.drag_coefficient = 0.1
        self.lift_coefficient = 0.8
        
        # Colors
        self.body_color = "0.2 0.2 0.2 1.0"  # Dark gray
        self.motor_color = "0.8 0.8 0.8 1.0"  # Light gray
        self.propeller_color = "0.1 0.1 0.1 1.0"  # Black
        
    def generate_urdf(self):
        """Generate complete URDF for quadcopter"""
        robot = ET.Element("robot")
        robot.set("name", self.model_name)
        
        # Add materials
        self.add_materials(robot)
        
        # Add base link
        self.add_base_link(robot)
        
        # Add motors and propellers
        self.add_motors_and_propellers(robot)
        
        # Add sensors
        self.add_sensors(robot)
        
        # Add plugins
        self.add_plugins(robot)
        
        return self.prettify_xml(robot)
    
    def add_materials(self, robot):
        """Add material definitions"""
        material = ET.SubElement(robot, "material")
        material.set("name", "body_material")
        color = ET.SubElement(material, "color")
        color.set("rgba", self.body_color)
        
        material = ET.SubElement(robot, "material")
        material.set("name", "motor_material")
        color = ET.SubElement(material, "color")
        color.set("rgba", self.motor_color)
        
        material = ET.SubElement(robot, "material")
        material.set("name", "propeller_material")
        color = ET.SubElement(material, "color")
        color.set("rgba", self.propeller_color)
    
    def add_base_link(self, robot):
        """Add the main body link"""
        link = ET.SubElement(robot, "link")
        link.set("name", f"{self.quad_id}_base_link")
        
        # Visual
        visual = ET.SubElement(link, "visual")
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        box.set("size", f"{self.body_length} {self.body_width} {self.body_height}")
        
        material = ET.SubElement(visual, "material")
        material.set("name", "body_material")
        
        # Collision
        collision = ET.SubElement(link, "collision")
        geometry = ET.SubElement(collision, "geometry")
        box = ET.SubElement(geometry, "box")
        box.set("size", f"{self.body_length} {self.body_width} {self.body_height}")
        
        # Inertial
        inertial = ET.SubElement(link, "inertial")
        mass = ET.SubElement(inertial, "mass")
        mass.set("value", str(self.body_mass))
        
        inertia = ET.SubElement(inertial, "inertia")
        # Simplified inertia matrix
        inertia.set("ixx", "0.1")
        inertia.set("ixy", "0.0")
        inertia.set("ixz", "0.0")
        inertia.set("iyy", "0.1")
        inertia.set("iyz", "0.0")
        inertia.set("izz", "0.1")
    
    def add_motors_and_propellers(self, robot):
        """Add motors and propellers at the four corners"""
        motor_positions = [
            (self.arm_length, self.arm_length, 0),   # Front right
            (-self.arm_length, self.arm_length, 0),  # Front left
            (-self.arm_length, -self.arm_length, 0), # Back left
            (self.arm_length, -self.arm_length, 0)   # Back right
        ]
        
        motor_names = ["front_right", "front_left", "back_left", "back_right"]
        
        for i, (pos, name) in enumerate(zip(motor_positions, motor_names)):
            # Motor link
            motor_link = ET.SubElement(robot, "link")
            motor_link.set("name", f"{self.quad_id}_{name}_motor")
            
            # Motor visual
            motor_visual = ET.SubElement(motor_link, "visual")
            motor_geometry = ET.SubElement(motor_visual, "geometry")
            motor_cylinder = ET.SubElement(motor_geometry, "cylinder")
            motor_cylinder.set("radius", "0.03")
            motor_cylinder.set("length", "0.05")
            
            motor_material = ET.SubElement(motor_visual, "material")
            motor_material.set("name", "motor_material")
            
            # Motor collision
            motor_collision = ET.SubElement(motor_link, "collision")
            motor_collision_geometry = ET.SubElement(motor_collision, "geometry")
            motor_collision_cylinder = ET.SubElement(motor_collision_geometry, "cylinder")
            motor_collision_cylinder.set("radius", "0.03")
            motor_collision_cylinder.set("length", "0.05")
            
            # Motor inertial
            motor_inertial = ET.SubElement(motor_link, "inertial")
            motor_mass_elem = ET.SubElement(motor_inertial, "mass")
            motor_mass_elem.set("value", str(self.motor_mass))
            
            motor_inertia = ET.SubElement(motor_inertial, "inertia")
            motor_inertia.set("ixx", "0.001")
            motor_inertia.set("ixy", "0.0")
            motor_inertia.set("ixz", "0.0")
            motor_inertia.set("iyy", "0.001")
            motor_inertia.set("iyz", "0.0")
            motor_inertia.set("izz", "0.001")
            
            # Motor joint
            motor_joint = ET.SubElement(robot, "joint")
            motor_joint.set("name", f"{self.quad_id}_{name}_motor_joint")
            motor_joint.set("type", "fixed")
            
            motor_parent = ET.SubElement(motor_joint, "parent")
            motor_parent.set("link", f"{self.quad_id}_base_link")
            
            motor_child = ET.SubElement(motor_joint, "child")
            motor_child.set("link", f"{self.quad_id}_{name}_motor")
            
            motor_origin = ET.SubElement(motor_joint, "origin")
            motor_origin.set("xyz", f"{pos[0]} {pos[1]} {pos[2]}")
            motor_origin.set("rpy", "0 0 0")
            
            # Propeller link
            propeller_link = ET.SubElement(robot, "link")
            propeller_link.set("name", f"{self.quad_id}_{name}_propeller")
            
            # Propeller visual
            propeller_visual = ET.SubElement(propeller_link, "visual")
            propeller_geometry = ET.SubElement(propeller_visual, "geometry")
            propeller_cylinder = ET.SubElement(propeller_geometry, "cylinder")
            propeller_cylinder.set("radius", str(self.propeller_radius))
            propeller_cylinder.set("length", "0.01")
            
            propeller_material = ET.SubElement(propeller_visual, "material")
            propeller_material.set("name", "propeller_material")
            
            # Propeller collision
            propeller_collision = ET.SubElement(propeller_link, "collision")
            propeller_collision_geometry = ET.SubElement(propeller_collision, "geometry")
            propeller_collision_cylinder = ET.SubElement(propeller_collision_geometry, "cylinder")
            propeller_collision_cylinder.set("radius", str(self.propeller_radius))
            propeller_collision_cylinder.set("length", "0.01")
            
            # Propeller inertial
            propeller_inertial = ET.SubElement(propeller_link, "inertial")
            propeller_mass_elem = ET.SubElement(propeller_inertial, "mass")
            propeller_mass_elem.set("value", str(self.propeller_mass))
            
            propeller_inertia = ET.SubElement(propeller_inertial, "inertia")
            propeller_inertia.set("ixx", "0.001")
            propeller_inertia.set("ixy", "0.0")
            propeller_inertia.set("ixz", "0.0")
            propeller_inertia.set("iyy", "0.001")
            propeller_inertia.set("iyz", "0.0")
            propeller_inertia.set("izz", "0.001")
            
            # Propeller joint (continuous for rotation)
            propeller_joint = ET.SubElement(robot, "joint")
            propeller_joint.set("name", f"{self.quad_id}_{name}_propeller_joint")
            propeller_joint.set("type", "continuous")
            
            propeller_parent = ET.SubElement(propeller_joint, "parent")
            propeller_parent.set("link", f"{self.quad_id}_{name}_motor")
            
            propeller_child = ET.SubElement(propeller_joint, "child")
            propeller_child.set("link", f"{self.quad_id}_{name}_propeller")
            
            propeller_origin = ET.SubElement(propeller_joint, "origin")
            propeller_origin.set("xyz", "0 0 0.03")
            propeller_origin.set("rpy", "0 0 0")
            
            propeller_axis = ET.SubElement(propeller_joint, "axis")
            propeller_axis.set("xyz", "0 0 1")
            
            # Joint limits
            propeller_limit = ET.SubElement(propeller_joint, "limit")
            propeller_limit.set("lower", "-1000")
            propeller_limit.set("upper", "1000")
            propeller_limit.set("effort", "10")
            propeller_limit.set("velocity", "1000")
    
    def add_sensors(self, robot):
        """Add sensors (IMU, GPS, etc.)"""
        # IMU sensor
        imu_link = ET.SubElement(robot, "link")
        imu_link.set("name", f"{self.quad_id}_imu_link")
        
        # IMU joint
        imu_joint = ET.SubElement(robot, "joint")
        imu_joint.set("name", f"{self.quad_id}_imu_joint")
        imu_joint.set("type", "fixed")
        
        imu_parent = ET.SubElement(imu_joint, "parent")
        imu_parent.set("link", f"{self.quad_id}_base_link")
        
        imu_child = ET.SubElement(imu_joint, "child")
        imu_child.set("link", f"{self.quad_id}_imu_link")
        
        imu_origin = ET.SubElement(imu_joint, "origin")
        imu_origin.set("xyz", "0 0 0.05")
        imu_origin.set("rpy", "0 0 0")
        
        # GPS sensor
        gps_link = ET.SubElement(robot, "link")
        gps_link.set("name", f"{self.quad_id}_gps_link")
        
        # GPS joint
        gps_joint = ET.SubElement(robot, "joint")
        gps_joint.set("name", f"{self.quad_id}_gps_joint")
        gps_joint.set("type", "fixed")
        
        gps_parent = ET.SubElement(gps_joint, "parent")
        gps_parent.set("link", f"{self.quad_id}_base_link")
        
        gps_child = ET.SubElement(gps_joint, "child")
        gps_child.set("link", f"{self.quad_id}_gps_link")
        
        gps_origin = ET.SubElement(gps_joint, "origin")
        gps_origin.set("xyz", "0 0 0.05")
        gps_origin.set("rpy", "0 0 0")
    
    def add_plugins(self, robot):
        """Add Gazebo plugins for physics and control"""
        gazebo = ET.SubElement(robot, "gazebo")
        
        # IMU plugin
        imu_plugin = ET.SubElement(gazebo, "plugin")
        imu_plugin.set("name", f"{self.quad_id}_imu_plugin")
        imu_plugin.set("filename", "libgazebo_ros_imu.so")
        
        imu_topic = ET.SubElement(imu_plugin, "topicName")
        imu_topic.text = f"/quad_{self.quad_id}/imu"
        
        imu_link_name = ET.SubElement(imu_plugin, "linkName")
        imu_link_name.text = f"{self.quad_id}_imu_link"
        
        # GPS plugin
        gps_plugin = ET.SubElement(gazebo, "plugin")
        gps_plugin.set("name", f"{self.quad_id}_gps_plugin")
        gps_plugin.set("filename", "libgazebo_ros_gps.so")
        
        gps_topic = ET.SubElement(gps_plugin, "topicName")
        gps_topic.text = f"/quad_{self.quad_id}/gps"
        
        gps_link_name = ET.SubElement(gps_plugin, "linkName")
        gps_link_name.text = f"{self.quad_id}_gps_link"
        
        # Battery plugin
        battery_plugin = ET.SubElement(gazebo, "plugin")
        battery_plugin.set("name", f"{self.quad_id}_battery_plugin")
        battery_plugin.set("filename", "libgazebo_ros_battery.so")
        
        battery_topic = ET.SubElement(battery_plugin, "topicName")
        battery_topic.text = f"/quad_{self.quad_id}/battery"
        
        battery_capacity = ET.SubElement(battery_plugin, "capacity")
        battery_capacity.text = "2200"  # mAh
        
        battery_voltage = ET.SubElement(battery_plugin, "voltage")
        battery_voltage.text = "11.1"  # V
        
        # Flight dynamics plugin
        flight_plugin = ET.SubElement(gazebo, "plugin")
        flight_plugin.set("name", f"{self.quad_id}_flight_dynamics")
        flight_plugin.set("filename", "libgazebo_ros_flight_dynamics.so")
        
        # Flight parameters
        max_thrust = ET.SubElement(flight_plugin, "maxThrust")
        max_thrust.text = str(self.max_thrust)
        
        max_torque = ET.SubElement(flight_plugin, "maxTorque")
        max_torque.text = str(self.max_torque)
        
        drag_coeff = ET.SubElement(flight_plugin, "dragCoefficient")
        drag_coeff.text = str(self.drag_coefficient)
        
        lift_coeff = ET.SubElement(flight_plugin, "liftCoefficient")
        lift_coeff.text = str(self.lift_coefficient)
        
        # Motor topics
        for motor_name in ["front_right", "front_left", "back_left", "back_right"]:
            motor_topic = ET.SubElement(flight_plugin, f"{motor_name}MotorTopic")
            motor_topic.text = f"/quad_{self.quad_id}/{motor_name}_motor"
    
    def prettify_xml(self, elem):
        """Return a pretty-printed XML string for the Element."""
        rough_string = ET.tostring(elem, 'unicode')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

def generate_quadcopter_urdf(quad_id="quad_001"):
    """Generate URDF for a quadcopter"""
    generator = QuadcopterURDFGenerator(quad_id)
    return generator.generate_urdf()

if __name__ == "__main__":
    # Example usage
    urdf_content = generate_quadcopter_urdf("quad_001")
    print(urdf_content) 