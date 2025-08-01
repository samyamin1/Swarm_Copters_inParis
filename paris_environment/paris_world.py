#!/usr/bin/env python3
"""
Paris World Model for Quadcopter Swarm Simulation
Creates realistic Paris environment with terrain, buildings, and landmarks
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
import math
import random

class ParisWorldGenerator:
    """
    Generates realistic Paris world model for Gazebo simulation
    Includes terrain, buildings, landmarks, and search and rescue scenarios
    """
    
    def __init__(self):
        self.world_name = "paris_swarm_world"
        
        # Paris coordinates and scale
        self.paris_center = [0, 0, 0]  # World coordinates
        self.world_size = 500  # meters
        self.building_height_range = [10, 50]  # meters
        
        # Landmarks and key locations
        self.landmarks = {
            'eiffel_tower': {'pos': [0, 0, 0], 'height': 324, 'scale': 0.1},
            'arc_de_triomphe': {'pos': [50, 0, 0], 'height': 50, 'scale': 1.0},
            'louvre': {'pos': [-50, 30, 0], 'height': 20, 'scale': 1.0},
            'notre_dame': {'pos': [-30, -40, 0], 'height': 69, 'scale': 1.0},
            'sacré_cœur': {'pos': [40, -30, 0], 'height': 83, 'scale': 1.0},
            'champs_élysées': {'pos': [25, 0, 0], 'height': 15, 'scale': 1.0},
            'seine_river': {'pos': [0, -20, 0], 'height': 0, 'scale': 1.0}
        }
        
        # Search and rescue targets
        self.search_targets = [
            {'pos': [20, 15, 0], 'type': 'victim', 'priority': 'high'},
            {'pos': [-25, -10, 0], 'type': 'victim', 'priority': 'medium'},
            {'pos': [35, -25, 0], 'type': 'victim', 'priority': 'low'},
            {'pos': [-15, 35, 0], 'type': 'emergency', 'priority': 'high'},
            {'pos': [45, 20, 0], 'type': 'emergency', 'priority': 'medium'}
        ]
        
        # Building clusters (simplified Paris districts)
        self.building_clusters = {
            'champs_elysees': {'center': [25, 0, 0], 'count': 15, 'height_range': [20, 40]},
            'louvre_district': {'center': [-50, 30, 0], 'count': 12, 'height_range': [15, 35]},
            'montmartre': {'center': [40, -30, 0], 'count': 20, 'height_range': [10, 25]},
            'seine_banks': {'center': [0, -20, 0], 'count': 18, 'height_range': [12, 30]},
            'business_district': {'center': [0, 50, 0], 'count': 25, 'height_range': [30, 60]}
        }
    
    def generate_world(self):
        """Generate complete Paris world for Gazebo"""
        world = ET.Element("sdf")
        world.set("version", "1.6")
        
        world_elem = ET.SubElement(world, "world")
        world_elem.set("name", self.world_name)
        
        # Add physics
        self.add_physics(world_elem)
        
        # Add lighting
        self.add_lighting(world_elem)
        
        # Add ground plane
        self.add_ground_plane(world_elem)
        
        # Add landmarks
        self.add_landmarks(world_elem)
        
        # Add building clusters
        self.add_building_clusters(world_elem)
        
        # Add search targets
        self.add_search_targets(world_elem)
        
        # Add weather effects
        self.add_weather_effects(world_elem)
        
        return self.prettify_xml(world)
    
    def add_physics(self, world):
        """Add physics configuration"""
        physics = ET.SubElement(world, "physics")
        physics.set("type", "ode")
        
        max_step_size = ET.SubElement(physics, "max_step_size")
        max_step_size.text = "0.001"
        
        real_time_factor = ET.SubElement(physics, "real_time_factor")
        real_time_factor.text = "1.0"
        
        real_time_update_rate = ET.SubElement(physics, "real_time_update_rate")
        real_time_update_rate.text = "1000"
        
        gravity = ET.SubElement(physics, "gravity")
        gravity.text = "0 0 -9.81"
        
        # ODE physics engine
        ode = ET.SubElement(physics, "ode")
        
        solver = ET.SubElement(ode, "solver")
        type_elem = ET.SubElement(solver, "type")
        type_elem.text = "quick"
        
        iters = ET.SubElement(solver, "iters")
        iters.text = "50"
        
        sor = ET.SubElement(solver, "sor")
        sor.text = "1.3"
        
        # Constraints
        constraints = ET.SubElement(ode, "constraints")
        cfm = ET.SubElement(constraints, "cfm")
        cfm.text = "0.0"
        
        erp = ET.SubElement(constraints, "erp")
        erp.text = "0.2"
        
        contact_max_correcting_vel = ET.SubElement(constraints, "contact_max_correcting_vel")
        contact_max_correcting_vel.text = "100.0"
        
        contact_surface_layer = ET.SubElement(constraints, "contact_surface_layer")
        contact_surface_layer.text = "0.001"
    
    def add_lighting(self, world):
        """Add lighting configuration"""
        # Sun light
        light = ET.SubElement(world, "light")
        light.set("type", "directional")
        light.set("name", "sun")
        
        cast_shadows = ET.SubElement(light, "cast_shadows")
        cast_shadows.text = "true"
        
        diffuse = ET.SubElement(light, "diffuse")
        diffuse.text = "0.8 0.8 0.8 1"
        
        specular = ET.SubElement(light, "specular")
        specular.text = "0.2 0.2 0.2 1"
        
        attenuation = ET.SubElement(light, "attenuation")
        range_elem = ET.SubElement(attenuation, "range")
        range_elem.text = "1000"
        
        direction = ET.SubElement(light, "direction")
        direction.text = "-0.5 0.1 -0.9"
        
        # Ambient light
        ambient = ET.SubElement(world, "ambient")
        ambient.text = "0.4 0.4 0.4 1"
        
        # Sky
        sky = ET.SubElement(world, "sky")
        sky.set("name", "sky")
        
        sky_ambient = ET.SubElement(sky, "ambient")
        sky_ambient.text = "0.1 0.1 0.1 1"
        
        sky_background = ET.SubElement(sky, "background")
        sky_background.text = "0.8 0.8 0.8 1"
        
        sky_clouds = ET.SubElement(sky, "clouds")
        sky_clouds.set("speed", "12")
        sky_clouds.set("direction", "1")
        sky_clouds.set("humidity", "0.65")
        sky_clouds.set("mean_size", "0.08")
        sky_clouds.set("ambient", "0.8 0.8 0.8")
    
    def add_ground_plane(self, world):
        """Add ground plane with Paris terrain"""
        ground = ET.SubElement(world, "model")
        ground.set("name", "ground_plane")
        
        static = ET.SubElement(ground, "static")
        static.text = "true"
        
        # Ground plane link
        ground_link = ET.SubElement(ground, "link")
        ground_link.set("name", "ground_link")
        
        # Ground visual
        ground_visual = ET.SubElement(ground_link, "visual")
        ground_visual.set("name", "ground_visual")
        
        ground_geometry = ET.SubElement(ground_visual, "geometry")
        ground_plane = ET.SubElement(ground_geometry, "plane")
        
        ground_normal = ET.SubElement(ground_plane, "normal")
        ground_normal.text = "0 0 1"
        
        ground_size = ET.SubElement(ground_plane, "size")
        ground_size.text = f"{self.world_size} {self.world_size}"
        
        # Ground material
        ground_material = ET.SubElement(ground_visual, "material")
        ground_script = ET.SubElement(ground_material, "script")
        
        ground_uri = ET.SubElement(ground_script, "uri")
        ground_uri.text = "file://media/materials/scripts/gazebo.material"
        
        ground_material_name = ET.SubElement(ground_script, "materialName")
        ground_material_name.text = "Gazebo/Grass"
        
        # Ground collision
        ground_collision = ET.SubElement(ground_link, "collision")
        ground_collision.set("name", "ground_collision")
        
        ground_collision_geometry = ET.SubElement(ground_collision, "geometry")
        ground_collision_plane = ET.SubElement(ground_collision_geometry, "plane")
        
        ground_collision_normal = ET.SubElement(ground_collision_plane, "normal")
        ground_collision_normal.text = "0 0 1"
        
        ground_collision_size = ET.SubElement(ground_collision_plane, "size")
        ground_collision_size.text = f"{self.world_size} {self.world_size}"
        
        # Ground inertial
        ground_inertial = ET.SubElement(ground_link, "inertial")
        ground_mass = ET.SubElement(ground_inertial, "mass")
        ground_mass.text = "1"
        
        ground_inertia = ET.SubElement(ground_inertial, "inertia")
        ground_inertia.set("ixx", "1")
        ground_inertia.set("ixy", "0")
        ground_inertia.set("ixz", "0")
        ground_inertia.set("iyy", "1")
        ground_inertia.set("iyz", "0")
        ground_inertia.set("izz", "1")
    
    def add_landmarks(self, world):
        """Add Paris landmarks"""
        for landmark_name, landmark_data in self.landmarks.items():
            landmark = ET.SubElement(world, "model")
            landmark.set("name", landmark_name)
            
            static = ET.SubElement(landmark, "static")
            static.text = "true"
            
            # Landmark link
            landmark_link = ET.SubElement(landmark, "link")
            landmark_link.set("name", f"{landmark_name}_link")
            
            # Landmark visual
            landmark_visual = ET.SubElement(landmark_link, "visual")
            landmark_visual.set("name", f"{landmark_name}_visual")
            
            landmark_geometry = ET.SubElement(landmark_visual, "geometry")
            
            if landmark_name == 'eiffel_tower':
                # Special Eiffel Tower geometry
                landmark_box = ET.SubElement(landmark_geometry, "box")
                landmark_box.set("size", f"{landmark_data['scale']*10} {landmark_data['scale']*10} {landmark_data['height']*landmark_data['scale']}")
            else:
                # Standard building geometry
                landmark_box = ET.SubElement(landmark_geometry, "box")
                landmark_box.set("size", f"{landmark_data['scale']*20} {landmark_data['scale']*20} {landmark_data['height']*landmark_data['scale']}")
            
            # Landmark material
            landmark_material = ET.SubElement(landmark_visual, "material")
            landmark_script = ET.SubElement(landmark_material, "script")
            
            landmark_uri = ET.SubElement(landmark_script, "uri")
            landmark_uri.text = "file://media/materials/scripts/gazebo.material"
            
            landmark_material_name = ET.SubElement(landmark_script, "materialName")
            if landmark_name == 'eiffel_tower':
                landmark_material_name.text = "Gazebo/Red"
            else:
                landmark_material_name.text = "Gazebo/Gray"
            
            # Landmark collision
            landmark_collision = ET.SubElement(landmark_link, "collision")
            landmark_collision.set("name", f"{landmark_name}_collision")
            
            landmark_collision_geometry = ET.SubElement(landmark_collision, "geometry")
            landmark_collision_box = ET.SubElement(landmark_collision_geometry, "box")
            
            if landmark_name == 'eiffel_tower':
                landmark_collision_box.set("size", f"{landmark_data['scale']*10} {landmark_data['scale']*10} {landmark_data['height']*landmark_data['scale']}")
            else:
                landmark_collision_box.set("size", f"{landmark_data['scale']*20} {landmark_data['scale']*20} {landmark_data['height']*landmark_data['scale']}")
            
            # Landmark pose
            landmark_pose = ET.SubElement(landmark, "pose")
            landmark_pose.text = f"{landmark_data['pos'][0]} {landmark_data['pos'][1]} {landmark_data['height']*landmark_data['scale']/2} 0 0 0"
    
    def add_building_clusters(self, world):
        """Add building clusters representing Paris districts"""
        for district_name, district_data in self.building_clusters.items():
            for i in range(district_data['count']):
                building = ET.SubElement(world, "model")
                building.set("name", f"{district_name}_building_{i}")
                
                static = ET.SubElement(building, "static")
                static.text = "true"
                
                # Building dimensions
                building_width = random.uniform(8, 15)
                building_length = random.uniform(8, 15)
                building_height = random.uniform(district_data['height_range'][0], district_data['height_range'][1])
                
                # Building position within district
                district_center = district_data['center']
                building_x = district_center[0] + random.uniform(-30, 30)
                building_y = district_center[1] + random.uniform(-30, 30)
                building_z = building_height / 2
                
                # Building link
                building_link = ET.SubElement(building, "link")
                building_link.set("name", f"{district_name}_building_{i}_link")
                
                # Building visual
                building_visual = ET.SubElement(building_link, "visual")
                building_visual.set("name", f"{district_name}_building_{i}_visual")
                
                building_geometry = ET.SubElement(building_visual, "geometry")
                building_box = ET.SubElement(building_geometry, "box")
                building_box.set("size", f"{building_width} {building_length} {building_height}")
                
                # Building material
                building_material = ET.SubElement(building_visual, "material")
                building_script = ET.SubElement(building_material, "script")
                
                building_uri = ET.SubElement(building_script, "uri")
                building_uri.text = "file://media/materials/scripts/gazebo.material"
                
                building_material_name = ET.SubElement(building_script, "materialName")
                building_material_name.text = "Gazebo/Stone"
                
                # Building collision
                building_collision = ET.SubElement(building_link, "collision")
                building_collision.set("name", f"{district_name}_building_{i}_collision")
                
                building_collision_geometry = ET.SubElement(building_collision, "geometry")
                building_collision_box = ET.SubElement(building_collision_geometry, "box")
                building_collision_box.set("size", f"{building_width} {building_length} {building_height}")
                
                # Building pose
                building_pose = ET.SubElement(building, "pose")
                building_pose.text = f"{building_x} {building_y} {building_z} 0 0 0"
    
    def add_search_targets(self, world):
        """Add search and rescue targets"""
        for i, target in enumerate(self.search_targets):
            target_model = ET.SubElement(world, "model")
            target_model.set("name", f"search_target_{i}")
            
            static = ET.SubElement(target_model, "static")
            static.text = "true"
            
            # Target link
            target_link = ET.SubElement(target_model, "link")
            target_link.set("name", f"search_target_{i}_link")
            
            # Target visual
            target_visual = ET.SubElement(target_link, "visual")
            target_visual.set("name", f"search_target_{i}_visual")
            
            target_geometry = ET.SubElement(target_visual, "geometry")
            target_sphere = ET.SubElement(target_geometry, "sphere")
            target_sphere.set("radius", "1.0")
            
            # Target material (color based on priority)
            target_material = ET.SubElement(target_visual, "material")
            target_script = ET.SubElement(target_material, "script")
            
            target_uri = ET.SubElement(target_script, "uri")
            target_uri.text = "file://media/materials/scripts/gazebo.material"
            
            target_material_name = ET.SubElement(target_script, "materialName")
            if target['priority'] == 'high':
                target_material_name.text = "Gazebo/Red"
            elif target['priority'] == 'medium':
                target_material_name.text = "Gazebo/Yellow"
            else:
                target_material_name.text = "Gazebo/Green"
            
            # Target collision
            target_collision = ET.SubElement(target_link, "collision")
            target_collision.set("name", f"search_target_{i}_collision")
            
            target_collision_geometry = ET.SubElement(target_collision, "geometry")
            target_collision_sphere = ET.SubElement(target_collision_geometry, "sphere")
            target_collision_sphere.set("radius", "1.0")
            
            # Target pose
            target_pose = ET.SubElement(target_model, "pose")
            target_pose.text = f"{target['pos'][0]} {target['pos'][1]} {target['pos'][2]} 0 0 0"
    
    def add_weather_effects(self, world):
        """Add weather effects for realism"""
        # Wind
        wind = ET.SubElement(world, "wind")
        wind.set("name", "wind")
        
        wind_velocity = ET.SubElement(wind, "velocity")
        wind_velocity.text = "2 1 0"
        
        wind_gust = ET.SubElement(wind, "gust")
        wind_gust.text = "0.5"
        
        # Fog
        fog = ET.SubElement(world, "fog")
        fog.set("name", "fog")
        
        fog_color = ET.SubElement(fog, "color")
        fog_color.text = "0.8 0.8 0.8"
        
        fog_density = ET.SubElement(fog, "density")
        fog_density.text = "0.1"
        
        fog_start = ET.SubElement(fog, "start")
        fog_start.text = "10"
        
        fog_end = ET.SubElement(fog, "end")
        fog_end.text = "100"
    
    def prettify_xml(self, elem):
        """Return a pretty-printed XML string for the Element."""
        rough_string = ET.tostring(elem, 'unicode')
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ")

def generate_paris_world():
    """Generate Paris world for Gazebo simulation"""
    generator = ParisWorldGenerator()
    return generator.generate_world()

if __name__ == "__main__":
    # Example usage
    world_content = generate_paris_world()
    print(world_content) 