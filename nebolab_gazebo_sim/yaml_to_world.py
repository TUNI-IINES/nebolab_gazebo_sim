#!/usr/bin/python3
import os
import rclpy
from distutils.dir_util import copy_tree
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
from rclpy.node import Node


class Wall:
        def __init__(self, vertex1, vertex2):
            self.__vertex1 = np.array(vertex1[:2])
            self.__vertex2 = np.array(vertex2[:2])

        @property
        def center(self):
            return (self.__vertex1 + self.__vertex2) / 2

        @property
        def length(self):
            return np.linalg.norm(self.__vertex2 - self.__vertex1)

        @property
        def angle(self):
            return np.arctan2(self.__vertex2[1] - self.__vertex1[1],
                                self.__vertex2[0] - self.__vertex1[0])
        
class Converter(Node):
    def __init__(self, scenarios_path, models_path, worlds_path):
        super().__init__("yaml_to_world")
        self.declare_parameter("yaml_name", "world")
        self.yaml_name = self.get_parameter("yaml_name").value
        self.scenarios_path = scenarios_path
        self.models_path = models_path
        self.worlds_path = worlds_path

    def convert(self):
        self.get_logger().info(f"Trying to find {self.yaml_name} from {self.scenarios_path} for conversion.")

        if not os.path.exists(os.path.join(self.scenarios_path, f"{self.yaml_name}.yml")):
            self.get_logger().warning(f"File {self.yaml_name} does not exist. Can not convert.")
            return

            
        self.get_logger().info(
            "Convert environment obstacle file")
        with open(os.path.join(self.scenarios_path, f"{self.yaml_name}.yml"), "r") as file:
            scenario = yaml.safe_load(file).values()
            

            # Set the static obstacles
            obstacles = list(scenario)[0].get("obstacles")
            walls = [Wall(o[idx], o[idx + 1]) for o in obstacles for idx in range(len(o) - 1)]

        self.get_logger().info(
            f"Make a new obstacle folder: {os.path.join(self.models_path, self.yaml_name)}")

        
        copy_tree(os.path.join(self.models_path, "example_room"), os.path.join(self.models_path, self.yaml_name))
        self.get_logger().info(
            f"Edit the model file: {os.path.join(self.models_path, self.yaml_name, 'model.sdf')}"
            )
        with open(os.path.join(self.models_path, self.yaml_name, "model.sdf"), "w") as file:
            obstacles = "\n".join([f"""
            <collision name="obs_wall_{idx + 1}">
                <pose>{w.center[0]} {w.center[1]} 0.25 0 0 {w.angle:.3f}</pose>
                <geometry>
                    <box><size>{w.length:.3f} 0.01 0.5</size></box>
                </geometry>
                <max_contacts>10</max_contacts>
                <surface><bounce/><friction><ode/></friction><contact><ode/></contact></surface>
            </collision>

            <visual name="obs_wall_{idx + 1}">
                <pose>{w.center[0]} {w.center[1]} 0.25 0 0 {w.angle:.3f}</pose>
                <geometry>
                    <box><size>{w.length:.3f} 0.01 0.5</size></box>
                </geometry>
                <material>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Bricks</name>
                    </script>
                </material>
            </visual>
            """ for idx, w in enumerate(walls)])

            file.write(f"""
            <sdf version="1.5">
                <model name="ros_symbol">
                    <static>1</static>
                    <link name="symbol">
                        {obstacles}
                    </link>
                </model>
            </sdf>
            """)

        self.get_logger().info(
            f"Modify world file: {self.worlds_path}/{self.yaml_name}.world")
        with open(os.path.join(self.worlds_path, f"{self.yaml_name}.world"), "w") as file:
            file.write(f"""
            <?xml version="1.0"?>
            <sdf version="1.6">
            <world name="default">

                <include>
                <uri>model://ground_plane</uri>
                </include>

                <include>
                <uri>model://sun</uri>
                </include>

                <scene>
                <shadows>false</shadows>
                </scene>

                <gui fullscreen="0">
                <camera name="user_camera">
                    <pose frame="">0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>
                    <view_controller>orbit</view_controller>
                    <projection_type>perspective</projection_type>
                </camera>
                </gui>

                <physics type="ode">
                <real_time_update_rate>1000.0</real_time_update_rate>
                <max_step_size>0.001</max_step_size>
                <real_time_factor>1</real_time_factor>
                <ode>
                    <solver>
                    <type>quick</type>
                    <iters>150</iters>
                    <precon_iters>0</precon_iters>
                    <sor>1.400000</sor>
                    <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
                    </solver>
                    <constraints>
                    <cfm>0.00001</cfm>
                    <erp>0.2</erp>
                    <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
                    <contact_surface_layer>0.01000</contact_surface_layer>
                    </constraints>
                </ode>
                </physics>
                
                <model name="{self.yaml_name}">
                <static>1</static>
                <include>
                    <uri>model://{self.yaml_name}</uri>
                </include>
                </model>

            </world>
            </sdf>
            """)

        self.get_logger().info("World should be ready for use!")
        self.get_logger().info("You can kill this node now by pressing Ctrl+C.")
        return    


def main(args=None):
    rclpy.init(args=args)
    pkg_share_dir = get_package_share_directory("nebolab_gazebo_sim")
    scenarios_path = os.path.join(pkg_share_dir, "scenarios")
    models_path = os.path.join(pkg_share_dir, "models")
    worlds_path = os.path.join(pkg_share_dir, "worlds")
    
    node = Converter(scenarios_path, models_path, worlds_path)
    node.convert()

    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    node.get_logger().info("Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()