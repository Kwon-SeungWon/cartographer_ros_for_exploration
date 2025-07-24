#!/usr/bin/env python3

import os
import shutil
from pathlib import Path
from ruamel.yaml import YAML

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

from ament_index_python.packages import get_package_share_directory #임시

class FileManager():
    def __init__(self, pkg_share_directory):
        print("start_map_manager")
        maps_directory = os.path.join(pkg_share_directory, "maps")
        self.file_list_ = []
        self.map_list_ = []

    def create_new_file(self, src_path, install_dir, file_name):
        install_path = os.path.join(install_dir, file_name)

        os.makedirs(os.path.dirname(src_path), exist_ok=True)

        config_content = """# Example ROS 2 Configuration
setting_1: value_1
setting_2: value_2
"""
        with open(src_path, "w") as f:
            f.write(config_content)
        print(f"Created config file at {src_path}")

        os.makedirs(install_dir, exist_ok=True)

        if os.path.exists(install_path) or os.path.islink(install_path):
            os.remove(install_path)

        os.symlink(src_path, install_path)
        print(f"Created symbolic link: {install_path} -> {src_path}")

    def delete_file(self, folder_path, filename):
        file_path = os.path.join(folder_path, filename)

        if not os.path.exists(file_path):
            print(f"File does not exist: {file_path}")
            return

        if os.path.islink(file_path):
            try:
                target_path = os.readlink(file_path)
                os.unlink(file_path)
                print(f"Deleted symbolic link: {file_path}")

                if os.path.exists(target_path) and not os.path.islink(target_path):
                    try:
                        os.remove(target_path)
                        print(f"Deleted original file: {target_path}")
                    except Exception as e:
                        print(f"Error deleting original file: {e}")
            except Exception as e:
                print(f"Error handling symbolic link: {e}")

        else:
            try:
                os.remove(file_path)
                print(f"Deleted file: {file_path}")
            except Exception as e:
                print(f"Error deleting file: {e}")

    def update_yaml_param(self, file_path, update_key, update_value):
        yaml = YAML()
        yaml.preserve_quotes = True 
        yaml.indent(mapping=2, sequence=4, offset=2)
        try:
            with open(file_path, "r", encoding="utf-8") as file:
                yaml_data = yaml.load(file)

            if update_key in yaml_data:
                yaml_data[update_key] = update_value
            else:
                print(f"{update_key} not exists.")

            with open(file_path, "w", encoding="utf-8") as file:
                yaml.dump(yaml_data, file)

            print(f"YAML file update: {update_key} : {update_value}")

        except FileNotFoundError:
            print(f"file not found: {file_path}")
        except yaml.YAMLError as e:
            print(f"yaml error: {e}")

    def get_file_list(self, dir_path, extension):
        return [file for file in os.listdir(dir_path) if file.endswith(extension)]


    def get_file(self, src_dir_path, dest_dir_path ,file_name):
        src_file_path = os.path.join(src_dir_path, file_name)
        dest_file_path = os.path.join(dest_dir_path, file_name)

        if not os.path.exists(src_file_path):
            print(f"Error: Source file does not exist: {src_file_path}")
            return False

        if not os.path.exists(dest_dir_path):
            os.makedirs(dest_dir_path)

        try:
            shutil.copy2(src_file_path, dest_file_path)
            print(f"File copied successfully: {dest_file_path}")
            return True
        except Exception as e:
            print(f"Error copying file: {e}")
            return False


    def save_map(self):
        pass
    
    def change_map(self):
        pass
    
    def change_nodelist(self):
        pass   
    

if __name__ == "__main__":
    package_name = "webserver_interface_ros2"
    package_share_directory = get_package_share_directory(package_name)
    mm = FileManager(package_share_directory)

    src_path = "/ros2_ws/src/webserver_interface_ros2/config/config.yaml"
    install_dir = "/ros2_ws/install/webserver_interface_ros2/share/webserver_interface_ros2/config/"
    file_name = "config.yaml"
    # mm.create_new_file(src_path, install_dir, file_name)
    # mm.delete_file("/ros2_ws/install/webserver_interface_ros2/share/webserver_interface_ros2/config/","config.yaml")
    mm.update_yaml_param(src_path,"currentMap","test_map")
    mm.update_yaml_param(src_path,"currentNode","test_map")
    print(mm.get_file_list("/ros2_ws/src/webserver_interface_ros2/maps",".pgm"))



'''
노드파일 생성(origin, shared)
노드파일 삭제(origin, shared)

지도파일 생성(origin, shared)
지도파일 삭제(origin, shared)


'''