# Copyright 2023 xFranv8
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bumpgo_cmd = Node(package="fsm_bump_go_cpp",
                      executable="bumpgo",
                      output="screen",
                      remappings=[
                        ("input_scan", "/scan"),
                        ("output_vel", "/cmd_vel"),
                        ("input_image", "/camera/color/image_raw"),
                        ("input_imu", "/imu_broadcaster/imu")
                      ])
    
    ld = LaunchDescription()
    ld.add_action(bumpgo_cmd)

    return ld