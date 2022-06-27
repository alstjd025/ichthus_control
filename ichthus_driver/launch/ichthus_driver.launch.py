import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    kia_cfg = os.path.join(
        get_package_share_directory('ichthus_driver'),
        'cfg',
        'kia.yaml'
    )
    mcm_cfg = os.path.join(
        get_package_share_directory('ichthus_driver'),
        'cfg',
        'mcm.yaml'
    )
    pid_cfg = os.path.join(
        get_package_share_directory('pid_control'),
        'cfg',
        'pid.yaml'
    )
    return LaunchDescription([
        Node(
            package = 'ichthus_driver',
            executable = 'mcm_management_node',
            parameters = [mcm_cfg],
            output="log",
        ),
        Node(
            package = 'ichthus_driver',
            executable = 'kia_reader_node',
            parameters = [kia_cfg],
            output="log",

        ),
        Node(
            package = 'pid_control',
            executable = 'pid_node',
            parameters = [pid_cfg,
            {
                'br_kp' : 150.0,
                'br_ki' : 80.0,
                'str_kp': 20.0,
                'str_ki' : 0.0, 
                'str_kd' : 10.0,
                # 'str_kp': 20.0, # 75.0. 125.0,
                # 'str_kd' : 10.0,
                'cur_angle_weight': 5.0,
                # 'cur_vel_weight': 5.0,
                # 'cur_vel_weight': 0.09,
                'cur_vel_weight': 0.09,
                'max_str': 3000.0,
                'imu_error': 0.5,
                'str_max_weight': 0.0,
                'right_thres' : 0.055,
                'left_thres' : -0.025,
                'str_minimum_thrs_buffer' : 0.0,
                # 'right_thres' : 0.0,
                # 'left_thres' : 0.0 
            }],
            output="screen",
        )
        
    ])



# str_kp: 25.0 # 75.0. 125.0
#     str_ki: 0.0
#     str_kd: 600.0 # 75.0, 125.0, 500.0
#     right_thres: 0.07
#     left_thres: -0.02