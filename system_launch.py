from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            parameters=[{'serial_baudrate': 460800}]
        ),

        # Servo control nodes
        Node(
            package='servo_control',
            executable='scan_sequence_node',
            name='scan_sequence_node'
        ),
        Node(
            package='servo_control',
            executable='servo_motor_node.py',
            name='servo_motor_node'
        ),
        Node(
            package='servo_control',
            executable='dynamic_tf_broadcaster',
            name='dynamic_tf_broadcaster',
            parameters=[
                {'parent_frame': 'base_link'},
                {'child_frame': 'laser_frame'},
                {'offset_x': 0.0},
                {'offset_y': 0.0},
                {'offset_z': 0.0}
            ]
        ),
        Node(
            package='servo_control',
            executable='lidar_pointcloud_node',
            name='lidar_pointcloud_node'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fcu_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'fcu', 'base_link'],
        ),

        # Octomap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap',
            remappings=[('cloud_in', '/lidar_points')],
            parameters=[
                {'queue_size': 1000},
                {'resolution': 0.1},
                {'sensor_model/max_range': 10.0},
                {'sensor_model/hit': 0.7},
                {'sensor_model/miss': 0.4},
                {'frame_id': 'base_link'},
                {'transform_tolerance': 0.5}
            ]
        ),
        Node(
            package='servo_control',
            executable='lidar_bbox_open3d.py',
            name='lidar_bbox_open3d'
        ),
    ])

