from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([              
        # tello node
        Node(
            package='tello',
            executable='tello',
            name='tello',
            remappings=[('/image_raw', '/image')],  # Remap to drone camera feed
            #output='screen'
        ),
        # Launch the drone control node
        Node(
            package='drone_control',
            executable='control',
            name='tello_control_node',
            remappings=[('/control', '/secure_cmd')],
            #output='screen'
        ),
        


        # Launch zbar_ros to read QR codes
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='qr_code_reader',
        ),
        
        Node(
            package='drone_control',
            executable='qr_code_follower',
            name='qr_code_follower',
            remappings=[('/control', '/control_qr')],
        ),

        Node(
            package='drone_control',
            executable='travelling',
            name='travelling',
            remappings=[('/control', '/control_travelling')],
        ),

        Node(
            package='drone_control',
            executable='monitoring_mode',
            name='monitoring_mode',
            remappings=[('/control', '/control_monitoring')],
        ),


        Node(
            package='joy',
            executable='joy_node',
            name='joy',
        ),
        

    ])
    
    
