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

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
        ),
        

    ])
    
    
