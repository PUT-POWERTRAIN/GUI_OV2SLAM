from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Deklaruj argumenty launch
    vo_pose_topic_arg = DeclareLaunchArgument(
        'vo_pose_topic',
        default_value='/vo_pose',
        description='Topic name for VO pose'
    )
    
    image_track_topic_arg = DeclareLaunchArgument(  
        'image_track_topic',
        default_value='/image_track',
        description='Topic name for image tracking'
    )
    point_cloud_topic_arg = DeclareLaunchArgument(  
        'point_cloud_topic',
        default_value='/point_cloud',
        description='Topic name for point cloud tracking'
    )
    # Node z parametrami
    visualizer_node = Node(
        package='imgui_app',
        executable='imgui_visualizer',
        name='imgui_visualizer',
        output='screen',
        parameters=[{
            'vo_pose_topic': LaunchConfiguration('vo_pose_topic'),
            'image_track_topic': LaunchConfiguration('image_track_topic'),
            'point_cloud_topic': LaunchConfiguration('point_cloud_topic'),
        }]
    )
    
    return LaunchDescription([
        vo_pose_topic_arg,
        image_track_topic_arg,
        point_cloud_topic_arg,
        visualizer_node
    ])
