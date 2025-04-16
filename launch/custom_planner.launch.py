from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Locate required packages
    tb3_nav2_pkg = get_package_share_directory("turtlebot3_navigation2")
    custom_pkg = get_package_share_directory("custom_global_planner")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_nav2_pkg, "launch", "navigation2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": os.path.join(custom_pkg, "config", "nav_params.yaml")
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])
