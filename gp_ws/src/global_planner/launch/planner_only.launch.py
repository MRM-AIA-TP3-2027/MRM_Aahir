"""
planner_only.launch.py
───────────────────────
Launches ONLY the GPS nav node and goal publisher — no Gazebo.
Use this when Gazebo + rover are already running (e.g. from rover_description).

Usage:
  ros2 launch global_planner planner_only.launch.py \\
      goal_latitude:=12.9726 goal_longitude:=77.5956
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    goal_lat_arg = DeclareLaunchArgument("goal_latitude",  default_value="12.9726")
    goal_lon_arg = DeclareLaunchArgument("goal_longitude", default_value="77.5956")
    tol_arg      = DeclareLaunchArgument("goal_tolerance_m", default_value="1.0")
    lin_arg      = DeclareLaunchArgument("max_linear_vel",   default_value="0.5")
    ang_arg      = DeclareLaunchArgument("max_angular_vel",  default_value="1.0")
    sim_arg      = DeclareLaunchArgument("use_sim_time",     default_value="true")

    goal_lat = LaunchConfiguration("goal_latitude")
    goal_lon = LaunchConfiguration("goal_longitude")
    tol      = LaunchConfiguration("goal_tolerance_m")
    lin      = LaunchConfiguration("max_linear_vel")
    ang      = LaunchConfiguration("max_angular_vel")
    sim      = LaunchConfiguration("use_sim_time")

    planner = Node(
        package="global_planner",
        executable="gps_nav_node",
        name="global_planner",
        output="screen",
        parameters=[{
            "goal_latitude":    goal_lat,
            "goal_longitude":   goal_lon,
            "goal_tolerance_m": tol,
            "max_linear_vel":   lin,
            "max_angular_vel":  ang,
            "use_sim_time":     sim,
            "linear_kp":  0.3,
            "angular_kp": 1.2,
            "angular_ki": 0.0,
            "angular_kd": 0.1,
        }],
    )

    goal_publisher = TimerAction(
        period=2.0,
        actions=[Node(
            package="global_planner",
            executable="goal_publisher_node",
            name="goal_publisher",
            output="screen",
            parameters=[{
                "goal_latitude":  goal_lat,
                "goal_longitude": goal_lon,
                "use_sim_time":   sim,
            }],
        )],
    )

    return LaunchDescription([
        goal_lat_arg, goal_lon_arg, tol_arg,
        lin_arg, ang_arg, sim_arg,
        LogInfo(msg="[GlobalPlanner] Starting planner nodes only ..."),
        planner,
        goal_publisher,
    ])
