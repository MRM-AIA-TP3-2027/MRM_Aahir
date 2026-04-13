"""
global_planner.launch.py
────────────────────────
Launch sequence:
  1. Start Gazebo (empty world) with the rover_description URDF/Xacro
  2. Spawn the rover at the origin
  3. Start robot_state_publisher
  4. Start the GlobalPlanner (gps_nav_node)
  5. Start the GoalPublisher (goal_publisher_node)  [one-shot]
  6. (Optional) Start RViz2 with a pre-configured layout

Usage:
  ros2 launch global_planner global_planner.launch.py \\
      goal_latitude:=12.9726 goal_longitude:=77.5956

Arguments:
  goal_latitude       Target latitude         (default 12.9726)
  goal_longitude      Target longitude        (default 77.5956)
  goal_tolerance_m    Arrival radius in m     (default 1.0)
  max_linear_vel      Max fwd speed m/s       (default 0.5)
  max_angular_vel     Max yaw speed rad/s     (default 1.0)
  use_rviz            Launch RViz2            (default true)
  use_sim_time        Use /clock topic        (default true)
  world               Gazebo world SDF path   (default empty.world)
  rover_urdf          Path to rover URDF/Xacro
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package directories ────────────────────────────────────────────────────
    pkg_gp      = get_package_share_directory("global_planner")
    pkg_gazebo  = get_package_share_directory("gazebo_ros")

    # ── Launch arguments ───────────────────────────────────────────────────────
    goal_lat_arg = DeclareLaunchArgument(
        "goal_latitude", default_value="12.9726",
        description="Target latitude for autonomous navigation")

    goal_lon_arg = DeclareLaunchArgument(
        "goal_longitude", default_value="77.5956",
        description="Target longitude for autonomous navigation")

    goal_tol_arg = DeclareLaunchArgument(
        "goal_tolerance_m", default_value="1.0",
        description="Distance tolerance to declare goal reached (metres)")

    max_lin_arg = DeclareLaunchArgument(
        "max_linear_vel", default_value="0.5",
        description="Maximum linear velocity (m/s)")

    max_ang_arg = DeclareLaunchArgument(
        "max_angular_vel", default_value="1.0",
        description="Maximum angular velocity (rad/s)")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz2 visualiser")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use Gazebo simulation time")

    world_arg = DeclareLaunchArgument(
        "world", default_value="empty.world",
        description="Gazebo world file name (must be on GAZEBO_RESOURCE_PATH)")

    rover_urdf_arg = DeclareLaunchArgument(
        "rover_urdf",
        default_value=os.path.join(
            get_package_share_directory("rover_description"),
            "urdf", "rover.urdf.xacro")
        if _package_exists("rover_description") else "",
        description="Full path to rover URDF / Xacro file")

    # ── Launch config shortcuts ────────────────────────────────────────────────
    goal_lat      = LaunchConfiguration("goal_latitude")
    goal_lon      = LaunchConfiguration("goal_longitude")
    goal_tol      = LaunchConfiguration("goal_tolerance_m")
    max_lin       = LaunchConfiguration("max_linear_vel")
    max_ang       = LaunchConfiguration("max_angular_vel")
    use_rviz      = LaunchConfiguration("use_rviz")
    use_sim_time  = LaunchConfiguration("use_sim_time")
    world         = LaunchConfiguration("world")
    rover_urdf    = LaunchConfiguration("rover_urdf")

    # ── 1. Gazebo ─────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "verbose": "false",
        }.items(),
    )

    # ── 2. Robot State Publisher (for TF tree) ────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", rover_urdf]),
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # ── 3. Spawn rover in Gazebo ──────────────────────────────────────────────
    spawn_rover = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "rover",
            "-x", "0.0", "-y", "0.0", "-z", "0.1",
        ],
        output="screen",
    )

    # ── 4. GPS Nav node (Global Planner) ──────────────────────────────────────
    gps_nav_node = Node(
        package="global_planner",
        executable="gps_nav_node",
        name="global_planner",
        output="screen",
        parameters=[{
            "goal_latitude":    goal_lat,
            "goal_longitude":   goal_lon,
            "goal_tolerance_m": goal_tol,
            "max_linear_vel":   max_lin,
            "max_angular_vel":  max_ang,
            "use_sim_time":     use_sim_time,
            # PID gains — tune as needed
            "linear_kp":  0.3,
            "angular_kp": 1.2,
            "angular_ki": 0.0,
            "angular_kd": 0.1,
        }],
    )

    # ── 5. Goal Publisher (one-shot, delayed 3 s to let planner start) ─────────
    goal_pub_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="global_planner",
                executable="goal_publisher_node",
                name="goal_publisher",
                output="screen",
                parameters=[{
                    "goal_latitude":  goal_lat,
                    "goal_longitude": goal_lon,
                    "use_sim_time":   use_sim_time,
                }],
            )
        ],
    )

    # ── 6. RViz2 (optional) ────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_gp, "rviz", "global_planner.rviz")
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # ── Build LaunchDescription ────────────────────────────────────────────────
    return LaunchDescription([
        # Args
        goal_lat_arg, goal_lon_arg, goal_tol_arg,
        max_lin_arg, max_ang_arg, use_rviz_arg,
        use_sim_time_arg, world_arg, rover_urdf_arg,

        # Info
        LogInfo(msg="[GlobalPlanner] Launching Gazebo + Rover + GPS Nav ..."),

        # Nodes
        gazebo,
        robot_state_publisher,
        spawn_rover,
        gps_nav_node,
        goal_pub_node,
        rviz2_node,
    ])


# ── Helper ────────────────────────────────────────────────────────────────────
def _package_exists(pkg_name: str) -> bool:
    try:
        get_package_share_directory(pkg_name)
        return True
    except Exception:
        return False
