# GP — Global Planner

> **ROS 2 Humble | Gazebo Classic | C++17**

Autonomous GPS-to-GPS traversal of the 4-wheel skid-steer rover designed in the Group Task, implemented as a ROS 2 C++ package using Object-Oriented Programming principles.

---

## 📌 Task Description

The Global Planner node navigates the rover autonomously from its **current GPS position** to a **goal GPS coordinate** provided at launch (or via a topic at runtime). No obstacle avoidance is required — the path is assumed to be clear.

The planner:
1. Reads the rover's live GPS fix and IMU heading.
2. Computes the **Haversine distance** and **initial bearing** to the goal.
3. Uses a **PID controller** to minimise heading error (angular velocity) and a **proportional controller** for forward speed (linear velocity).
4. Publishes `geometry_msgs/Twist` to `/cmd_vel` until the rover is within the goal tolerance.

---

## 🏗️ OOP Architecture

| Class | Responsibility |
|---|---|
| `GeoUtils` | Static helpers: Haversine distance, bearing, angle wrapping, quaternion→yaw |
| `PIDController` | Generic discrete PID with integral wind-up clamping and reset |
| `RoverState` | Struct holding live GPS + IMU snapshot |
| `GlobalPlanner` | Main `rclcpp::Node`; owns state machine, PID instances, all pub/sub |

### State Machine

```
IDLE ──(GPS + IMU valid)──► NAVIGATING ──(dist ≤ tolerance)──► REACHED
  ▲                                                                   │
  └──────────────(new /goal_gps received)────────────────────────────┘
```

---

## 🔌 ROS Nodes, Topics & Messages

### Nodes

| Node | Executable | Description |
|---|---|---|
| `global_planner` | `gps_nav_node` | Core navigation controller |
| `goal_publisher` | `goal_publisher_node` | One-shot helper that publishes the goal GPS |

### Subscribed Topics

| Topic | Message Type | Source | Description |
|---|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | Gazebo GPS plugin | Current rover GPS position |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo IMU plugin | Current rover orientation (quaternion) |
| `/goal_gps` | `sensor_msgs/NavSatFix` | `goal_publisher_node` | Target GPS coordinate |

### Published Topics

| Topic | Message Type | Consumers | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `diff_drive` / Gazebo | Linear + angular velocity commands |
| `/planner/status` | `std_msgs/String` | RViz / logging | Current state: IDLE / NAVIGATING / REACHED |

### Launch Parameters

| Parameter | Default | Description |
|---|---|---|
| `goal_latitude` | `12.9726` | Goal latitude |
| `goal_longitude` | `77.5956` | Goal longitude |
| `goal_tolerance_m` | `1.0` | Arrival radius (m) |
| `max_linear_vel` | `0.5` | Max forward speed (m/s) |
| `max_angular_vel` | `1.0` | Max yaw rate (rad/s) |
| `linear_kp` | `0.3` | P gain for linear controller |
| `angular_kp` | `1.2` | P gain for heading PID |
| `angular_ki` | `0.0` | I gain for heading PID |
| `angular_kd` | `0.1` | D gain for heading PID |

---

## 📊 RQT Graph

```
┌─────────────────────┐       /gps/fix          ┌───────────────────┐
│  Gazebo GPS Plugin  │ ─────────────────────►  │                   │
└─────────────────────┘                          │                   │
                                                 │  global_planner   │ ──────► /cmd_vel ──► Gazebo diff_drive
┌─────────────────────┐       /imu/data          │  (gps_nav_node)   │
│  Gazebo IMU Plugin  │ ─────────────────────►  │                   │
└─────────────────────┘                          │                   │ ──────► /planner/status
                                                 └───────────────────┘
┌─────────────────────┐       /goal_gps                  ▲
│  goal_publisher     │ ─────────────────────────────────┘
│  _node              │
└─────────────────────┘
```

> **Generate live graph:**
> ```bash
> rqt_graph
> ```

---

## 🚀 Build & Run

### Prerequisites

```bash
# ROS 2 Humble + Gazebo Classic
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-robot-state-publisher \
                 ros-humble-xacro \
                 ros-humble-rviz2

source /opt/ros/humble/setup.bash
```

### Build

```bash
# From your workspace root (NOT ros2_ws — use a clean folder)
mkdir -p ~/gp_ws/src
cd ~/gp_ws/src
# Place this package here (clone / copy)
cd ~/gp_ws
colcon build --packages-select global_planner --symlink-install
source install/setup.bash
```

### Run with Gazebo

```bash
# Full stack — Gazebo + rover + planner
ros2 launch global_planner global_planner.launch.py \
    goal_latitude:=12.9726 goal_longitude:=77.5956

# Planner only (if Gazebo already running from rover_description)
ros2 launch global_planner planner_only.launch.py \
    goal_latitude:=12.9726 goal_longitude:=77.5956
```

### Send a new goal at runtime

```bash
ros2 topic pub --once /goal_gps sensor_msgs/msg/NavSatFix \
  "{latitude: 12.9730, longitude: 77.5960, altitude: 0.0,
    status: {status: 0}}"
```

### Check planner status

```bash
ros2 topic echo /planner/status
```

---

## 🎥 YouTube Video

> 📹 **[Watch the demo on YouTube](https://youtu.be/YOUR_VIDEO_ID_HERE)**
>
> *(Replace the link above after uploading your screen recording)*

---

## 📂 Package Structure

```
GP/
└── global_planner/
    ├── CMakeLists.txt
    ├── package.xml
    ├── src/
    │   ├── gps_nav_node.cpp          ← Main planner (OOP: GeoUtils, PIDController, GlobalPlanner)
    │   └── goal_publisher_node.cpp   ← One-shot goal publisher helper
    ├── launch/
    │   ├── global_planner.launch.py  ← Full stack (Gazebo + rover + planner)
    │   └── planner_only.launch.py    ← Planner nodes only
    ├── rviz/
    │   └── global_planner.rviz       ← RViz2 layout
    ├── config/                       ← (reserved for future YAML params)
    └── README.md
```

---

## 📐 Algorithm Detail

### Haversine Distance

$$d = 2R \cdot \arcsin\!\left(\sqrt{\sin^2\!\tfrac{\Delta\phi}{2} + \cos\phi_1\cos\phi_2\sin^2\!\tfrac{\Delta\lambda}{2}}\right)$$

### Bearing (ENU)

$$\theta_{ENU} = \frac{\pi}{2} - \arctan2\!\left(\sin\Delta\lambda\cos\phi_2,\; \cos\phi_1\sin\phi_2 - \sin\phi_1\cos\phi_2\cos\Delta\lambda\right)$$

### Control Law

```
heading_error  = wrap(desired_heading − current_yaw)
angular_vel    = PID(heading_error)                    // clipped to ±max_angular_vel
heading_factor = max(0, cos(heading_error))            // slow down when misaligned
linear_vel     = Kp_lin × distance × heading_factor   // clipped to [0, max_linear_vel]
```
