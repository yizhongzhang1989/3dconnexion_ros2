# 3DConnexion SpaceMouse ROS2 Dashboard

ROS2 package that reads 3Dconnexion SpaceMouse data and provides a real-time web dashboard for visualization. Includes the [`spacenav`](https://github.com/ros-drivers/joystick_drivers/tree/ros2/spacenav) driver — no external dependencies to clone.

Can be used **standalone** or as a **git submodule** inside an existing workspace.

## Features

- Includes the `spacenav` ROS2 driver — no extra repos to clone
- Web dashboard with live 3D cube preview responding to 6-DOF input
- Real-time axis bar charts (linear + angular)
- Button state indicators
- Raw joystick value readout
- Configurable via launch arguments or YAML

## Prerequisites

```bash
# spacenavd daemon and library (required by the spacenav driver)
sudo apt install spacenavd libspnav-dev
```

## Installation

### Standalone

```bash
mkdir -p ~/spacemouse_ws/src
cd ~/spacemouse_ws/src
git clone <this-repo-url> 3dconnexion_ros2
cd ~/spacemouse_ws
colcon build --symlink-install
source install/setup.bash
```

### As a submodule

```bash
cd ~/your_ws/src
git submodule add <this-repo-url> 3dconnexion_ros2
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

> **Note:** The `spacenavd` daemon starts automatically on boot after installation.
> If it's not running, start it with `sudo systemctl start spacenavd`.

### Launch everything (SpaceMouse + Dashboard)

```bash
ros2 launch spacemouse_dashboard spacemouse_dashboard.launch.py
```

Then open **http://localhost:8080** in your browser.

### Launch individually

```bash
# SpaceMouse driver only
ros2 launch spacemouse_dashboard spacemouse.launch.py

# Dashboard only (spacenav must already be running)
ros2 launch spacemouse_dashboard dashboard.launch.py
```

### Launch arguments

| Argument    | Default | Description                      |
|-------------|---------|----------------------------------|
| `http_port` | `8080`  | HTTP port for the web UI + data  |

```bash
ros2 launch spacemouse_dashboard spacemouse_dashboard.launch.py http_port:=3000
```

## Topics

Subscribed by the dashboard (published by the `spacenav` driver):

| Topic                  | Type                          | Description                        |
|------------------------|-------------------------------|------------------------------------|
| `spacenav/twist`       | `geometry_msgs/msg/Twist`     | Combined linear + angular velocity |
| `spacenav/offset`      | `geometry_msgs/msg/Vector3`   | Linear offset (scaled)             |
| `spacenav/rot_offset`  | `geometry_msgs/msg/Vector3`   | Angular offset (scaled)            |
| `spacenav/joy`         | `sensor_msgs/msg/Joy`         | Raw axes + buttons                 |

## Package structure

```
3dconnexion_ros2/
├── .gitignore
├── spacenav/                          # spacenav driver (from joystick_drivers)
│   ├── cmake/FindSPNAV.cmake
│   ├── include/spacenav/spacenav.hpp
│   ├── src/spacenav.cpp
│   ├── launch/classic-launch.py
│   ├── CMakeLists.txt
│   └── package.xml
├── spacemouse_dashboard/              # Dashboard ROS2 package
│   ├── config/
│   │   └── spacenav_params.yaml
│   ├── launch/
│   │   ├── spacemouse.launch.py
│   │   ├── dashboard.launch.py
│   │   └── spacemouse_dashboard.launch.py
│   ├── spacemouse_dashboard/
│   │   ├── __init__.py
│   │   └── dashboard_node.py
│   ├── web/
│   │   └── index.html
│   ├── resource/spacemouse_dashboard
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── README.md
```

## License

MIT
