# 3DConnexion SpaceMouse ROS2 Dashboard

ROS2 package that reads 3Dconnexion SpaceMouse data and provides a real-time web dashboard for visualization. Includes the [`spacenav`](https://github.com/ros-drivers/joystick_drivers/tree/ros2/spacenav) driver — no external dependencies to clone.

Can be used **standalone** or as a **git submodule** inside an existing workspace.

## Coordinate System

<img src="doc/spacemouse_coordinate.jpg" alt="SpaceMouse Coordinate System" width="400">

> **Tip:** For the most intuitive control, align the coordinate system of your control target (e.g. robot end-effector) with the SpaceMouse. When the axes match, pushing the SpaceMouse forward moves the target forward, and rotating it maps directly to the target's rotation — matching natural human expectation.

## Features

- Includes the `spacenav` ROS2 driver — no extra repos to clone
- Web dashboard with live 3D cube preview responding to 6-DOF input
- Real-time axis bar charts (linear + angular)
- Named button panel laid out like the physical SpaceMouse Pro (15 buttons)
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
ros2 launch spacemouse spacemouse.launch.py dashboard_port:=8080
```

Then open **http://localhost:8080** in your browser.

### Launch individually

```bash
# SpaceMouse driver only (no dashboard)
ros2 launch spacemouse spacemouse.launch.py

# Dashboard only (spacenav must already be running)
ros2 launch spacemouse dashboard.launch.py
```

### Launch arguments

| Argument         | Default | Description                                              |
|------------------|---------|----------------------------------------------------------|
| `dashboard_port` | (empty) | `spacemouse.launch.py`: if set (e.g. `8080`), also start the dashboard on this port. Empty = driver only. |
| `http_port`      | `8080`  | `dashboard.launch.py`: HTTP port for the web UI + data.  |

```bash
ros2 launch spacemouse spacemouse.launch.py dashboard_port:=3000
```

## Topics

Subscribed by the dashboard (published by the `spacenav` driver):

| Topic                  | Type                          | Description                        |
|------------------------|-------------------------------|------------------------------------|
| `spacenav/twist`       | `geometry_msgs/msg/Twist`     | Combined linear + angular velocity |
| `spacenav/offset`      | `geometry_msgs/msg/Vector3`   | Linear offset (scaled)             |
| `spacenav/rot_offset`  | `geometry_msgs/msg/Vector3`   | Angular offset (scaled)            |
| `spacenav/joy`         | `sensor_msgs/msg/Joy`         | Raw axes + fixed 15-button array (see [Buttons](#buttons-spacemouse-pro)) |

## Buttons (SpaceMouse Pro)

This package targets the **3Dconnexion SpaceMouse Pro** (15 buttons). The
`spacenav` driver reports button state in `spacenav/joy` as a **fixed-width**
`sensor_msgs/msg/Joy` `buttons[]` array (`1` = pressed, `0` = released). Every
index always maps to the same physical button, so each message reflects the
status of every button — they no longer appear only after first being pressed.

| Index | Button          | Group        |
|-------|-----------------|--------------|
| 0     | `1`             | Function key |
| 1     | `2`             | Function key |
| 2     | `3`             | Function key |
| 3     | `4`             | Function key |
| 4     | Menu            | Menu         |
| 5     | Fit             | Fit          |
| 6     | T (top view)    | QuickView    |
| 7     | R (right view)  | QuickView    |
| 8     | F (front view)  | QuickView    |
| 9     | Roll view       | QuickView    |
| 10    | Rotation toggle | Rotation     |
| 11    | Esc             | Modifier     |
| 12    | Alt             | Modifier     |
| 13    | Shift           | Modifier     |
| 14    | Ctrl            | Modifier     |

The dashboard's **Buttons** panel arranges these **radially, matching the
physical device**: the function keys (`1`–`4`) arc over the top, the keyboard
modifiers (`Esc` / `Shift` / `Ctrl` / `Alt`) and `Menu` sit to the left of the
cap, the four QuickView keys form a 2×2 with the **rotation toggle in its
center** (and `Fit` below) to the right of the cap, and the controller cap sits
in the middle. Each named key lights up when it is pressed.

> **Notes**
> - The fixed width is set by the `spacenav_node` parameter `num_buttons`
>   (default `15`). The array still grows automatically if a device ever reports
>   a higher index.
> - The contiguous `0–14` mapping above is produced by `spacenavd` for the
>   SpaceMouse Pro. The Ubuntu-packaged `spacenavd` 0.7.1 mis-maps the Pro's
>   buttons (sparse indices); build a recent `spacenavd` (≥ 1.x) from source if
>   your buttons don't line up with the table.

### Known limitation — multi-button ghosting

Pressing **three or more buttons at once** can light up a **phantom button** you
did not press (and some combinations drop or swap a real one). Examples observed
directly on the raw `spacenav/joy` topic:

| Buttons pressed | Reported on topic |
|-----------------|-------------------|
| 0 + 1 + 2       | 0, 1, 2, **9**    |
| 0 + 1 + 3       | 0, 1, 3, **6**    |
| 0 + 1 + 9       | 0, 1, **7**       |

Pressing **one or two** buttons is always reported correctly.

This is **button-matrix ghosting** — a hardware characteristic of the SpaceMouse
Pro, whose buttons share a row/column matrix without full N-key rollover (no
anti-ghost diodes). It is **not a bug in this package**: the phantom is already
present in the data coming from the device, below both `spacenavd` and the
`spacenav` driver, which only pass each button's state through individually.
Because a ghost is electrically indistinguishable from a real press, it cannot be
reliably filtered in software — so **avoid relying on 3+ simultaneous button
presses**.

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
├── spacemouse/                        # SpaceMouse driver-launch + dashboard package
│   ├── config/
│   │   └── spacenav_params.yaml
│   ├── launch/
│   │   ├── spacemouse.launch.py       # driver (+ dashboard if dashboard_port set)
│   │   └── dashboard.launch.py        # dashboard only
│   ├── spacemouse/
│   │   ├── __init__.py
│   │   └── dashboard_node.py
│   ├── web/
│   │   └── index.html
│   ├── resource/spacemouse
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── README.md
```

## License

MIT
