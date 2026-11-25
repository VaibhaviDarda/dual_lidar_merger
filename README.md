# dual_lidar_merger

A lightweight ROS 2 Humble package that **merges two YDLidar X2 LaserScan streams into one unified `/scan_merged` topic**.  
Both LiDARs must be mounted on the **same horizontal plane**.

---

## Table of Contents

1. [How It Works](#how-it-works)  
2. [Prerequisites](#prerequisites)  
3. [Installation](#installation)  
4. [Physical Setup](#physical-setup)  
5. [URDF / TF Setup](#urdf--tf-setup)  
6. [Configuration](#configuration)  
7. [Running the Package](#running-the-package)  
8. [Visualising the Output](#visualising-the-output)  
9. [Parameter Reference](#parameter-reference)  
10. [Overlap Strategies](#overlap-strategies)  
11. [Angle Masking](#angle-masking)  
12. [Diagnostics](#diagnostics)  
13. [Troubleshooting](#troubleshooting)  

---

## How It Works

```
/scan  (laser_frame)  ──┐
                         ├──► [dual_lidar_merger_node] ──► /scan_merged (base_link)
/scan2 (laser_frame2) ──┘
```

1. Both scans are **transformed into `base_link`** using TF2, so their ranges are expressed in the same coordinate frame regardless of where each LiDAR is physically mounted on the robot.  
2. Every point is **binned** into an angular slot of the output scan (configurable resolution, default 0.5°).  
3. When both LiDARs see the **same bin**, the configured *overlap strategy* decides the final range value (`nearest`, `farthest`, or `average`).  
4. The merged scan is published at the rate of whichever LiDAR delivers data faster (typically ~8 Hz).

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS 2 Humble | Ubuntu 22.04 recommended |
| `ydlidar_ros2_driver` | Must already be installed and working |
| `tf2`, `tf2_ros`, `tf2_geometry_msgs` | Standard Humble packages |
| `laser_geometry` | `sudo apt install ros-humble-laser-geometry` |
| `libpcl-dev` | `sudo apt install libpcl-dev` |
| `pcl_conversions` | `sudo apt install ros-humble-pcl-conversions` |

Install all dependencies at once:

```bash
sudo apt update
sudo apt install \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-laser-geometry \
  ros-humble-pcl-conversions \
  libpcl-dev
```

---

## Installation

```bash
# 1. Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src

# 2. Copy or clone this package
cp -r /path/to/dual_lidar_merger .

# 3. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select dual_lidar_merger --symlink-install

# 5. Source the workspace
source install/setup.bash
```

---

## Physical Setup

```
          ▲ Forward (robot X-axis)
          │
    ┌─────┼─────┐
    │   LiDAR 1 │  ← front, xyz: (0.0,  0.13, 0.113)  frame: laser_frame
    │     │     │
    │   base    │
    │     │     │
    │   LiDAR 2 │  ← rear,  xyz: (0.0, -0.13, 0.113)  frame: laser_frame2
    └─────────── ┘
```

- Both LiDARs are at the **same height** (z = 0.113 m from `base_link`).  
- LiDAR 2 faces **backward** → set `reversion: true` in `lidar2_params.yaml`.  
- The merger handles the coordinate transformation automatically — you only need the TF tree to be correct.

---

## URDF / TF Setup

Your URDF should already define both frames as shown below. Verify the `xyz` values match the **actual physical positions** on your robot.

```xml
<!-- LiDAR 1 — FRONT -->
<joint name="lidar_joint" type="fixed">
  <origin xyz="0.0 0.13 0.113" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="laser_frame"/>
</joint>

<!-- LiDAR 2 — REAR -->
<joint name="lidar_joint2" type="fixed">
  <origin xyz="0.0 -0.13 0.113" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="laser_frame2"/>
</joint>
```

> **Tip:** Verify your TF tree is publishing correctly before starting the merger:
> ```bash
> ros2 run tf2_tools view_frames
> # Opens frames.pdf showing the full TF tree
> ```

---

## Configuration

All parameters live in `params/merger_params.yaml`.  
You can override any parameter on the command line (see [Running the Package](#running-the-package)).

### Key parameters

| Parameter | Default | Description |
|---|---|---|
| `scan1_topic` | `/scan` | Input topic from LiDAR 1 |
| `scan2_topic` | `/scan2` | Input topic from LiDAR 2 |
| `output_topic` | `/scan_merged` | Merged output topic |
| `target_frame` | `base_link` | TF frame for the merged scan |
| `overlap_mode` | `nearest` | `nearest` / `farthest` / `average` |
| `angle_increment` | `0.00873` rad | Angular resolution (~0.5°) |
| `range_min` | `0.12` m | Discard readings closer than this |
| `range_max` | `8.0` m | Discard readings farther than this |
| `max_scan_age` | `0.5` s | Discard stale scans older than this |
| `mask_angles` | `[]` | Angle sectors to silence (degrees) |
| `invalid_range_is_inf` | `true` | Fill empty bins with `inf` or `0` |

---

## Running the Package

### Option A — Full stack (recommended for first use)

Starts both LiDAR drivers **and** the merger in one command:

```bash
ros2 launch dual_lidar_merger dual_lidar_full.launch.py
```

With overrides:

```bash
ros2 launch dual_lidar_merger dual_lidar_full.launch.py \
  lidar1_port:=/dev/ttyUSB0 \
  lidar2_port:=/dev/ttyUSB1 \
  overlap_mode:=nearest \
  rviz:=true
```

### Option B — Merger only

Use this when your LiDAR drivers are **already running** (e.g. launched by your robot's main bringup):

```bash
ros2 launch dual_lidar_merger merger_only.launch.py
```

With overrides:

```bash
ros2 launch dual_lidar_merger merger_only.launch.py \
  overlap_mode:=average \
  scan1_topic:=/lidar_front/scan \
  scan2_topic:=/lidar_rear/scan
```

### Option C — View only

Open RViz to visualise an already-running merged scan:

```bash
ros2 launch dual_lidar_merger view_merged.launch.py
```

---

## Visualising the Output

The RViz config (`rviz/merged_scan.rviz`) shows **three overlaid scans**:

| Colour | Topic | Meaning |
|---|---|---|
| 🔴 Red | `/scan` | Raw LiDAR 1 output |
| 🔵 Blue | `/scan2` | Raw LiDAR 2 output |
| 🟢 Green | `/scan_merged` | Final merged result |

The view is **top-down orthographic** centred on `base_link`.

---

## Parameter Reference

### merger_params.yaml — full list

```yaml
dual_lidar_merger:
  ros__parameters:
    # Topics
    scan1_topic:   "/scan"
    scan2_topic:   "/scan2"
    output_topic:  "/scan_merged"

    # Frames
    target_frame:  "base_link"
    scan1_frame:   "laser_frame"
    scan2_frame:   "laser_frame2"

    # Output geometry
    angle_min:        -3.14159265   # rad
    angle_max:         3.14159265   # rad
    angle_increment:   0.00872664626 # rad (~0.5°)
    range_min:         0.12         # m
    range_max:         8.0          # m

    # Overlap
    overlap_mode: "nearest"         # nearest | farthest | average

    # Staleness
    max_scan_age: 0.5               # seconds

    # Masking  (flat list of min_deg, max_deg pairs)
    mask_angles: []

    # Fill value for empty bins
    invalid_range_is_inf: true

    # Diagnostics
    publish_diagnostics: true
```

---

## Overlap Strategies

When both LiDARs report a reading in the **same angular bin**:

| Mode | Picks | Best for |
|---|---|---|
| `nearest` | Closest range value | Obstacle avoidance, navigation |
| `farthest` | Farthest range value | Mapping open spaces |
| `average` | Mean of all readings | Smoothed, noise-reduced output |

> **Recommended:** Use `nearest` for safety-critical navigation.

---

## Angle Masking

You can mute specific angular sectors in the **merged output** — useful to suppress readings from the robot body, cables, or mounting hardware.

`mask_angles` is a **flat list of degree pairs** `[min, max, min, max, ...]`:

```yaml
# Mute a single zone (e.g. a cable running across the scan at ~90°)
mask_angles: [85.0, 95.0]

# Mute two symmetric zones
mask_angles: [85.0, 95.0,  -95.0, -85.0]

# Mute the entire rear sector
mask_angles: [-180.0, -90.0,  90.0, 180.0]
```

Angles follow **ROS convention**: 0° = forward (+X), +90° = left (+Y), ±180° = rear (−X).

---

## Diagnostics

When `publish_diagnostics: true`, the node publishes to `/merger/diagnostics` every 5 seconds:

```
DualLidarMerger | scan1_rx=240  scan2_rx=239  merged=239  dropped=1
```

Monitor it live:

```bash
ros2 topic echo /merger/diagnostics
```

---

## Troubleshooting

### No output on `/scan_merged`

1. Check that both `/scan` and `/scan2` are publishing:
   ```bash
   ros2 topic hz /scan
   ros2 topic hz /scan2
   ```
2. Check TF is available:
   ```bash
   ros2 run tf2_ros tf2_echo base_link laser_frame
   ros2 run tf2_ros tf2_echo base_link laser_frame2
   ```
3. Check the node is running:
   ```bash
   ros2 node info /dual_lidar_merger
   ```

### "TF lookup failed" warnings

- Your `robot_state_publisher` (or `static_transform_publisher`) is not running.  
- The `frame_id` in your LiDAR driver params does not match the URDF joint child link name.
- Start your robot description/bringup **before** launching this package.

### Merged scan has a gap

- The two LiDARs have a physical blind spot between them. This is normal.  
- Reduce `range_min` slightly if close-range readings are being dropped.
- Check `blocked_angle_min/max` in the individual LiDAR driver params — you may be blocking too wide a range.

### Points in wrong position

- Verify the `xyz` and `rpy` in the URDF joints match physical measurements.  
- Verify `reversion` and `inverted` flags in each LiDAR's param file.  
- Use the RViz top-down view: red/blue points from each LiDAR should form one continuous ring around the robot.

### Serial port permission denied

```bash
sudo usermod -aG dialout $USER
# Log out and log back in
```

Or create a persistent udev rule (recommended):

```bash
# Find device attributes
udevadm info -a -n /dev/ttyUSB0 | grep serial

# Create rule
sudo nano /etc/udev/rules.d/99-ydlidar.rules
# Add:
# SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<serial1>", SYMLINK+="ttyYDLIDAR1"
# SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<serial2>", SYMLINK+="ttyYDLIDAR2"

sudo udevadm control --reload-rules && sudo udevadm trigger
# Then use /dev/ttyYDLIDAR1 and /dev/ttyYDLIDAR2 as port values
```
