# Odometry Estimator - Skid Steering
**Authors**: Riccardo Serraino, Giuliano Livi
First Robotics Project - Politecnico di Milano

A ROS 2 package implementing wheel odometry for the **Agile-X Bunker Pro** tracked robot, along with a ground-truth error evaluation node.

---

## Package Overview

| Item | Value |
|------|-------|
| Package name | `first_project` |
| Language | C++ |
| ROS version | ROS 2 |
| Robot | Agile-X Bunker Pro (skid-steering) |

---

## Nodes

### 1. `odometer` — Odometry Node

Subscribes to the raw robot status topic and integrates wheel velocities to estimate the robot's pose over time.

**Subscriptions**

| Topic | Type |
|-------|------|
| `/bunker_status` | `bunker_msgs/msg/BunkerStatus` |

**Publications**

| Topic | Type |
|-------|------|
| `/project_odom` | `nav_msgs/msg/Odometry` |
| TF broadcast | `odom` → `base_link2` |

**Service**

| Name | Behavior |
|------|----------|
| `reset` | Resets position (x, y) and orientation (θ) to zero |

---

### 2. `tf_error` — Error Evaluation Node

Compares the ground-truth transform from the bag (`odom → base_link`) with the computed transform (`odom → base_link2`) and publishes a message.

**Publications**

| Topic | Type |
|-------|------|
| `/tf_error_msg` | `first_project/msg/TfErrorMsg` |

**Custom message — `TfErrorMsg`**

```
std_msgs/Header  header
float32          tf_error            # Euclidean distance between GT and estimated pose
int32            time_from_start     # Seconds elapsed since the bag started
float32          travelled_distance  # Total distance travelled by the robot (metres)
```

---

## Skid-Steering Odometry

The Bunker Pro is a **skid-steering tracked robot**: it has no steerable wheels, so turning is achieved by commanding different velocities to the left and right tracks.

From the motor RPM readings the node derives:

- **v** — linear velocity of the robot centre (average of left/right track speeds)  
- **ω** — angular velocity (difference of left/right track speeds divided by the track separation)

These are then integrated over the time step `dt` to update the pose `(x, y, θ)`.

### Integration Methods

Because a single fixed formula can become numerically unstable depending on how fast the robot is turning, the node uses an **adaptive strategy** that picks the most appropriate integrator at each step.

---

#### Exact Integration *(closed-form)*

Used when **|ω| > threshold** — i.e. the robot is clearly turning.  
This is the most accurate formula for curved motion.

```
Δx = (v / ω) · [ sin(θ_k + ω·dt) − sin(θ_k) ]
Δy = (v / ω) · [ cos(θ_k) − cos(θ_k + ω·dt) ]
Δθ = ω · dt
```

The division by ω gives a precise arc-length computation, but becomes problematic when ω is near zero (division by a tiny number amplifies floating-point noise) — hence the fallback below.

---

#### Runge-Kutta 2nd Order *(Midpoint method)*

Used when **|ω| ≤ threshold** — i.e. the robot is moving nearly straight.  
Avoids the numerical singularity of the exact method when ω ≈ 0.

```
θ_mid = θ_k + (ω · dt) / 2       ← evaluate heading at mid-step

Δx   = v · cos(θ_mid) · dt
Δy   = v · sin(θ_mid) · dt
Δθ   = ω · dt
```

Using the midpoint heading instead of the initial heading gives second-order accuracy, which is more than sufficient for near-straight trajectories.

---

#### Adaptive Selection

```
if |ω| > ω_threshold:
    use Exact Integration      ← accurate on curves
else:
    use Runge-Kutta 2          ← stable on straight segments
```

The threshold is tuned so that the transition between methods is smooth and no discontinuities are introduced in the pose estimate.

---

## Usage

### Prerequisites

- ROS 2 installed and sourced  
- `bunker_msgs` package available in your workspace  
- The `first_project` package built:

```bash
cd ~/colcon_ws
colcon build
source install/setup.bash
```

### Running the project

**Select the bag file**

Open the launch file and set the bag path:

```python
cd first_project/launch/first_project.launch.py

# default_value='../bags/rosbag2_<number>',   # <-- change this line
```

**Launch the project**

```bash
ros2 launch first_project first_project.launch.py
```

This single command starts both nodes (`odometer` and `tf_error`) and opens RViz with the pre-configured top-view showing both the ground-truth (`base_link`) and estimated (`base_link2`) transforms with their labels.


**Resetting odometry**

While the project is running:

```bash
ros2 service call /reset_odom first_project/srv/Reset "{reset: true}"
```

---

## Notes
The project has been developed with container provided by the course of Robotics, find it at the following link:
[Robotics Course](https://github.com/AIRLab-POLIMI/robotics-ros2.git)
