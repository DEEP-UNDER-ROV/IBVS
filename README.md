# Stereo-enhanced Image-based Visual Servoing (IBVS) for Underwater ROV

A ROS2 implementation of **Stereo-enhanced Image-based Visual Servoing (IBVS)** for underwater ROV positioning using **stereo vision**, **AprilTag feature extraction**, and a **PI-augmented control law**.

The system estimates target pose directly from image features and generates body-frame velocity commands for closed-loop visual servoing.

---

## System Architecture

```
Stereo Camera InfraRed Left-right
       │
       ▼
AprilTag Detection Corner
       │
       ▼
Feature Extraction per each corner (u,v,Z)
       │
       ▼
Image Error Computation
       │
       ▼
Interaction Matrix (Ls)
       │
       ▼
PI-Augmented IBVS Controller
       │
       ▼
Body Velocity Command
       │
       ▼
MAVROS RC Override
       │
       ▼
ROV Motion
```


---

## Mathematical Formulation

The image feature error is defined as

```text
e = s - s_d
```

where

* (s) : current image feature vector
* (s_d) : desired image feature vector

The interaction matrix relates camera velocity to image feature velocity:

[\dot{s}=L(s,Z)\mathbf{v}]

where

* (L) : interaction matrix
* (Z) : feature depth
* (\mathbf{v}) : camera velocity

The proposed PI-augmented control law is

[
\mathbf{v}
==========

-\lambda L^{+} \left(e + K_i \int e,dt \right)]

where

* (\lambda) : proportional gain
* (K_i) : integral gain
* (L^{+}) : Moore-Penrose pseudoinverse of the interaction matrix

---

## Dependencies

* ROS2 Jazzy
* Python 3.10+
* OpenCV
* NumPy
* SciPy
* cv_bridge
* MAVROS
* AprilTag detector
* geometry_msgs
* std_msgs

---

## Installation

Clone the repository into your ROS2 workspace.

```bash
cd ~/rov_ws/src

git clone https://github.com/USERNAME/IBVS.git
```

Build the package.

```bash
cd ~/rov_ws

colcon build --symlink-install

source install/setup.bash
```

---

## Running

Launch the IBVS system:

```bash
ros2 launch ibvs ibvs.launch.py
```

---

## Published Topics

| Topic                 | Description              |
| --------------------- | ------------------------ |
| `/ibvs/error`         | Image feature error      |
| `/ibvs/velocity`      | Desired body velocity    |
| `/detector/corners`   | AprilTag corner features |
| `/stereo/depth`       | Estimated feature depth  |
| `/mavros/rc/override` | RC override commands     |

---

## Controller Parameters

| Parameter      | Description          |
| -------------- | -------------------- |
| `lambda_gain`  | Proportional gain    |
| `ki_gain`      | Integral gain        |
| `deadband`     | Error deadband       |
| `max_velocity` | Velocity saturation  |
| `control_rate` | Controller frequency |

---

## Example Results

Typical closed-loop response:

* Stable feature convergence
* Smooth velocity commands
* Reduced steady-state error
* Robust tracking under depth variation

---
