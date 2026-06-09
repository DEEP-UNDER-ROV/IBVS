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
* DU Perception AprilTag Triangulate Library
  https://github.com/DEEP-UNDER-ROV/DU_Perception_Apriltag_Triangulate
* cv_bridge
* MAVROS

---

## Installation
Install required library for GStreamer
```bash
sudo apt install -y python3-opencv \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools

```

Clone the DU Perception repository first into your ROS2 workspace, follow the guides on readme.md.
Clone IBVS repository into your ROS2 workspace;

```bash
cd ~/your_ws/src

git clone https://github.com/DEEP-UNDER-ROV/IBVS.git
```

Build the package.

```bash
cd ~/rov_ws

colcon build

source install/setup.bash
```

---

## Running
Launch the DU Perception AprilTag Triangulate
Launch the IBVS system:

```bash
ros2 launch ibvs visualize
ros2 launch ibvs ibvs
```

---

## Published Topics

| Topic                 | Description              |
| --------------------- | ------------------------ |
| `/apriltag/corners`   | AprilTag corner features |
| `/ibvs/error`         | Image feature error      |
| `/ibvs/vel_body`      | Desired body velocity    |
| `/ibvs/pwm_debug`     | IBVS PWM Control         |
| `/mavros/rc/override` | RC override commands     |

---

## Controller Parameters

| Parameter      | Description              |
| -------------- | -------------------------|
| `Kp`           | Proportional gain        |
| `Ki`           | Integral gain            |
| `mu`           | Pseudoinverse Dampening  |

---

## Example Results


---
