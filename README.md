# 🐢 TurtleBot3 Open-Loop Control (ROS2 Galactic)

This ROS2 package demonstrates **open-loop velocity control** of a TurtleBot3 in the Gazebo simulation environment using two approaches:

1. **Constant Velocity Control**
2. **Trapezoidal Velocity Profile Control**

Both nodes command the robot to move forward for a fixed distance, record odometry, and plot position vs. time using `matplotlib`.

---

## 📁 Package Overview

This package contains two main scripts.

| Script | Description |
|--------|-------------|
| `tb_openLoop.py` | Controls the robot using a trapezoidal velocity profile with acceleration, cruise, and deceleration phases. |
| `tb_openLoop_noAccel.py` | Drives the robot at a constant velocity over a fixed duration. |

---
