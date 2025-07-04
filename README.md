# UR5e Robot Motion Control

This repository provides motion scripts and launch files for controlling a **UR5e robotic arm** using **ROS Noetic**. It leverages the [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) library and a **Computed Torque Controller+PD Feedback**, supporting various trajectories including circular and spiral paths.

---

## Requirements

* **Operating System**: Ubuntu 20.04 LTS
* **ROS Version**: ROS 1 (Noetic)
* **Simulator**: [Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
* **Visualization**: [Rviz](http://wiki.ros.org/rviz)
* **Python**: 3.8.10

---

## Robot Description

* **Manipulator**: UR5e
* **Driver**: [Universal Robots ROS Driver – GitHub](https://github.com/ros-industrial/universal_robot.git)

---

## Installation

### 1. Install ROS and Dependencies

Make sure **ROS Noetic** and **Gazebo** are installed. Then install essential ROS packages:

```bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros ros-noetic-rviz
```

### 1-1. Create a Catkin Workspace

You can name your workspace `new_ws` (or modify it later in the launch/config files):

```bash
mkdir -p ~/new_ws/src
cd ~/new_ws
catkin_make
echo "source ~/new_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Clone the Repository

```bash
cd ~/new_ws/src
git clone https://github.com/Raha-Mir/ur5e_manipulator.git
cd ~/new_ws
```

### 4. Build the Workspace

After cloning or making changes, rebuild and source your workspace:

```bash
catkin_make
source devel/setup.bash
```

---

## Running Simulations

### 1. Launch the Main Controller in Simulation

This will start the UR5e robot in Gazebo with inverse dynamics-based control:

```bash
roslaunch ur5e_manipulator main.launch
```

#### 2. Motion Scripts

Each of the following scripts controls the UR5e robot via inverse dynamics:

* **Position reaching**:

```bash
rosrun ur5e_manipulator position_reach.py
```

* **Circular trajectory**:

```bash
rosrun ur5e_manipulator circular.py
```

* **Spiral trajectory**:

```bash
rosrun ur5e_manipulator spiral.py
```

* **Fourier-based periodic trajectory**:

```bash
rosrun ur5e_manipulator fourier_series_trajectory.py
```

### 3. Launch UR5e with `ros_control`

To launch UR5e using ros_control (effort controllers):

```bash
roslaunch simulation spawn_ur5e_eff_controller.launch
```

---
### ⚙️ We recommend that: 
Use this script if you want more stable and accurate control, especially under CPU load:

```bash
sudo -E env "PATH=$PATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PYTHONPATH=$PYTHONPATH" \
taskset -c 2 nice -n -10 rosrun ur5e_manipulator desire_script.py
```

**This script runs each 'desire_script' with elevated CPU priority and core binding to improve real-time performance. It uses `taskset` and `nice` to reduce latency and ensure smoother torque control on the UR5e robot (you can adjust it for your system without `sudo` or `taskset`).**

---

### 📊 Analysis
This folder contains Python scripts for visualizing and analyzing UR5e robot motion data:

---
Make sure your trajectory log files (e.g. `.pkl` or `.npz`) are located in `~/ur5e_logs/`.


> ℹ️ **Dependencies**:
> Make sure you have installed the following Python packages:
> `numpy`, `matplotlib`, `scipy`, `pickle`

You can install them with:

```bash
pip install numpy matplotlib scipy
```

###  Run


```bash
cd new_ws/src/ur5e_manipulator/analysis
```
###  Then Run your desire script


#####  `3d_plot_circular.py`

Visualizes the **end-effector circular trajectory** in 3D space.
Includes actual vs. desired vs. ideal circular path with color-coded time gradient.

```bash
python3 3d_plot_circular.py
```

#####  `3d_plot_spiral.py`

Plots a **spiral trajectory** followed by the end-effector in 3D.
Shows smooth trajectory evolution with projection lines and color-based time progression.

```bash
python3 3d_plot_spiral.py
```

#####  `interactive_plot_joint_positions.py`

Interactive viewer for **joint-level data** (position, velocity, acceleration).
Includes real-time sliders to adjust filtering parameters (Savitzky-Golay + Butterworth).

```bash
python3 interactive_plot_joint_positions.py
```

#####  `interactive_plot_trajectory_error.py`

Extends the joint viewer to also compare **desired vs. actual** trajectories.
Displays RMSE, overshoot, and settling time per joint. Ideal for controller performance evaluation.

```bash
python3 interactive_plot_trajectory_error.py

```

---
### 📁 Project Structure Overview

```
ur5e_manipulator/
├── Document/
│   ├── presentation file
│   └── results
├── analysis/
│   ├── 3d_plot_circular.py
│   ├── 3d_plot_spiral.py
│   ├── interactive_plot_joint_positions.py
│   └── interactive_plot_trajectory_error.py
├── config/
├── launch/
│   ├── demo.launch
│   ├── demo_gazebo.launch
│   ├── spawn_ur5e_eff_controller.launch
│   └── main.launch
├── rviz/
│   └── rviz_ur5e.launch
├── urdf/
│   ├── eef.xacro
│   ├── my_ur5e.urdf
│   └── my_ur5e.xacro
├── scripts/
│   ├── position_reach.py
│   ├── fourier_series_trajectory.py
│   ├── circular.py
│   └── spiral.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

