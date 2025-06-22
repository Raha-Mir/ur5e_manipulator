# UR5e Robot Motion Control

This repository provides motion scripts and launch files for controlling a **UR5e robotic arm** using **ROS Noetic**. It leverages the [Pinocchio](https://stack-of-tasks.github.io/pinocchio/) library and a **Computed Torque Controller**, supporting various trajectories including circular and spiral paths.

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

### 2. Create a Catkin Workspace

You can name your workspace `new_ws` (or modify it later in the launch/config files):

```bash
mkdir -p ~/new_ws/src
cd ~/new_ws
catkin_make
echo "source ~/new_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone the Repository

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

###  Then Run

```bash
cd new_ws/src/ur5e_manipulator/analysis
```


####  `3d_plot_circular.py`

Visualizes the **end-effector circular trajectory** in 3D space.
Includes actual vs. desired vs. ideal circular path with color-coded time gradient.

```bash
python3 3d_plot_circular.py
```

####  `3d_plot_spiral.py`

Plots a **spiral trajectory** followed by the end-effector in 3D.
Shows smooth trajectory evolution with projection lines and color-based time progression.

```bash
python3 3d_plot_spiral.py
```

####  `interactive_plot_joint_positions.py`

Interactive viewer for **joint-level data** (position, velocity, acceleration).
Includes real-time sliders to adjust filtering parameters (Savitzky-Golay + Butterworth).

```bash
python3 interactive_plot_joint_positions.py
```

####  `interactive_plot_trajectory_error.py`

Extends the joint viewer to also compare **desired vs. actual** trajectories.
Displays RMSE, overshoot, and settling time per joint. Ideal for controller performance evaluation.

```bash
python3 interactive_plot_trajectory_error.py

```

---
### 📁 Project Structure Overview

```
ur5e_manipulator/
├── analysis/
│   ├── 3d_plot_circular.py
│   ├── 3d_plot_spiral.py
│   ├── interactive_plot_joint_positions.py
│   └── interactive_plot_trajectory_error.py
├── launch/
├── scripts/
├── config/
├── urdf/
├── CMakeLists.txt
├── package.xml
└── README.md
```

```
