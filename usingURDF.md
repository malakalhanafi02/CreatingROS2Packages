# This package provides a ROS2 for visualizing and controlling a robot arm using **URDF** and RViz. 

https://github.com/user-attachments/assets/f2e8d7c5-56f5-4f99-bb6f-97ca6809cb02

<img width="1507" alt="ros2" src="https://github.com/user-attachments/assets/a72f5fd8-6249-47ee-ae14-46a1e731b7b1">


----

## 🔢 Steps

### ⚠️ 1. Prerequisites

- Ensure you have ROS2 Foxy installed on your system, and install the necessary dependencies:

```bash
sudo apt update
sudo apt install ros-foxy-ament-cmake ros-foxy-urdf ros-foxy-rviz2 ros-foxy-robot-state-publisher ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui
```
----

### 💻 2. Create a ROS2 Workspace
  1. Source ROS 2 environment:
     ```bash
     source /opt/ros/foxy/setup.bash
     ```
  2. Create a new directory:
     ```bash
     mkdir -p ~/ros2_ws/src
     cd ~/ros2_ws/src
     ```
  3. Initialize the workspace:
     ```bash
     colcon build
     source install/setup.bash
     ```
<img width="262" alt="image" src="https://github.com/user-attachments/assets/cc87ec85-7c8b-457a-8128-75893f743634">
----
### 📦 3. Create a Robot Arm Package (using CMake as the build system)
  1. Navigate to the src directory of your workspace: 
     ```bash
     cd ~/ros2_ws/src
     ```
  2. Create a new package for your robot arm using CMake:
     ```bash
     ros2 pkg create --build-type ament_cmake robot_arm_package
     ```
<img width="651" alt="image" src="https://github.com/user-attachments/assets/196a7cf0-000c-4d9a-a4e6-e68c88fdc169">

----
### ©️ 4. Copy URDF and Mesh Files
  1. Clone the repository:
     ```bash
     cd ~/ros2_ws/src
     git clone https://github.com/smart-methods/arduino_robot_arm.git
     ```
  2. Copy the URDF and mesh files:
     ```bash
      mkdir -p ~/ros2_ws/src/robot_arm_package/urdf
      mkdir -p ~/ros2_ws/src/robot_arm_package/meshes
      cp ~/ros2_ws/src/arduino_robot_arm/robot_arm_pkg/urdf/arduino_robot_arm.urdf ~/ros2_ws/src/robot_arm_package/urdf/
      cp -r ~/ros2_ws/src/arduino_robot_arm/robot_arm_pkg/meshes/* ~/ros2_ws/src/robot_arm_package/meshes/
      ```
- 🔗 [URDF File](arduino_robot_arm.urdf): The URDF file describing the robot model.

----
### 📂 5. Creating and Updating Files:
- Update the `CMakeLists.txt` and `package.xml` files:
  - 🔗 [CMakeLists.txt](CMakeLists.txt): The CMake configuration file for this package.
  - 🔗 [Package.xml](package.xml): The package file.
- Create Launch File to visualize the robot in RViz:
```bash
cd ~/ros2_ws/src/robot_arm_package
mkdir -p launch
nano launch/display.launch.py
```
  - 🔗 [Launch File](display.launch.py): The launch file to start the robot visualization.
----
### 🏗️ 6. Build and Launch

  1. Build the workspace:
  ```bash
  cd ~/ros2_ws
  colcon build
  ```
  2. Source the setup file:
  ```bash
  source install/setup.bash
  ```
  3. Launch the URDF:
  ```bash
  ros2 launch robot_arm_package display.launch.py
  ```
<img width="1202" alt="RVIZ specs" src="https://github.com/user-attachments/assets/dadbd550-860c-417c-b6cb-4f17073b08f4">

-  *Set the "Fixed Frame"*:
    - In RViz, go to the "Global Options" section in the "Displays" panel on the left.
    - Set the "Fixed Frame" to `base`.

-  *Add a "RobotModel" Display*:
    - In the "Displays" panel, click the "Add" button at the bottom.
    - In the "By display type" tab, select "RobotModel" and click "OK".
    - Set the "Description Topic" to `/robot_description`.
  
  4. Using the Joint State Publisher GUI
  ```bash
  ros2 run joint_state_publisher_gui joint_state_publisher_gui
  ```
<img width="262" alt="gui" src="https://github.com/user-attachments/assets/6a6e3e15-962d-4572-a6d9-cf309f6141ff">

-----

## Done!! You should now see the robot arm model visualized in RViz and be able to interact with its joints using the Joint State Publisher GUI.

----





