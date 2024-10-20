# METR4202 2024 Final Demo Test Environment

## System requirements
- Ubuntu 22.04 Jammy Jellyfish
- ROS2 Humble Hawksbill
- Gazebo-11 (classic Gazebo)
- TurtleBot3 simulation package (https://github.com/ROBOTIS-GIT/turtlebot3_simulations)

## Installation

Clone the repository into your ROS2 workspace and build.

```bash
cd ~/your_ws/src
git clone https://github.com/METR4202/metr4202_aruco_explore.git
cd ~/your_ws
colcon build --symlink-install --packages-select metr4202_aruco_explore
```
## Run

Source your workspace and export TurtleBot3 WafflePi environment variable.

```bash
cd ~/your_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch metr4202_aruco_explore metr4202_2024_demo_world.launch.py
```
The following Gazebo world should appear, along with spawning a TurtleBot3 WafflePi.

![Screenshot 2024-10-20 120750](https://github.com/user-attachments/assets/c4dca81f-f43f-47e0-9dac-116258f57490)

## Note

Gazebo can be a bit temperamental. If it doesn't load the first time:
- Try rebuilding and rerunning again
- Hit `ctrl-c`, wait for the process to exit and run again
- Kill all Gazebo processes `killall -s SIGKILL gzclient gzserver` and run again
