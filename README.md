# MuJoCo Simulation (for KROS)

## Installation and Execution
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kwlee365/kros_mujoco_simulation.git
cd ..
catkin build
source devel/setup.bash
roslaunch husky_controller simulation.launch
```

## Command

- m: Move Husky Robot
- s: Stop Husky Robot
