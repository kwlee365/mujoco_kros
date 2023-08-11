# MuJoCo Simulation (for KROS)

## Panda ver.

## Installation and Execution
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kwlee365/kros_mujoco_simulation.git
cd ..
catkin build
source devel/setup.bash
roslaunch panda_controller simulation.launch
```

## Command

- i: initial pose
- c: CLIK
