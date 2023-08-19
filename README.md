# rosplan_demos

## Installation

Before installing the simulation environment, make sure your desktop is setup with a standard installation of **ROS Melodic** 
on **Ubuntu 18.04**.

Installation instructions:
1. The demo requires that you install fetch gazebo simulator and some other ros debian pkgs, this will be done later on using rosdep, for now setup only build dependencies:

```
mkdir -p ~/rosplan_ws/src
cd ~/rosplan_ws/src
git clone -b action-interface-py https://git.oschina.net/oschina/android-app.git
git clone https://github.com/clearpathrobotics/occupancy_grid_utils
```

2. Compile the code:
```
cd ~/rosplan
catkin build
```

3. Source the workspace and install runtime dependencies:
```
source ~/rosplan_ws/devel/setup.bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-fetch-gazebo-demo
```

---

