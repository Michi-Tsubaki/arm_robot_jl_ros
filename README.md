# ArmRobot.jl (Julia module)
# arm_robot_jl_ros (ROS1 package)

## Setup
```bash
cd <path_to_catkin_ws>
source devel/setup.bash
cd src
git clone ...
cd arm_robot_jl_ros
rosdep update --include-eol-distro
rosdep install --from-path . -i -r -y
cd <path_to_catkin_ws>
source devel/setup.bash
catkin build
```