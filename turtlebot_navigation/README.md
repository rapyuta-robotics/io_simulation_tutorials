# Turtlebot Gazebo and Navigation

### Overview
Sample ROS packages for running turtlebot3 simulation with rapyuta.io. They demonstrate separation of your launch files
into simulation and app. simulation (with gazebo) can run on rapyuta.io, and the app (with navigation) can run on your
laptop or other cloud instance.

### Packages
- `io_gazebo_turtlebot_bringup` : Includes launch files which run simulation and app separately.
- `io_gazebo_turtlebot_description` : Includes launch file which will start gazebo. There're dependencies on turtlebot3's
    description which includes URDF, meshes, and gazebo plugin sources. These are required by both simulation and app.
- `io_gazebo_turtlebot_navigation` : Includes navigation launch file and config files.
- `io_gazebo_turtlebot_demo_app` : Includes sample app which publish sequential move_base goal.

### Build
#### Preparation
- [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

#### Build steps

**Note:** Please replace catkin_ws with your own workspace.

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/rapyuta-robotics/io_simulation_tutorials.git
sudo rosdep init
rosdep update
rosdep install --from-paths --ignore-src io_simulation_tutorials/turtlebot_navigation -y --rosdistro melodic
cd ../
source /opt/ros/melodic/setup.bash
catkin build io_gazebo_turtlebot_bringup io_gazebo_turtlebot_demo_app io_gazebo_turtlebot_description io_gazebo_turtlebot_navigation
```

### Run
#### Run on your local machine
```
cd ../
source catkin_ws/devel/setup.bash
roslaunch io_gazebo_turtlebot_bringup bringup.launch gui:=true
```

#### Run with rapyuta.io
To run it on rapyuta.io, please go through the tutorial [here](https://userdocs.rapyuta.io/build-solutions/sample-walkthroughs/separate-navigation-simulation).
