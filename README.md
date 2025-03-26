for environment launch

cd catkin_ws/
catkin_build
source devel/setup.bash
roslaunch iiwa_gazebo iiwa_gazebo.launch

for planner launch(in seperate terminal)

cd catkin_ws/
catkin_build
source devel/setup.bash
roslaunch iiwa_gazebo planner.launch
