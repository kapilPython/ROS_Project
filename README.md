# ROS_Project

The multi-goal driver has been tried with mpo_700 neo_simulation

git clone the package using

cd ~/catkin_ws/src

git clone https://github.com/kapilPython/ROS_Project.git

cd ..

catkin_make

The package has a dependency on standard actionlib package so install it using :

git clone https://github.com/ros/actionlib.git

For making the robot move as per your yaml coordinates run

rosrun my_pkg test_node

For making the points visible on rviz just run 

rosrun my_pkg waypoint_publisher


After running this in Rviz add the topic /waypoint PoseArray.
