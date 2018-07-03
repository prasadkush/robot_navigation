Process to make the package:

1) catkin_make sbpl package

2) catkin_make sbpl_lattice_planner package

In sbpl_lattice_planner/src/sbpl_lattice_planner.cpp

change line 101 which declares the path of input motion primitive file.

3) catkin_make robot_navigation

4) catkin_make simulator

The latest working src files being used are controller_new.cpp, planner_new.cpp and robot_navigation.cpp

Process to run the package:

1) rosrun simulator sim_node

2) roslaunch robot_navigation robot_navigation.launch
