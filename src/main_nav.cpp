#include <ros/ros.h>
#include <robot_navigation/robot_navigation.h>

using namespace std;

int main(int argc, char** argv)
{
ros::init(argc, argv, "planner");
tf::TransformListener tf_(ros::Duration(10));
costmap_2d::Costmap2DROS* planner_costmap_ros_;
planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
planner_costmap_ros_->pause();
robot_nav nav("trajectory_", planner_costmap_ros_);
//planner simulate_("trajectory_", planner_costmap_ros_);
//boost::thread th1(&planner::make_plan, &simulate_);
//boost::thread th2(&planner::visualize, &simulate_);

//boost::thread th2(&vo_planner::robot_trajectory::global_planner_thread, &simulate_);
//boost::thread th3(&vo_planner::robot_trajectory::visualize, &simulate_);
ros::spin();
return 0;
}