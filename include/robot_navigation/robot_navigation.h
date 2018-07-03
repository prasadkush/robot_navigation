#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

#include <ros/ros.h>
#include <robot_navigation/robot_config.h>
#include <robot_navigation/planner.h>
#include <robot_navigation/controller.h>
#include <robot_navigation/navviz.h>
#include <robot_navigation/robot_state_msg.h>
#include <vector>
#include <fstream>
#include <string>

using namespace std;

class robot_nav
{
public:

	robot_nav(string name, costmap_2d::Costmap2DROS* costmap_ros);

	~robot_nav();

	void initialize_visualizer(string vis_name, double robotW, double robotH, double OffX, double OffY, double OffZ);

	planner* simulate_;

	controller* trajectory_obj;
  
    NavVisualizer* visual;

    bool use_file, use_planner;

    //void visualize_(vector<double> p1_, vector<double> p2_, vector< vector<double> >& path_output_);

    void visualize_(vector< vector<double> >& path_output_);

    void read_points_from_file(const string path_name, vector<robot_state>& robot_path_, vector<state_xytheta_vel>& robot_trajectory_, 
    	  vector< vector<double> >& path_output_);

    void plan_using_file(const string file_name, vector<robot_state>& robot_path_, vector<state_xytheta_vel>& robot_trajectory_,
                                vector< vector<double> >& path_output_, robot_navigation::robot_state_msg& start_msg);

    void plan_using_planner(vector< vector<double> >& path_output_, vector<robot_state>& path_robot_, 
         vector<state_xytheta_vel>& trajectory_robot, double turining_vel, double straight_vel, double precision_vel);

    void output_path_size(vector<robot_state>& path_robot_, vector<state_xytheta_vel>& trajectory_robot_, vector< vector<double> >& path_output_);

    void initialize_controller(int indices_ahead, ros::NodeHandle& n, bool using_replan_, vector<robot_state>& path_robot_,
       vector<state_xytheta_vel>& trajectory_robot_, costmap_2d::Costmap2DROS* planner_costmap_ros_);

    void assign_robot_state(double x, double y, double theta, double gamma, bool pallet_up, int pallet_st, bool goal, robot_state& temp_state);

    void assign_robot_traj_state(double x, double y, double theta, double gamma, double vel, state_xytheta_vel& temp_state);

    void skip_spaces(ifstream& file_stream, int skip_value);
   
	ros::NodeHandle n;

	vector<double> p1, p2;

	ros::Publisher start_pub_robot_nav;
};

#endif

