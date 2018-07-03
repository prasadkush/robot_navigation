#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <sbpl/headers.h>
#include <nav_msgs/Path.h>
#include <robot_navigation/navviz.h>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include <robot_navigation/robot_config.h>
#include <Eigen/Dense>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

/*
struct traj_info
{
	string index;
	int no_of_points, total_points;
};
*/

class planner
{
	//friend class controller;

public:

	planner(string name, costmap_2d::Costmap2DROS* costmap_ros);

	planner(string name, costmap_2d::Costmap2DROS* costmap_ros, ros::NodeHandle& n);

	~planner();
    
	void visualize();

	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, ros::NodeHandle& n);

    double quaternion_to_angle(geometry_msgs::Quaternion orientation);

    geometry_msgs::Quaternion angle_to_quaternion(double theta);

	void specify_start_goal();

    void assign_seg_values(double start_x, double start_y, double start_theta, double final_x, double final_y, double final_theta, char seg_ind_,
          bool pallet_up_, bool goal, segment& temp);
  
   void assign_seg_values(double start_x, double start_y, double start_theta, double final_x, double final_y, double final_theta, char seg_ind_, bool pallet_up, int final_pallet_state, bool goal_, segment& temp);
    
    void make_plan_and_trajectory(vector< vector<double> >& path_output_, vector<robot_state>& path_robot_,
    vector<state_xytheta_vel>& trajectory_, double turning_vel, double straight_vel, double precision_vel);

    void assign_pose_orientation(geometry_msgs::PoseStamped& point_, double x, double y, double z, double theta);

    void compute_trajectory_indices(vector<robot_state>& path_, vector<traj_info>& traj_segments_);

    double compute_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_, vector<traj_info>& path_segments, int pallet_state, bool pallet_up_, int k,
          double turning_vel, double straight_vel, double precision_vel, double init_vel, double final_vel_des);

    double compute_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_, vector<traj_info>& path_segments, int pallet_state, bool pallet_up_,
         double turning_vel, double straight_vel, double precision_vel, double init_vel, double final_vel_des, bool pallet_segm); 

    double compute_smooth_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_robot_, bool pallet_up_, int pallet_state,
        vector< vector<double> >& path_output_, int j, double init_vel, double final_vel, double seg_vel, string index_traj); 

    double compute_smooth_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_robot_, bool pallet_up_, int pallet_state,
    vector< vector<double> >& path_output_, const robot_state& start_, const robot_state& final_, bool pallet_point, double init_vel, double init_theta_vel, double final_vel, 
    double final_theta_vel, double seg_vel, string index_traj);

    void plan_path_compute_indices(double start_x, double start_y, double start_theta, double final_x,
            double final_y, double final_theta, vector<traj_info>& input_seg);

    double compute_seg_vel_replan(string& index_traj, vector<traj_info>& prev_segments, double turning_vel, double straight_vel, double precision_vel);

    void output_trajectory(vector<state_xytheta_vel>& trajectory_);

    void output_path(vector< vector<double> >& path_for_output);

    void write_points_to_file();

    void read_waypoints_from_file(const string waypoints_file);

    void skip_spaces(ifstream& file_stream, int skip_value);

    sbpl_lattice_planner::SBPLLatticePlanner* lattice_planner_rack;

    geometry_msgs::PoseStamped start;
        
    geometry_msgs::PoseStamped goal;

    vector<geometry_msgs::PoseStamped> plan;

    //ros::NodeHandle np;

    NavVisualizer* visual;

    vector<robot_state> path_robot;

    vector< vector<double> > path_output, path_output_2;

    bool path_planned;

    bool map_received;

    double wheel_base;

    vector<seg_indices> seg_info;    // if seg_ind of seg_info is 'p', the points are from the sbpl path
	                               // if seg_ind of seg_info is 's'
    
    vector<segment> path_segs;

    vector<state_xytheta_vel> trajectory;

   vector<traj_info> traj_segments;

private:

	void GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	void assign_pose_orientation(geometry_msgs::PoseStamped& point_, double x, double y, double z, double ox, double oy, double oz, double ow);

    void assign_pose_orientation_(geometry_msgs::PoseStamped& point, double x_, double y_, double theta_);

    void compute_path_min_jerk(const VectorXd& x, const VectorXd& y, const VectorXd& theta, const double incr, double T, vector<robot_state>& path_min_jerk, double init_vel);

    void compute_path_min_jerk(const VectorXd& x, const VectorXd& y, const VectorXd& theta, const double incr, double T, vector<robot_state>& path_min_jerk);

    void assign_robot_state(robot_state& temp, const VectorXd& x, const VectorXd& y, const VectorXd& theta, double i);

    void pallet_pose_push(int j);

    VectorXd compute_min_jerk(double x_s, double x_f, double v_s, double v_f, double a_s, double a_f, double T);

	double distance(double x1, double y1, double x2, double y2);

	double distance(robot_state state_1, robot_state state_2);

	double compute_final_vel_for_smooth_traj(double& seg_vel, string& index_traj, vector<traj_info>& prev_segments, vector<traj_info>& next_segments, 
	              int j, double turning_vel, double straight_vel, double precision_vel);

    double compute_final_vel(vector<traj_info>& prev_segments, vector<traj_info>& next_segments, int j, double turning_vel,
	          double straight_vel, double precision_vel);      

    //void compute_trajectory_indices();

    double compute_vel_of_seg(double turning_vel, double straight_vel, double precision_vel, string index);

    double compute_gamma_of_seg(string index);

    void check_segment_change(vector<robot_state>& path_, vector<traj_info>& path_segment, int i, string& current_ind, int& num_points, int& total_points);

    void assign_segment_index(string& curr_ind, vector<traj_info>& path_segment, string ind, int num_points, int i, int& total_points);

    void assign_state_xythetavel(double state_x, double state_y, double state_theta, double state_vel, state_xytheta_vel& state);

    void assign_orientation(geometry_msgs::Quaternion& orient, double x_, double y_, double z_, double w_);

    void assign_all_counts_zero();

    void clearCostmapWindows(double size_x, double size_y, robot_state rack_state, geometry_msgs::Quaternion orientation);

    void assign_points_for_segment_first(vector<double>& temp_points, robot_state& temp_state, state_xytheta_vel& temp_xytheta_vel,
 vector<robot_state>& path_robot_, vector< vector<double> >& path_output_, vector<state_xytheta_vel>& trajectory_, 
        bool& pallet_up, int& pallet_state, vector<string>& next_seg_indices, int j);

    void assign_points_for_segment(vector<double>& temp_points, robot_state& temp_state, state_xytheta_vel& temp_xytheta_vel, vector<robot_state>& path_robot_,
      vector< vector<double> >& path_output_, vector<state_xytheta_vel>& trajectory_, 
        bool& pallet_up, int& pallet_state, vector<string>& next_seg_indices, int j);

    void assign_sbpl_plan_for_segment(vector<double>& temp_points, robot_state& temp_state, vector< vector<double> >& path_output_, 
               vector<robot_state>& path_robot_, vector<robot_state>& path_, int seg_index, bool pallet_up, int pallet_state);

    void assign_final_states_for_path(vector<robot_state>& path_robot_, vector<robot_state>& path_, int seg_index);

    void assign_final_states_for_path(vector<robot_state>& path_robot_, int seg_index);

    void assign_sbpl_plan(vector<geometry_msgs::PoseStamped>& temp_plan, vector<robot_state>& temp_path);

    void assign_index_of_next_seg(vector<traj_info>& next_segments, vector<string>& next_seg_indices);

    void compute_final_vel_given_nextseg(double& final_vel, vector<double>& temp_, vector<string>& next_seg_indices, int seg_ind,
      vector<traj_info>& path_segments, vector<traj_info>& next_segments, double turning_vel,
    double straight_vel, double precision_vel);

    void assign_final_states_for_trajectory(vector<state_xytheta_vel>& trajectory_, int seg_index);

    void compute_final_vel_smoothtraj_given_nextseg(bool pallet_up, int pallet_state, double& final_vel, double& seg_vel,
     vector<traj_info>& prev_segments, vector<traj_info>& next_segments, int seg_index, string& index_traj,  
     double turning_vel, double straight_vel, double precision_vel);

    void assign_path_min_jerk_points(vector<robot_state>& path_min_jerk, vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_robot_,
      vector< vector<double> >& path_output_, state_xytheta_vel& temp_xytheta_vel, bool pallet_up_, int pallet_state);

    void compute_time_for_curve(double& T, string index_traj, const robot_state& start_, const robot_state& final_, double seg_vel);

    void compute_init_vel_for_curve(bool pallet_point, string index_traj, double& init_vel);

    double compute_theta_difference(double theta1, double theta2);

    inline int sign(double x, double y);

    inline int sign(double x);

    geometry_msgs::PoseArray pallet_poses;
              
	ros::Subscriber map_sub;

	costmap_2d::Costmap2DROS* costmap_ros_;

	sbpl_lattice_planner::SBPLLatticePlanner* lattice_planner;

	double radius, robotW, robotH, offX, offY, offZ;

	double sensor_offset, curve_factor;

	int points_count_, forward_count, backward_count, fl_count, bl_count, fr_count, br_count, first_assign;

	string vis_name, text_points;

    ros::Publisher start_pub, pallet_poses_pub;
};

#endif
