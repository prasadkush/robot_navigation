#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <teb_local_planner/teb_local_planner_THRSL.h>
#include <teb_local_planner/TrajectoryPointMsg.h>
#include <robot_navigation/robot_config.h>
#include <robot_navigation/planner.h>
#include <robot_navigation/robot_state_msg.h>
#include <robot_navigation/control_commands.h>
#include <boost/thread/thread.hpp>
#include <time.h>
#include <Eigen/Dense>
#include <fstream>

using namespace std;

using namespace teb_local_planner;

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct chained_form 
{
	double x_1, x_2, x_3, x_4;
};

struct velocities
{
	double val1, val2;
};

struct control_info
{
	double k1, k2, k3;
	double k11, k12, k13, k14, k21, k22, k23, k24;
	double kv11, kv12, kv13, kv14, kv21, kv22, kv23, kv24;
};

/*
struct state_xytheta_vel
{
	double x;
	double y;
	double theta;
	double vel;
	double gamma;
};
*/

/*
struct traj_info
{
	string index;
	int no_of_points, total_points;
};
*/

class controller
{

    friend class planner;

public:

	controller();

	controller(int indices, ros::NodeHandle& n, bool using_replan_);

	controller(int indices, ros::NodeHandle& n, planner* planner_nav, bool using_replan_, costmap_2d::Costmap2DROS* planner_costmap_ros_);
	ros::NodeHandle nc;

    planner* planner_;

	geometry_msgs::PoseStamped pos_current;

	geometry_msgs::PoseStamped pos_desired;

	robot_state current_state, current_transformed, error_xytheta, prev_error_xytheta, error_xytheta_dot, start;

	robot_state desired_state, goal_state, dist_to_goal, pallet_goal, dist_to_pallet, goal_current;
         
        vector<robot_state> goals_vec, start_vec;

	//int current_state_index;

	vector<state_xytheta_vel> trajectory;

	vector<traj_info> traj_segments;
    
        vector<bool> goals_reached;

	velocities control_input;

	robot_navigation::control_commands feedback_vals;

	void find_closest_point(const robot_state& input_state, robot_state& closest_state);

	void find_closest_point(const robot_state& input_state);

	void find_closest_point(const robot_state& input_state, int& curr_state_ind);

	void find_closest_point_replan(const robot_state& input_state);

	void replan_for_pallet(const robot_state& curr_state_, const robot_state& pallet_goal_);

	void replan_path(const robot_state& curr_state_, const robot_state& goal_state_, int goal_index_);

	void replan_using_min_jerk(const robot_state& curr_state_, const robot_state& goal_state_, int goal_index_);

	//void compute_input();

	//void control_loop();

	void receive_path(vector<robot_state>& robot_path);

	void compute_trajectory(double turning_vel, double straight_vel, double precision_vel);

	//void compute_current_state(const robot_navigation::robot_state_msg::ConstPtr& msg);

	void stateCallback(const robot_navigation::robot_state_msg::ConstPtr& msg);

	void stateCallback_amcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	void feedbackCallback(const robot_navigation::control_commands::ConstPtr& msg);

	void palletposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	void compute_trajectory_indices();

	void output_trajectory();

	void output_replanned_trajectory();

	void output_path();

	void control_loop(const robot_navigation::robot_state_msg::ConstPtr& msg);

	void make_plan_for_pallet(const robot_state& curr_state, const robot_state& pallet_goal_, double last_segment_length);

	NavVisualizer* visual;

	void visualize();

	void visualize_replanned_path();

	vector< vector<double> > path_followed;

	double wheel_base;

	VectorXd compute_min_jerk(double x_s, double x_f, double v_s, double v_f, double a_s, double a_f, double T);

	void assign_robot_state(robot_state& temp, const VectorXd& x, const VectorXd& y, const VectorXd& theta, double i);

	void compute_path_min_jerk(const VectorXd& x, const VectorXd& y, const VectorXd& theta, const double incr, double T, vector<robot_state>& path_min_jerk);

	void receive_traj(vector<state_xytheta_vel>& robot_trajectory);

	void replan_thread();

	TebLocalPlannerTHRSL*  local_planner;

	vector<TrajectoryPointMsg> local_trajectory;

	nav_msgs::Odometry start_odom, goal_odom;

	vector<nav_msgs::Odometry> waypoints;

private:

	int current_state_index, current_state_index_replan, pallet_state_index, goal_state_index, goals_index, start_index;

	int prev_state_index, desired_state_index, desired_state_index_replan, goal_index_for_replanning;

	int indices_ahead, indices_ahead_goal, indices_ahead_in, iterations;

	bool path_received;

    bool use_replan_thread, replanned, pallet_pose_received, replan_goal_assigned, using_replan, visualize_replan, replan_done;
 
    int goal_state_index_replan;

    vector<int> start_indices, goal_indices;

	double gamma_max, vel_max, acc_max, incr;

    robot_state pallet_pose, replan_goal;

    robot_state mean_error, max_error, sum_error, closest_state_on_path, current_trans_wrt_path;

	ros::Subscriber sub, sub_feedback, sub_pallet_pose;

	chained_form current_chained, desired_chained, error_chained;

	vector<robot_state> input_path;

	vector<state_xytheta_vel> input_traj;

	control_info gains, gains_straight, gains_turn, gains_reverse, gains_reverse_1,  gains_reverse_2;

	bool desired_zero, feedback, state_feedback;

	void initialize(int indices, ros::NodeHandle& n, bool using_replan_, costmap_2d::Costmap2DROS* planner_costmap_ros_);

	double distance(robot_state state_1, robot_state state_2);

	double distance(double x1, double y1, double x2, double y2);

	double compute_slope(double y_2, double y_1, double x_2, double x_1);

	double quaternion_to_angle(geometry_msgs::Quaternion orientation);

	geometry_msgs::Quaternion angle_to_quaternion(double theta) ;

	void obtain_current_state(const robot_navigation::robot_state_msg::ConstPtr& msg);

	void obtain_current_state(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	void compute_states();

	void compute_desired_state();

	void compute_desired_state_replan();

	void assign_state_xythetavel(double x, double y, double theta, double vel, state_xytheta_vel& state);

	void change_output_coordinates();

	void compute_error_chained();

	void compute_error_xytheta();

	void check_segment_change(int i, string& current_ind, int& num_points, int& total_points);

	void assign_segment_index(string& curr_ind, string ind, int num_points, int i, int& total_points);

	inline int sign(double x, double y);

	inline int sign(double x);

	void compute_control_input();

	void compute_control_input_xytheta();

	void initialize_gain_matrix();

	void make_desired_state_zero(bool local);

	void make_closest_state_zero(bool local);

	void compute_indices_ahead(bool goal_condition_);

	void control_flow_functions(const robot_navigation::robot_state_msg::ConstPtr& msg);

	void compute_goal_pallet_condition(double& distance_to_goal_, bool& goal_condition_, bool& pallet_condition_);

	void perform_velocity_check(bool goal_condition, bool pallet_condition, double distance_to_goal);

	void perform_goal_pallet_check(double distance_to_goal, robot_navigation::control_commands& control_msg);

	void compute_replan_goal(int curr_state_ind);

	void set_start_goal_replan();

	void convert_to_robot_states();

	void compute_min_jerk(robot_state start, robot_state final);

	void initialize_indices_replan();

    void replan_using_sbpl(const robot_state& curr_state, const robot_state& goal_state_, int goal_index_);

    void replan_using_teb_local_planner();

    void assign_sbpl_plan(vector<geometry_msgs::PoseStamped>& plan_, vector < vector<double> >& replanned_path_output_,
	vector<robot_state>& replanned_path_robot_, vector<robot_state>& path_, bool pallet_up_, int pallet_state_);

   void assign_final_states(vector<robot_state>& replanned_path_robot_, vector<robot_state>& path_, int curr_state_ind);

    costmap_2d::Costmap2DROS* local_costmap_ros_;

    costmap_2d::Costmap2D* local_costmap_;

    vector < vector<double> > replanned_path_output;
	vector<robot_state> replanned_path_robot;
	vector<state_xytheta_vel> replanned_trajectory;

	double delta_time, sensor_offset;

	int no_of_points, no_of_goals, goals_count, goal_reached_count, goal_tolerance, forkstate;

    double duration;

    double str_vel, turn_vel, prec_vel;

	bool goal_reached, robot_stopped, fork_step, pallet_position, pallet_fork_step, goal_fork_step, behavior_fork;

    bool test_teb_local_planner;

    int pallet_position_st;

	geometry_msgs::PointStamped current_pos;

	ros::Time prev_time, prev_time_stopped, curr_time, time_prev_vel;
  
        ros::Duration duration_stop;

	ros::Publisher pub, pub_point;

	boost::thread* th1;
	//vector<seg_indices> seg_info;

};

#endif
