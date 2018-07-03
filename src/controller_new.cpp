#include <ros/ros.h>
#include <robot_navigation/controller.h>
#include <math.h>
#include <time.h>

using namespace std;

double Velocity_Max = 0.6;

controller::controller(int indices, ros::NodeHandle& n, planner* planner_nav, bool using_replan_, costmap_2d::Costmap2DROS* planner_costmap_ros_)
{
	cout<<"inside controller constructor\n";
	planner_ = planner_nav;
	initialize(indices, n, using_replan_, planner_costmap_ros_);
}

controller::controller()
{
	path_received = 0;
	//goal_reached = 0;
	current_state_index = 30;
	desired_state_index = 0;
	indices_ahead = 0;
	delta_time = 0.1;
	gamma_max = 70.0*PI/180;
	initialize_gain_matrix();
}

//This function is a callback function. It receives localization feedback from the topic /robot_state (from autonav)
//control loop is called inside this function
void controller::stateCallback(const robot_navigation::robot_state_msg::ConstPtr& msg)
{
	obtain_current_state(msg);
	//cout<<"inside stateCallback\n";
	if (path_received == 1 && feedback == 1 && goal_reached == 0)
	{
        //cout<<"forkstate: "<<forkstate<<" pallet position: "<<pallet_position_st<<"\n";
        if (forkstate == pallet_position_st)
        {
		   control_loop(msg);
        }
		state_feedback = 1;
	}
}

void controller::stateCallback_amcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	if (path_received == 1 && feedback == 1)
	{
		//cout<<"\nrobot_state x: "<<msg->x<<" y: "<<msg->y<<" z: "<<msg->z<<"\n";
		//control_loop(msg);
	}
}

void controller::feedbackCallback(const robot_navigation::control_commands::ConstPtr& msg)
{
	feedback_vals.val1 = msg->val1;
	feedback_vals.val2 = msg->val2;
	current_state.gamma = msg->val2;
        feedback_vals.val3 = msg->val3;
	forkstate = feedback_vals.val3;
        //cout<<"\nfeedback val1 : "<<feedback_vals.val1<<" val2: "<<feedback_vals.val2<<"\n";
	feedback = 1;
}

void controller::initialize(int indices, ros::NodeHandle& n, bool using_replan_, costmap_2d::Costmap2DROS* planner_costmap_ros_)
{
	double robotW, robotH, offX, offY, offZ;
	string vis_name;
	feedback = 0;
	goal_reached = 0;  
	state_feedback = 0;
	path_received = 0;
	robot_stopped = 0;
	fork_step = 0;
	pallet_fork_step = 0;
	goal_fork_step = 0;
	behavior_fork = 0;
	current_state_index = 0;
	desired_state_index = 0;
	indices_ahead = indices;
	indices_ahead_goal = indices;
	indices_ahead_in = indices;
	goal_index_for_replanning = 0;
	local_costmap_ros_ = planner_costmap_ros_;
	local_costmap_ = local_costmap_ros_->getCostmap();
	//nc = n;
	gamma_max = 70.0*PI/180.0;
	feedback_vals.val1 = 0.0;
	feedback_vals.val2 = 0.0;
	vis_name = "robot_state_points"; 
	replan_done = 0;
	using_replan = using_replan_;
	use_replan_thread = 0;
	replanned = 0;
	replan_goal_assigned = 0;
	visualize_replan = 1;
	test_teb_local_planner = 0;
	str_vel = 0.5;
	turn_vel = 0.3;
	prec_vel = 0.1;
	incr = 0.05;
        robotW = 1.0;
	robotH = 1.0;
	offX = 0.0;
	offY = 0.0;
	offZ = 0.0;
	sensor_offset = 1.1;
        forkstate = 0;
	pallet_position;
        pallet_position_st = 0;
        iterations = 0;
        mean_error.x = 0.0;
        mean_error.y = 0.0;
        mean_error.theta = 0.0;
        max_error.x = 0.0;
        max_error.y = 0.0;
        max_error.theta = 0.0;
        visual = new NavVisualizer(vis_name, robotW, robotH, offX, offY, offZ);
	initialize_gain_matrix();
	//ros::Subscriber sub = nc.subscribe("/state", 1000, &controller::stateCallback, this);
	// callback for localization feedback
	sub = nc.subscribe("robot_state", 1, &controller::stateCallback, this);   //
	// callback for actuator feedback
	sub_feedback = nc.subscribe("feedback_speed_steer", 1, &controller::feedbackCallback, this);
	sub_pallet_pose = nc.subscribe("/pallet_pose", 1, &controller::palletposeCallback, this);
	pub = nc.advertise<robot_navigation::control_commands>("control_input", 1);
	pub_point = nc.advertise<geometry_msgs::PointStamped>("robot_point", 1);
	if (use_replan_thread)
    {   tf::TransformListener tf(ros::Duration(10)); 
    	local_planner = new TebLocalPlannerTHRSL;
		local_planner->initialize("teb_local_planner", &tf,  planner_costmap_ros_);
    	th1 = new boost::thread(&controller::replan_thread, this); }
}

void controller::palletposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	pallet_pose.x = msg->pose.pose.position.x;
	pallet_pose.y = msg->pose.pose.position.y;
	pallet_pose.z = msg->pose.pose.position.z;
	pallet_pose.theta = quaternion_to_angle(msg->pose.pose.orientation);
	pallet_pose_received = 1;
}

double controller::quaternion_to_angle(geometry_msgs::Quaternion orientation)
{
	double theta;
	theta = 2 * atan2(orientation.z, orientation.w);
	return theta;
}

geometry_msgs::Quaternion controller::angle_to_quaternion(double theta)    // yaw angle to quaternion conversion
{
	tf::Quaternion temp;
	temp.setEulerZYX(theta,0,0);
	geometry_msgs::Quaternion orientation; 
	orientation.x = temp.getX();
	orientation.y = temp.getY();
	orientation.z = temp.getZ();
	orientation.w = temp.getW();
	return orientation;
}

double controller::distance(robot_state state_1, robot_state state_2)
{
	return sqrt(pow((state_1.x - state_2.x),2) + pow((state_1.y - state_2.y),2));
}
//vector<seg_indices> seg_info_;
double controller::distance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
}


//Receives path of the robot (after being computed by the planner)
//Also stores goal poses, goal indices, start indices and initializes other indices
void controller::receive_path(vector<robot_state>& robot_path)
{
	bool pallet_, start_next = 0, temp = 0, stop_robot = 0;
	int  start_count = 0, prev_pallet = 0;
	goals_count = 0;
	pallet_ = 0;
	input_path.resize(robot_path.size());
	if (robot_path.size() > 0)
	{
		start_vec.push_back(robot_path[0]);
		start_indices.push_back(0);
	}
	start_count = start_count + 1;
	for (int i = 0; i < robot_path.size(); i++)
	{
		if (start_next == 1)
		{
			start_vec.push_back(robot_path[i]);
			start_indices.push_back(i);
			start_next = 0;
		}
		input_path[i].x = robot_path[i].x;
		input_path[i].y = robot_path[i].y;
		input_path[i].theta = robot_path[i].theta;
		//input_path[i].gamma = 0.0;
		input_path[i].gamma = robot_path[i].gamma;
		input_path[i].pallet_up = robot_path[i].pallet_up;
                input_path[i].pallet_state = robot_path[i].pallet_state;
		input_path[i].goal = robot_path[i].goal;
		//if (input_path[i].pallet_up != prev_pallet && stop_robot == 0 && i != robot_path.size()-1)
		if ((input_path[i].goal == 1 && i != robot_path.size() -1) || 
				(input_path[i].pallet_state != prev_pallet && i != robot_path.size() - 1))
		{
			goals_vec.push_back(input_path[i]);
			goals_reached.push_back(temp);
			goal_indices.push_back(i);
			goals_count = goals_count + 1;
			start_next = 1;
			stop_robot = 1;
			pallet_ = 1;                      
			prev_pallet = input_path[i].pallet_state;  
			//cout<<"inside goal condition\n";
			//cout<<"goal index: "<<i<<"\n";
		}                
	}
	path_received = 1;
	no_of_points = input_path.size();
	goal_state.x = input_path[no_of_points - 1].x;
	goal_state.y = input_path[no_of_points - 1].y;
	goal_state.theta = input_path[no_of_points - 1].theta;
	goals_vec.push_back(input_path[no_of_points - 1]);
	goal_current = goals_vec[0];
	goals_reached.push_back(temp);
	goal_indices.push_back(no_of_points-1);
	goals_count = goals_count+1;
	goals_index = 0;
	start_index = 0;
	start.x = input_path[0].x;
	start.y = input_path[0].y;
	start.theta = input_path[0].theta;
	cout<<"goals_vec size: "<<goals_vec.size()<<"\n";
	for (int j = 0; j < goals_vec.size(); j++)
	{
		cout<<"goals vec "<<j+1<<". x: "<<goals_vec[j].x<<" y: "<<goals_vec[j].y<<"\n";
		cout<<"goal_indices "<<j+1<<": "<<goal_indices[j]<<"\n";
	}
	//cout<<"goals_vec 1. x: "<<goals_vec[1].x<<" y: "<<goals_vec[1].y<<"\n";
	//goal_state_index = input_path.size() - 1;
	goal_state_index = goal_indices[goals_index];
	if (abs(input_path[no_of_points - 1].vel - input_path[no_of_points-2].vel) <= 0.04)
	{
		goal_tolerance = 2;
	} 
	else
	{
		goal_tolerance = 25;
	}
	cout<<"goal tolenrace: "<<goal_tolerance<<"\n";
	pallet_position = 0;
	goal_reached_count = 0;
}

// function for replanning
void controller::make_plan_for_pallet(const robot_state& curr_state, const robot_state& pallet_goal_, double last_segment_length)
{
	vector < vector<double> > replanned_path_output_;
	vector<robot_state> replanned_path_robot_;
	vector<state_xytheta_vel> replanned_trajectory_;
	robot_state inter_pallet_point;
	int curr_state_ind;
	inter_pallet_point.x = pallet_goal_.x + last_segment_length*cos(pallet_goal_.theta);
	inter_pallet_point.y = pallet_goal_.y + last_segment_length*sin(pallet_goal_.theta);
	inter_pallet_point.theta = pallet_goal_.theta;
    find_closest_point(curr_state, curr_state_ind); 
	streambuf *psbuf, *backup;
	ofstream filestr;
	filestr.open("test_planner.txt");
    state_xytheta_vel temp_xytheta_vel;
	bool plan_path, pallet_up_, pallet_ = 0, goal = 0;
    int pallet_state_ = 0;
    pallet_up_ = input_path[curr_state_ind].pallet_up;
    pallet_state_ = input_path[curr_state_ind].pallet_state;
	double theta, init_theta_vel, final_theta_vel;
	int i, no_of_plans = 1, j;
	double init_vel, seg_vel, final_vel;
	vector<geometry_msgs::PoseStamped> plan_;
	vector<double> temp_points(3,0.0);
	vector<double> temp_(2,0.0);
	vector< vector<double> > init_final_vels;
	vector<string> next_seg_indices;
	robot_state temp_state;
	vector<robot_state> temp_path, path_;
	vector<traj_info> path_segments;

	geometry_msgs::PoseStamped temp;
    vector<geometry_msgs::PoseStamped> points_(2, temp);
    planner_->assign_pose_orientation(points_[0], curr_state.x, curr_state.y, 0.0, curr_state.theta);
    planner_->assign_pose_orientation(points_[1], inter_pallet_point.x, inter_pallet_point.y, 0.0, inter_pallet_point.theta);
	//backup = cout.rdbuf();
	//psbuf = filestr.rdbuf();
	//cout.rdbuf(psbuf);
	//plan_path = lattice_planner->makePlan(points_[0], points_[1], plan);
    plan_path = planner_->lattice_planner_rack->makePlan(points_[0], points_[1], plan_);
    //cout.rdbuf(backup);
    cout<<"\nplan_path: "<<plan_path<<"  plan size: "<<plan_.size()<<"\n";
	//cout<<"plan "<<j+1<<": \n";
	for (i = 0; i < plan_.size(); i++)
	{
		temp_points[0] = plan_[i].pose.position.x;
		temp_points[1] = plan_[i].pose.position.y;
		temp_points[2] = plan_[i].pose.position.z;
		theta = 2 * atan2(plan_[i].pose.orientation.z, plan_[i].pose.orientation.w);
		//cout<<" theta: "<<theta<<" degrees: "<<theta*180/PI<<"\n";
		temp_state.x = plan_[i].pose.position.x;
		temp_state.y = plan_[i].pose.position.y;
		temp_state.theta = theta;
		temp_state.gamma = 0.0;
		temp_state.pallet_up = pallet_up_;
        temp_state.pallet_state = pallet_state_;
		temp_state.goal = 0;
		temp_points[2] = theta;
		//cout<<" theta: "<<theta<<"\n";
		replanned_path_output_.push_back(temp_points);
		replanned_path_robot_.push_back(temp_state);
		path_.push_back(temp_state);
	}
	replanned_path_robot_[replanned_path_robot_.size() - 1].pallet_up = input_path[curr_state_ind].pallet_up;
	path_[path_.size() - 1].pallet_up = input_path[curr_state_ind].pallet_up;
    replanned_path_robot_[replanned_path_robot_.size() - 1].pallet_state = input_path[curr_state_ind].pallet_up;
    path_[path_.size() - 1].pallet_state = input_path[curr_state_ind].pallet_state;
	replanned_path_robot_[replanned_path_robot_.size() - 1].goal = 0;
	path_[path_.size() - 1].goal = 0;
	//pallet_up = path_segs[j].final_pallet_up;
	plan_.clear();
	planner_->compute_trajectory_indices(path_, path_segments);
	cout<<"inside smooth trajectory generation\n";
    seg_vel = 0.1;
    final_vel = 0.0;
    init_theta_vel = 0.0;
    final_theta_vel = 0.0;
    string index_traj = "b";
    init_vel = planner_->compute_trajectory(replanned_trajectory_, path_, path_segments, pallet_state_, pallet_up_, turn_vel, str_vel, prec_vel, init_vel, final_vel, 0);
    replanned_trajectory_[replanned_trajectory_.size()-1].pallet_up = pallet_up_;
    replanned_trajectory_[replanned_trajectory_.size()-1].pallet_state = pallet_state_;
    cout<<"replanned path output size: "<<replanned_path_output_.size()<<"\n";                   
	//init_vel = planner_->compute_smooth_trajectory(replanned_trajectory_, replanned_path_robot_, pallet_up_, pallet_state_, replanned_path_output_, j, init_vel, final_vel, seg_vel, index_traj);
	init_vel = planner_->compute_smooth_trajectory(replanned_trajectory_, replanned_path_robot_, pallet_up_, pallet_state_,
    replanned_path_output_, inter_pallet_point, pallet_goal_, 1, init_vel, init_theta_vel, final_vel, final_theta_vel, seg_vel, index_traj);

	cout<<"after compute smooth trajectory\n";
	cout<<"replanned path output size: "<<replanned_path_output_.size()<<"\n";     
	replanned_trajectory_[replanned_trajectory_.size()-1].pallet_up = goal_current.pallet_up;
	replanned_path_robot_[replanned_path_robot_.size()-1].pallet_up = goal_current.pallet_up;
    replanned_trajectory_[replanned_trajectory_.size()-1].pallet_state = goal_current.pallet_state;
    replanned_path_robot_[replanned_path_robot_.size()-1].pallet_state = goal_current.pallet_state;
	replanned_path_robot_[replanned_path_robot_.size()-1].goal = 1;
	pallet_up_ = goal_current.pallet_up;
    pallet_state_ = goal_current.pallet_state;
    replanned_path_robot = replanned_path_robot_;
    replanned_path_output = replanned_path_output_;
    replanned_trajectory = replanned_trajectory_;
    //goal_state_index_replan = replanned_path_robot.size();
    //current_state_index = 0;
    replanned = 1;
    //cout<<"\n\nreplanned path _ size: "<<replanned_path_output_.size()<<"\n\n";
}

// used in replanning
void controller::assign_sbpl_plan(vector<geometry_msgs::PoseStamped>& plan_, vector < vector<double> >& replanned_path_output_,
	vector<robot_state>& replanned_path_robot_, vector<robot_state>& path_, bool pallet_up_, int pallet_state_)
{
	double theta;
	vector<double> temp_points(3,0.0);
	robot_state temp_state;
	for (int i = 0; i < plan_.size(); i++)
	{
		temp_points[0] = plan_[i].pose.position.x;
		temp_points[1] = plan_[i].pose.position.y;
		temp_points[2] = plan_[i].pose.position.z;
		theta = 2 * atan2(plan_[i].pose.orientation.z, plan_[i].pose.orientation.w);
		//cout<<" theta: "<<theta<<" degrees: "<<theta*180/PI<<"\n";
		temp_state.x = plan_[i].pose.position.x;
		temp_state.y = plan_[i].pose.position.y;
		temp_state.theta = theta;
		temp_state.gamma = 0.0;
		temp_state.pallet_up = pallet_up_;
        temp_state.pallet_state = pallet_state_;
		temp_state.goal = 0;
		temp_points[2] = theta;
		//cout<<" theta: "<<theta<<"\n";
		replanned_path_output_.push_back(temp_points);
		replanned_path_robot_.push_back(temp_state);
		path_.push_back(temp_state);
	}
}

void controller::assign_final_states(vector<robot_state>& replanned_path_robot_, vector<robot_state>& path_, int curr_state_ind)
{
	replanned_path_robot_[replanned_path_robot_.size() - 1].pallet_up = input_path[curr_state_ind].pallet_up;
	path_[path_.size() - 1].pallet_up = input_path[curr_state_ind].pallet_up;
    replanned_path_robot_[replanned_path_robot_.size() - 1].pallet_state = input_path[curr_state_ind].pallet_up;
    path_[path_.size() - 1].pallet_state = input_path[curr_state_ind].pallet_state;
	replanned_path_robot_[replanned_path_robot_.size() - 1].goal = 0;
	path_[path_.size() - 1].goal = 0;
}

void controller::replan_using_sbpl(const robot_state& curr_state, const robot_state& goal_state_, int goal_index_)
{
	vector < vector<double> > replanned_path_output_;
	vector<robot_state> replanned_path_robot_;
	vector<state_xytheta_vel> replanned_trajectory_;
	int pallet_state_ = 0, i, j, curr_state_ind;
	double theta, init_vel, final_vel;
	bool plan_path, pallet_up_, pallet_ = 0, goal = 0;

    find_closest_point(curr_state, curr_state_ind); 
	streambuf *psbuf, *backup;
	ofstream filestr;
	filestr.open("test_planner.txt");
	
    pallet_up_ = input_path[curr_state_ind].pallet_up;
    pallet_state_ = input_path[curr_state_ind].pallet_state;
	
	vector<geometry_msgs::PoseStamped> plan_;
	vector<double> temp_(2,0.0);
	vector< vector<double> > init_final_vels;
	vector<robot_state> temp_path, path_;
	vector<traj_info> path_segments;

	geometry_msgs::PoseStamped temp;
    vector<geometry_msgs::PoseStamped> points_(2, temp);
    planner_->assign_pose_orientation(points_[0], curr_state.x, curr_state.y, 0.0, curr_state.theta);
    planner_->assign_pose_orientation(points_[1], goal_state_.x, goal_state_.y, 0.0, goal_state_.theta);
	backup = cout.rdbuf();
	psbuf = filestr.rdbuf();
	cout.rdbuf(psbuf);
	//plan_path = lattice_planner->makePlan(points_[0], points_[1], plan);
    plan_path = planner_->lattice_planner_rack->makePlan(points_[0], points_[1], plan_);
    cout.rdbuf(backup);
    cout<<"\nplan_path: "<<plan_path<<"  plan size: "<<plan_.size()<<"\n";
	//cout<<"plan "<<j+1<<": \n";

    assign_sbpl_plan(plan_, replanned_path_output_, replanned_path_robot_, path_, pallet_up_, pallet_state_);

    assign_final_states(replanned_path_robot_, path_, curr_state_ind);

	//pallet_up = path_segs[j].final_pallet_up;
	plan_.clear();
	planner_->compute_trajectory_indices(path_, path_segments);
    final_vel = trajectory[goal_index_].vel;
    init_vel = planner_->compute_trajectory(replanned_trajectory_, path_, path_segments, pallet_state_, pallet_up_, turn_vel, str_vel, prec_vel, init_vel, final_vel, 0);
    replanned_trajectory_[replanned_trajectory_.size()-1].pallet_up = pallet_up_;
    replanned_trajectory_[replanned_trajectory_.size()-1].pallet_state = pallet_state_;
    replanned_path_robot.clear();
    replanned_path_robot = replanned_path_robot_;
    replanned_path_output.clear();
    replanned_path_output = replanned_path_output_;
    replanned_trajectory.clear();
    replanned_trajectory = replanned_trajectory_;

}

void controller::replan_for_pallet(const robot_state& curr_state_, const robot_state& pallet_goal_)
{
   make_plan_for_pallet(curr_state_, pallet_goal_, 2.0);   
}

void controller::replan_path(const robot_state& curr_state_, const robot_state& goal_state_, int goal_index_)
{

    cout<<"INSIDE replan_path\n";
	//replan_for_pallet(curr_state_, goal_state_);
	//replan_using_min_jerk(curr_state_, goal_state_, goal_index_);
	//replan_using_sbpl(curr_state_, goal_state_, goal_index_);
	replan_using_teb_local_planner();
	initialize_indices_replan();
	replan_done = 1;
	replanned = 1;
	visualize_replan = 1;
}

/*
void controller::replan_using_sbpl(const robot_state& curr_state, const robot_state& goal_state, int goal_index_)
{
	
}
*/

void controller::initialize_indices_replan()
{
	current_state_index_replan = 2;
	desired_state_index_replan = 3;
	find_closest_point_replan(current_state);
	goal_state_index_replan = replanned_path_robot.size() - 1;
}

// for replanning using min_jerk
void controller::replan_using_min_jerk(const robot_state& curr_state_, const robot_state& goal_state_, int goal_index_)
{
	vector<traj_info> path_segments, next_segments;
	vector < vector<double> > replanned_path_output_;
	vector<robot_state> replanned_path_robot_;
	vector<state_xytheta_vel> replanned_trajectory_;
	double init_vel, final_vel, init_theta_vel, final_theta_vel, seg_vel;
	int curr_state_ind;
	bool pallet_up_, pallet_state_, goal_ = false;
	string index_traj;
	streambuf *psbuf, *backup;
	ofstream filestr;
	filestr.open("test_planner.txt");
    //backup = cout.rdbuf();
	//spsbuf = filestr.rdbuf();
	//cout.rdbuf(psbuf);
	planner_->plan_path_compute_indices(curr_state_.x, curr_state_.y, curr_state_.theta, goal_state_.x, goal_state_.y, goal_state_.theta,
		                          path_segments);
	//cout.rdbuf(backup);
	seg_vel = planner_->compute_seg_vel_replan(index_traj, path_segments, turn_vel, str_vel, prec_vel);
	final_vel = trajectory[goal_index_].vel;
	final_theta_vel = (trajectory[goal_index_].theta - trajectory[goal_index_ - 1].theta)/incr;
	find_closest_point(curr_state_, curr_state_ind); 
    pallet_up_ = input_path[curr_state_ind].pallet_up;
    pallet_state_ = input_path[curr_state_ind].pallet_state;
    init_vel  = trajectory[curr_state_ind].vel;
    cout<<"inside replan_using_min_jerk, init_vel: "<<init_vel<<"\n";
    init_theta_vel = (trajectory[curr_state_ind].theta - trajectory[max(0,curr_state_ind - 1)].theta)/incr;
	init_vel = planner_->compute_smooth_trajectory(replanned_trajectory_, replanned_path_robot_, pallet_up_, pallet_state_,
    replanned_path_output_, curr_state_, goal_state_, goal_, init_vel, init_theta_vel, final_vel, final_theta_vel, seg_vel, index_traj);
    replanned_path_robot.clear();
    replanned_path_robot = replanned_path_robot_;
    replanned_path_output.clear();
    replanned_path_output = replanned_path_output_;
    replanned_trajectory.clear();
    replanned_trajectory = replanned_trajectory_;
}


//replan_thread, shall run concurrently if we are using replanning method
void controller::replan_thread()
{
	while (1)
	{
		ros::Duration(0.4).sleep();
	    if (using_replan == 1 && replan_goal_assigned == 1)
	    {
		    //replan_for_pallet(current_state, replan_goal);
		    cout<<"inside replan thread\n";
		    replan_path(current_state, replan_goal, goal_index_for_replanning);
        }
    }
}

//This function receives trajectory (after being computed from the planner)
void controller::receive_traj(vector<state_xytheta_vel>& robot_trajectory)
{	
	trajectory.resize(robot_trajectory.size());
	for (int i = 0; i < robot_trajectory.size(); i++)
	{
		trajectory[i].x = robot_trajectory[i].x;
		trajectory[i].y = robot_trajectory[i].y;
		trajectory[i].theta = robot_trajectory[i].theta;
		trajectory[i].gamma = robot_trajectory[i].gamma;
		trajectory[i].vel = robot_trajectory[i].vel;
	}	
	start.x = trajectory[0].x;
	start.y = trajectory[0].y;
	start.theta = trajectory[0].theta;
}

void controller::find_closest_point(const robot_state& input_state, int& curr_state_ind)
{
	int lower_index, upper_index, i;
	double dist, closest_dist = 1000.0;
	int current_state_index_;
	current_state_index_ = current_state_index;
	lower_index = max(max(current_state_index - 5, start_indices[min(start_index, int(start_indices.size())-1)]), 0);   // changing this line for fork movement control
	if (current_state_index < goal_state_index)
	{
		upper_index = min(current_state_index_ + 10, goal_state_index);
	}
	else
	{
		upper_index = min(current_state_index_ + 10, goal_indices[min(goals_index+1, int(goal_indices.size())-1)]);
	}
	for (i = lower_index; i <= upper_index; i++)
	{
		if (i >= 0 && i < input_path.size())
		{
			dist = distance(input_path[i], input_state);
			if (dist <= closest_dist && i >= current_state_index_)
			{
				closest_dist = dist;
				current_state_index_ = min(i,goal_indices[min(goals_index,int(goal_indices.size())-1)]);
				current_state_index = current_state_index_;
			}
		}
	}

	curr_state_ind = current_state_index_;
}

void controller::find_closest_point(const robot_state& input_state)
{
	int lower_index, upper_index, i;
	double dist, closest_dist = 1000.0;
	lower_index = max(max(current_state_index - 5, start_indices[min(start_index, int(start_indices.size())-1)]), 0);   // changing this line for fork movement control
	if (current_state_index < goal_state_index)
	{
		upper_index = min(current_state_index + 10, goal_state_index);
	}
	else
	{
		upper_index = min(current_state_index + 10, goal_indices[min(goals_index+1, int(goal_indices.size())-1)]);
	}
	for (i = lower_index; i <= upper_index; i++)
	{
		if (i >= 0 && i < input_path.size())
		{
			dist = distance(input_path[i], input_state);
			if (dist <= closest_dist && i >= current_state_index)
			{
				closest_dist = dist;
				current_state_index = min(i,goal_indices[min(goals_index,int(goal_indices.size())-1)]);
				if (current_state_index < goal_state_index)
				{
					desired_state_index = max(desired_state_index, min(current_state_index + indices_ahead, goal_state_index));
				}
				else
				{
					desired_state_index = max(desired_state_index, min(current_state_index + indices_ahead, goal_indices[min(goals_index+1, int(goal_indices.size())-1)]));
				}
			}
		}
	}
        /* This is commented for fork movement control
	if (current_state_index >= input_path.size() - 3 && current_state_index <= input_path.size() - 1)
	{
		goal_reached = 1;
	}
        */
	closest_state_on_path = input_path[current_state_index];
}

//Computes closest point on the replanned path given an input robot pose
void controller::find_closest_point_replan(const robot_state& input_state)
{
	int lower_index, upper_index, i, indices_ahead_replan = 3;
	double dist, closest_dist = 1000.0;
	unsigned int mapx, mapy;
	if (replan_done == 0)
    { desired_state_index_replan = 0; }
	lower_index = max(current_state_index_replan - 5, 0);   // changing this line for fork movement control
	if (current_state_index_replan < goal_state_index_replan)
	{
		upper_index = min(current_state_index_replan + 10, goal_state_index_replan);
	}
	else
	{
		upper_index = min(current_state_index_replan + 10, goal_state_index_replan);
	}
	for (i = lower_index; i <= upper_index; i++)        // lower_index, upper_index are the range of indices in which we find the closest point
	{
		if (i >= 0 && i < replanned_path_robot.size())       
		{
			dist = distance(replanned_path_robot[i], input_state);    // compute distance of point on path from input pose
			if (dist <= closest_dist && i >= current_state_index_replan)     // if distance is lesser than the current least distance, we store it
			{
				closest_dist = dist;
				current_state_index_replan = min(i,goal_state_index_replan);
				if (current_state_index_replan < goal_state_index_replan)   //current state index shall not be greater than the goal state index
				{
					//computation of desired state index
					desired_state_index_replan = max(desired_state_index_replan, min(current_state_index_replan + indices_ahead_replan, goal_state_index_replan));
				}
				else
				{  
					//computation of desired state index
					desired_state_index_replan = max(desired_state_index_replan, min(current_state_index_replan + indices_ahead_replan, goal_state_index_replan));
				}
			}
		}
	}
	closest_state_on_path = replanned_path_robot[current_state_index_replan];  //compute closest state on path using the index found
}

void controller::assign_state_xythetavel(double state_x, double state_y, double state_theta, double state_vel, state_xytheta_vel& state)
{
	state.x = state_x;
	state.y = state_y;
	state.theta = state_theta;
	state.vel = state_vel;
}

inline int controller::sign(double x, double y)
{
	return (x >= y ? 1: -1);
}

inline int controller::sign(double x)
{
	if (x > 0)
	{
		return 1;
	}
	else if(x < 0)
	{
		return -1;
	}
	else if (x == 0)
	{
		return 0;
	}
	return 0;
}

void controller::output_trajectory()
{
	if (trajectory.size() > 0)
	{
		cout<<"\nTrajectory size: "<<trajectory.size()<<"\n";
		cout<<"Trajectory values: \n";
		for (int i = 0; i < trajectory.size(); i++)
		{
			cout<<i+1<<". "<<"x: "<<trajectory[i].x<<"  y: "<<trajectory[i].y<<" theta: "<<trajectory[i].theta*180/PI<<"  vel: "<<trajectory[i].vel<<" gamma: "
				<<trajectory[i].gamma<<"\n";
		}
	}
}

void controller::output_replanned_trajectory()
{
	if (replanned_trajectory.size() > 0)
	{
		cout<<"\nReplanned Trajectory size: "<<replanned_trajectory.size()<<"\n";
		cout<<"start x: "<<start_odom.pose.pose.position.x<<" y: "<<start_odom.pose.pose.position.y<<" vel: "<<
		start_odom.twist.twist.linear.x<<" theta: "<<current_state.theta<<"\n";
		cout<<"goal x: "<<goal_odom.pose.pose.position.x<<" y: "<<goal_odom.pose.pose.position.y<<" vel: "<<
		goal_odom.twist.twist.linear.x<<" theta: "<<replan_goal.theta<<"\n";
		
		cout<<"Replanned Trajectory values: \n";
		for (int i = 0; i < replanned_trajectory.size(); i++)
		{
			cout<<i+1<<". "<<"x: "<<replanned_trajectory[i].x<<"  y: "<<replanned_trajectory[i].y<<" theta: "<<replanned_trajectory[i].theta*180/PI<<"  vel: "<<replanned_trajectory[i].vel<<" gamma: "
				<<replanned_trajectory[i].gamma<<"\n";
		}
	}
}

void controller::output_path()
{
	if (input_path.size() > 0)
	{
		cout<<"\npath size: "<<input_path.size()<<"\n";
		cout<<"Path: \n";
		for (int i = 0; i < input_path.size(); i++)
		{
			cout<<i+1<<". "<<"x: "<<input_path[i].x<<"  y: "<<input_path[i].y<<"  theta: "<<input_path[i].theta<<" gamma: "<<input_path[i].gamma<<"\n";
		}
	}
}

void controller::obtain_current_state(const robot_navigation::robot_state_msg::ConstPtr& msg)
{
	//current_state.x = msg->x - sensor_offset*cos(msg->theta);   //this ensures that the current state represents base link point
	//current_state.y = msg->y - sensor_offset*sin(msg->theta);   //the point between the two rear wheels
	current_state.x = msg->x;    
	current_state.y = msg->y;
	current_state.theta = msg->theta;
	//current_state.x = msg->x + sensor_offset*cos(msg->theta);
	//current_state.y = msg->y + sensor_offset*sin(msg->theta);
	current_pos.point.x = current_state.x;
	current_pos.point.y = current_state.y;
	current_pos.point.z = 0.0;
	current_pos.header.stamp = ros::Time::now();
	current_pos.header.frame_id = "map";
	pub_point.publish(current_pos);
	//current_state.gamma = msg->gamma;
}

void controller::obtain_current_state(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	current_state.x = msg->pose.pose.position.x;
	current_state.y = msg->pose.pose.position.y;
	current_state.theta = quaternion_to_angle(msg->pose.pose.orientation);
	//current_state.gamma = msg->gamma;
}

void controller::compute_desired_state()
{
    desired_state.x = input_path[desired_state_index].x;
	desired_state.y = input_path[desired_state_index].y;
    desired_state.theta = input_path[desired_state_index].theta;
	desired_state.gamma = trajectory[desired_state_index].gamma;
}

//stores the desired state on the replanned path in desired state
void controller::compute_desired_state_replan()
{
    desired_state.x = replanned_path_robot[desired_state_index_replan].x;
	desired_state.y = replanned_path_robot[desired_state_index_replan].y;
	desired_state.theta = replanned_path_robot[desired_state_index_replan].theta;
	desired_state.gamma = replanned_trajectory[desired_state_index_replan].gamma;
}

void controller::change_output_coordinates()
{
	double eps = 0.02;
	current_chained.x_1 = current_state.x;
	if (abs(abs(current_state.theta) - PI/2) <= eps)
	{
		if ((abs(current_state.theta) - PI/2) < 0)
		{
			current_state.theta = sign(current_state.theta)*(1.52);
		}
		else if ((abs(current_state.theta) - PI/2) > 0)
		{
			current_state.theta = sign(current_state.theta)*(1.62);
		}
	}
	else if(abs(abs(current_state.theta) - 3*PI/2) <= eps)
	{
		if ((abs(current_state.theta) - 3*PI/2) < 0)
		{
			current_state.theta = sign(current_state.theta)*(4.66);
		}
		else if ((abs(current_state.theta) - 3*PI/2) > 0)
		{
			current_state.theta = sign(current_state.theta)*(4.76);
		}
	}
	current_chained.x_2 = tan(current_state.gamma)/(pow(cos(current_state.theta),3));
	current_chained.x_3 = tan(current_state.theta);
	current_chained.x_4 = current_state.y;

	desired_chained.x_1 = desired_state.x;
	if (abs(abs(desired_state.theta) - PI/2) <= eps)
	{
		if ((abs(desired_state.theta) - PI/2) < 0)
		{
			desired_state.theta = sign(desired_state.theta)*(1.52);
		}
		else if ((abs(desired_state.theta) - PI/2) > 0)
		{
			current_state.theta = sign(desired_state.theta)*(1.62);
		}
	}
	else if(abs(abs(desired_state.theta) - 3*PI/2) <= eps)
	{
		if ((abs(desired_state.theta) - 3*PI/2) < 0)
		{
			desired_state.theta = sign(desired_state.theta)*(4.66);
		}
		else if ((abs(desired_state.theta) - 3*PI/2) > 0)
		{
			current_state.theta = sign(desired_state.theta)*(4.76);
		}
	}
	desired_chained.x_2 = tan(desired_state.gamma)/(pow(cos(desired_state.theta),3));
	desired_chained.x_3 = tan(desired_state.theta);
	desired_chained.x_4 = desired_state.y;
}

void controller::make_desired_state_zero(bool local)  // This function computes current state with respect to desired state robot and global coordinate frame 
{                                                     // This function also computes dist_to_goal.x and dist_to_goal.y in robot coordinate frame
	double eps = 0.02;
	robot_state current_state_, desired_state_;
	double x_g, y_g;
	//robot_state current_transformed;
	current_transformed.x = current_state.x - desired_state.x;
	current_transformed.y = current_state.y - desired_state.y;
	if (current_state.theta <= desired_state.theta)
	{
		if (abs(current_state.theta - desired_state.theta) <= abs(current_state.theta - (desired_state.theta - 2*PI)))
		{
			current_transformed.theta = current_state.theta - desired_state.theta;

		}
		else
		{
			current_transformed.theta = current_state.theta - (desired_state.theta - 2*PI);
	
		}

	}
	else if (current_state.theta > desired_state.theta)
	{
		if (abs(current_state.theta - desired_state.theta) <= abs(desired_state.theta - (current_state.theta - 2*PI)))
		{
			current_transformed.theta = current_state.theta - desired_state.theta;

		}
		else
		{
			current_transformed.theta = (current_state.theta - 2*PI) - desired_state.theta;

		}
	}
	//current_transformed.theta = current_state.theta - desired_state.theta;
	current_transformed.gamma = current_state.gamma - desired_state.gamma;
	if (local == true)
	{
		x_g = current_state.x - desired_state.x;
		y_g = current_state.y - desired_state.y;
		current_transformed.x = x_g*cos(current_state.theta) + y_g*sin(current_state.theta);
		current_transformed.y = y_g*cos(current_state.theta) - x_g*sin(current_state.theta);
	}
	robot_state dist_to_goal_, dist_to_pallet_;
	//dist_to_goal_.x = current_state.x - goal_state.x;   // for single goal
	//dist_to_goal_.y = current_state.y - goal_state.y;   // for single goal
	dist_to_goal_.x = current_state.x - goals_vec[min(goals_index, int(goals_vec.size()) -1)].x;
	dist_to_goal_.y = current_state.y - goals_vec[min(goals_index, int(goals_vec.size()) -1)].y;
	dist_to_pallet_.x = current_state.x - pallet_goal.x;
	dist_to_pallet_.y = current_state.y - pallet_goal.y;
	dist_to_goal.x = dist_to_goal_.x*cos(current_state.theta) + dist_to_goal_.y*sin(current_state.theta);
	dist_to_goal.y = dist_to_goal_.y*cos(current_state.theta) - dist_to_goal_.x*sin(current_state.theta);
	dist_to_pallet.x = dist_to_pallet_.x*cos(current_state.theta) + dist_to_pallet_.y*sin(current_state.theta);
	dist_to_pallet.y = dist_to_pallet_.y*cos(current_state.theta) + dist_to_pallet_.x*sin(current_state.theta);
	desired_zero = 1;
}

void controller::make_closest_state_zero(bool local)  // This function computes current state with respect to desired state robot and global coordinate frame 
{                                                     // This function also computes dist_to_goal.x and dist_to_goal.y in robot coordinate frame
	double eps = 0.02;
	robot_state current_state_, desired_state_;
	double x_g, y_g;
	//robot_state current_transformed;
	current_trans_wrt_path.x = current_state.x - closest_state_on_path.x;
	current_trans_wrt_path.y = current_state.y - closest_state_on_path.y;
	if (current_state.theta <= closest_state_on_path.theta)
	{
		if (abs(current_state.theta - closest_state_on_path.theta) <= abs(current_state.theta - (closest_state_on_path.theta - 2*PI)))
		{
			current_trans_wrt_path.theta = current_state.theta - closest_state_on_path.theta;

		}
		else
		{
			current_trans_wrt_path.theta = current_state.theta - (closest_state_on_path.theta - 2*PI);
	
		}

	}
	else if (current_state.theta > closest_state_on_path.theta)
	{
		if (abs(current_state.theta - closest_state_on_path.theta) <= abs(closest_state_on_path.theta - (current_state.theta - 2*PI)))
		{
			current_trans_wrt_path.theta = current_state.theta - closest_state_on_path.theta;

		}
		else
		{
			current_trans_wrt_path.theta = (current_state.theta - 2*PI) - closest_state_on_path.theta;

		}
	}
	//current_transformed.theta = current_state.theta - desired_state.theta;
	if (local == true)
	{
		x_g = current_state.x - closest_state_on_path.x;
		y_g = current_state.y - closest_state_on_path.y;
		current_trans_wrt_path.x = x_g*cos(closest_state_on_path.theta) + y_g*sin(closest_state_on_path.theta);
		current_trans_wrt_path.y = y_g*cos(closest_state_on_path.theta) - x_g*sin(closest_state_on_path.theta);
	}
}

//This function is yet to be tested. It is control using the chained form of state space equations
void controller::compute_error_chained()
{
	if (desired_zero == 1)
	{
		error_chained.x_1 = current_chained.x_1;
		error_chained.x_2 = current_chained.x_2;
		error_chained.x_3 = current_chained.x_3;
		error_chained.x_4 = current_chained.x_4;
	}
	else
	{
		error_chained.x_1 = current_chained.x_1 - desired_chained.x_1;
		error_chained.x_2 = current_chained.x_2 - desired_chained.x_2;
		error_chained.x_3 = current_chained.x_3 - desired_chained.x_3;
		error_chained.x_4 = current_chained.x_4 - desired_chained.x_4;
	}
}

void controller::compute_control_input()
{
	velocities u, v;
	double wheel_vel, gamma;
	ros::Duration delta;
	v.val1 = trajectory[desired_state_index].vel;
	u.val1 = v.val1/cos(current_transformed.theta);
	u.val2 = -gains.k1*abs(u.val1)*error_chained.x_2 - gains.k2*u.val1*error_chained.x_3 - gains.k3*abs(u.val1)*error_chained.x_4;
	v.val2 = -3*sin(current_transformed.theta)*pow(sin(current_transformed.gamma),2)*u.val1/pow(cos(current_transformed.theta),2)
		+ pow(cos(current_transformed.theta),3)*pow(cos(current_transformed.theta),2)*u.val2;
	delta_time = ros::Time::now().toSec() - prev_time.toSec();
	prev_time = ros::Time::now();
	//delta_time = delta.toSec();
	gamma = current_state.gamma + delta_time*v.val2;
	gamma = sign(gamma)*min(abs(gamma),gamma_max);
	//wheel_vel = v.val1/cos(gamma);
	wheel_vel = v.val1/(cos(current_transformed.theta)*cos(gamma));
	control_input.val1 = wheel_vel;
	control_input.val2 = gamma;
}

// Initializing gains used for computing control inputs 
void controller::initialize_gain_matrix()
{
	gains_straight.k11 = 0.6;
	gains_straight.k12 = 0.0;
	gains_straight.k13 = 0.0;
	gains_straight.k21 = 0.0;
	gains_straight.k22 = 0.18;
	gains_straight.k23 = 0.9;

	/*
	   gains_turn.k11 = 0.6;
	   gains_turn.k12 = -0.0468;
	   gains_turn.k13 = -0.078;
	   gains_turn.k21 = -0.156;
	   gains_turn.k22 = 0.192;
	   gains_turn.k23 = 0.92;
	 */

	gains_turn.k11 = 0.10;
	gains_turn.k12 = -0.0446;
	gains_turn.k13 = -0.0372;
	gains_turn.k21 = -0.150;
	//gains_turn.k22 = 0.738;
	//gains_turn.k22 = 0.95;
	//gains_turn.k22 = 1.90;
	gains_turn.k22 = 2.20;
	//gains_turn.k23 = 1.83;
	//gains_turn.k23 = 1.20;
	gains_turn.k23 = 1.10;
	gains_turn.k24 = 0.20;

	gains_turn.kv11 = 0.10;
	gains_turn.kv12 = -0.0446;
	gains_turn.kv13 = -0.0372;
	gains_turn.kv21 = -0.150;
	gains_turn.kv22 = 2.20;
	gains_turn.kv23 = 1.10;
	gains_turn.kv24 = 0.20;

	gains_reverse.k11 = 0.10;
	gains_reverse.k12 = -0.0446;
	gains_reverse.k13 = -0.0372;
	gains_reverse.k21 = -0.250;
	//gains_reverse.k22 = 2.50;
	//gains_reverse.k23 = -3.60;
	gains_reverse.k22 = 2.50;
	gains_reverse.k23 = -3.80;
	gains_reverse.k24 = 1.00;

	gains_reverse_1.k11 = 0.10;
	gains_reverse_1.k12 = -0.0446;
	gains_reverse_1.k13 = -0.0372;
	gains_reverse_1.k21 = -0.250;
	//gains_reverse.k22 = 2.50;
	//gains_reverse.k23 = -3.60;
	gains_reverse_1.k22 =  2.50;  // 3.00
	gains_reverse_1.k23 = -3.80; //-3.80
	//gains_reverse_1.k24 = 1.00;
	gains_reverse_1.k24 = 0.5;

	gains_reverse_2.k11 = 0.10;
	gains_reverse_2.k12 = -0.0446;
	gains_reverse_2.k13 = -0.0372;
	gains_reverse_2.k21 = -0.250;
	//gains_reverse.k22 = 2.50;
	//gains_reverse.k23 = -3.60;
	//gains_reverse_2.k22 = 1.50;   //already tested gain value
	gains_reverse_2.k22 = 3.0;     //new gain value
	gains_reverse_2.k23 = -4.50;
	gains_reverse_2.k24 = 1.00;


}

void controller::compute_error_xytheta()
{

	// computation of error
	error_xytheta.x = current_transformed.x;
	error_xytheta.y = current_transformed.y;
	error_xytheta.theta = current_transformed.theta;
	error_xytheta_dot.x = error_xytheta.x - prev_error_xytheta.x;
	error_xytheta_dot.y = error_xytheta.y - prev_error_xytheta.y;
	error_xytheta_dot.theta = error_xytheta.theta - prev_error_xytheta.theta;
	prev_error_xytheta = error_xytheta;
    

    // summation of errors till now
	sum_error.x = sum_error.x + abs(current_trans_wrt_path.x);
	sum_error.y = sum_error.y + abs(current_trans_wrt_path.y);
	sum_error.theta = sum_error.theta + abs(current_trans_wrt_path.theta);
	max_error.x = max(max_error.x, abs(current_trans_wrt_path.x));   // maximum error till now
	max_error.y = max(max_error.y, abs(current_trans_wrt_path.y));
	max_error.theta = max(max_error.theta, abs(current_trans_wrt_path.theta));
	iterations = iterations + 1;

}

// This function computes comtrol inputs for the robot given error in x, y, theta and velocity of the robot at desired state.
void controller::compute_control_input_xytheta()
{
	velocities u, v;
	double wheel_vel, gamma;
	int sign_vel = 1;         
	if (replanned == 0)
	{
	if (sign(trajectory[desired_state_index].vel) >= 0)   // control input computation for forward velocity
	{ 
		sign_vel = 1;
		u.val1 = -gains_turn.k11*error_xytheta.x - gains_turn.k12*error_xytheta.y - gains_turn.k13*error_xytheta.theta;
		u.val2 = - gains_turn.k21*error_xytheta.x - gains_turn.k22*error_xytheta.y 
			- sign_vel*gains_turn.k23*error_xytheta.theta - gains_turn.k24*(current_state.gamma - desired_state.gamma); 
	}
	else           // control input computation for reverse velocity
	{
		u.val1 = -gains_reverse_1.k11*error_xytheta.x - gains_reverse_1.k12*error_xytheta.y - gains_reverse_1.k13*error_xytheta.theta;
		u.val2 = - gains_reverse_1.k21*error_xytheta.x - gains_reverse_1.k22*error_xytheta.y
			- gains_reverse_1.k23*error_xytheta.theta;
	}

	if (cos(feedback_vals.val2+error_xytheta.theta) != 0)  
	{
		//computation of wheel velocity for the robot.
		control_input.val1 = trajectory[current_state_index].vel/abs(cos(feedback_vals.val2+error_xytheta.theta)) + u.val1; 	
	} 
	else
	{
		//computation of steer angle for the robot.
		control_input.val1 = trajectory[current_state_index].vel/0.15 + u.val1;
	}
    }
    else if (replanned == 1)
    {
    if (sign(replanned_trajectory[desired_state_index_replan].vel) >= 0)
	{ 
		sign_vel = 1;
		u.val1 = -gains_turn.k11*error_xytheta.x - gains_turn.k12*error_xytheta.y - gains_turn.k13*error_xytheta.theta;
		u.val2 = - gains_turn.k21*error_xytheta.x - gains_turn.k22*error_xytheta.y 
			- sign_vel*gains_turn.k23*error_xytheta.theta - gains_turn.k24*(current_state.gamma - desired_state.gamma); 
	}
	else
	{
		u.val1 = -gains_reverse_1.k11*error_xytheta.x - gains_reverse_1.k12*error_xytheta.y - gains_reverse_1.k13*error_xytheta.theta;
		u.val2 = - gains_reverse_1.k21*error_xytheta.x - gains_reverse_1.k22*error_xytheta.y
			- gains_reverse_1.k23*error_xytheta.theta;
	}
	if (cos(feedback_vals.val2+error_xytheta.theta) != 0)
	{
		control_input.val1 = replanned_trajectory[current_state_index_replan].vel/abs(cos(feedback_vals.val2+error_xytheta.theta)) + u.val1; 
	} 
	else
	{
		control_input.val1 = replanned_trajectory[current_state_index_replan].vel/0.15 + u.val1;
	}	
    }
	//control_input.val2 = feedback_vals.val2 + u.val2;
	//control_input.val2 = current_state.gamma + u.val2;
	control_input.val2 = u.val2;

} 

// Computes lookahead indices
void controller::compute_indices_ahead(bool goal_condition_)
{
	if (goal_condition_)
	{
		indices_ahead = indices_ahead_goal;
	}    
	else
	{
		indices_ahead = indices_ahead_in;
	}
}


// stores the start, goal and intermediate points for replanning in waypoints
void controller::set_start_goal_replan()
{   waypoints.clear();
	start_odom.pose.pose.position.x = current_state.x;
	start_odom.pose.pose.position.y = current_state.y;
	start_odom.pose.pose.orientation = angle_to_quaternion(current_state.theta);
	start_odom.twist.twist.linear.x = trajectory[current_state_index].vel;
	start_odom.twist.twist.linear.y = 0;
	start_odom.twist.twist.angular.z = 0;

	goal_odom.pose.pose.position.x = replan_goal.x;
	goal_odom.pose.pose.position.y = replan_goal.y;
	goal_odom.pose.pose.orientation = angle_to_quaternion(replan_goal.theta);
	goal_odom.twist.twist.linear.x = replan_goal.vel;
	goal_odom.twist.twist.linear.y = 0;
	goal_odom.twist.twist.angular.z = 0;
	waypoints.push_back(start_odom);
	waypoints.push_back(goal_odom);
}

//This funciton is specific to teb local planner. It stores the output of teb local planner to replanned path robot,
//replanned path trajectory and replanned path output 
void controller::convert_to_robot_states()
{
	robot_state temp_state;
	state_xytheta_vel state;
	vector<double> temp(3,0.0);
	replanned_path_robot.clear();
	replanned_trajectory.clear();
	replanned_path_output.clear();
	cout<<"local_trajectory size: "<<local_trajectory.size()<<"\n";
    for (int i = 0; i < local_trajectory.size(); i++)
    {
    	temp_state.x = local_trajectory[i].pose.position.x;
    	temp_state.y = local_trajectory[i].pose.position.y;
    	temp_state.theta = quaternion_to_angle(local_trajectory[i].pose.orientation);
    	temp_state.vel = local_trajectory[i].velocity.linear.x;
    	temp[0] = temp_state.x;
    	temp[1] = temp_state.y;
    	replanned_path_output.push_back(temp);
    	replanned_path_robot.push_back(temp_state);
        assign_state_xythetavel(temp_state.x, temp_state.y, temp_state.theta, temp_state.vel, state);
        replanned_trajectory.push_back(state);
    }	
}

//This function contains basic functions of the computations to be done inside the control loop
//The computations include setting a current robot pose, finding closest point on path from current robot pose,
//computing desired state of the robot, transforming desired state to local robot frame, transforming closest state on path
//to local robot frame, computing the error in x, y, theta and computing the control input
void controller::control_flow_functions(const robot_navigation::robot_state_msg::ConstPtr& msg)
{ 
	obtain_current_state(msg);
	if (replanned == 1)
	{
		find_closest_point_replan(current_state); // compute closest pooint on the replanned path given the current state
		compute_desired_state_replan();  //compute desired state on the replanned path 
		if (abs(goal_state_index_replan - desired_state_index_replan) < 3)  // check if robot is robot near goal state of replanned path
	    {   replanned = 0;  } 
	}
	if (replanned == 0)
	{
		find_closest_point(current_state); // find closest point on robot path from current robot pose
	    compute_desired_state();     // compute desired state of robot, using index of closest point and lookahead indices
	}
	make_desired_state_zero(1);   // transform desired state to local frame of robot 
	make_closest_state_zero(1);  // transform closet point on path to local frame of the robot 
	compute_error_xytheta();    // compiute error in x, y, theta
	compute_control_input_xytheta();  // compute control input using error in x, y, theta
}

// Computes goal to to used for replanning
// replan_goal stores the goal for replanning
void controller::compute_replan_goal(int curr_state_ind)
{
    //if (pallet_position_st != goal_current.pallet_state && using_replan == 1)
    //{
    	//if (distance_to_pallet <= 4.0 && replan_done == 0)
        //{
        	//cout<<"replanning for pallet\n";
        	//replan_path(current_state, goal_current, goal_state_index);
    	    //replan_for_pallet(current_state, goal_current);
            unsigned char cost_thresh = 15;
            unsigned int mapx, mapy; 
    	    find_closest_point(current_state, curr_state_ind);
    	    goal_index_for_replanning = min(curr_state_ind + 170, goal_state_index);
    	    cout<<"curr_state_ind: "<<curr_state_ind<<" goal_state_index: "<<goal_state_index<<"\n";

    	    replan_goal = input_path[goal_index_for_replanning];
    	    bool temp = local_costmap_->worldToMap(replan_goal.x, replan_goal.y, mapx, mapy);
    	    if (local_costmap_->getCost(mapx,mapy) > cost_thresh)
    	    { for (int i = goal_index_for_replanning; i < input_path.size(); i++)
    	       { temp = local_costmap_->worldToMap(input_path[i].x, input_path[i].y, mapx, mapy); 
    	       	if (local_costmap_->getCost(mapx,mapy) < cost_thresh)
    	       		{   goal_index_for_replanning = i;
    	       			break; }
    	       	}
    	     }
    	     cout<<"cost at goal: "<<(int)local_costmap_->getCost(mapx,mapy)<<"\n";     
    	    replan_goal = input_path[goal_index_for_replanning];
    	    replan_goal.vel = trajectory[goal_index_for_replanning].vel;
    	    //replan_goal = input_path[min(current_state_index + 180, goal_state_index)];
    	    replan_goal_assigned = 1;
        //}
    //}
}

//Computes pallet and goal condition based on distance from next goal or pallet pose
void controller::compute_goal_pallet_condition(double& distance_to_goal_, bool& goal_condition_, bool& pallet_condition_)
{
	distance_to_goal_ = sqrt(pow(dist_to_goal.x,2)+pow(dist_to_goal.y,2));

    goal_condition_ = sqrt(pow(dist_to_goal.x,2)+pow(dist_to_goal.y,2)) < 0.80 && pallet_position_st == goal_current.pallet_state;

    pallet_condition_ = distance_to_goal_ < 0.80 && pallet_position_st != goal_current.pallet_state;
}

//This function limits the control inputs (velocity, steer angle) to a minimum and maximum value and performs other 
//checks on them as well
void controller::perform_velocity_check(bool goal_condition, bool pallet_condition, double distance_to_goal)
{
	if (abs(control_input.val1) <= 0.05 && goal_condition != 1 && pallet_condition != 1)
	{
		//control_input.val1 = sign(control_input.val1)*0.05;
		cout<<"inside val1 zero condt..."<<"\n";
		control_input.val1 = sign(trajectory[current_state_index].vel)*0.05;  // if computed control input vel is less than 0.05
		// then give a minimum velocity of 0.05
		if (trajectory[current_state_index].vel == 0)        // if current_state_index vel is 0 (stored in trajectory), then give a minimum of 0.05
		{
			control_input.val1 = sign(trajectory[current_state_index+20].vel)*0.05;
		}
	}
	else if (abs(control_input.val1) <= 0.05 && (goal_condition == 1 || pallet_condition == 1))
	{
		// if control input velocity is less than 0.05 and goal or pallet is nearby, then give the computed control input velocity
		control_input.val1 = sign(trajectory[current_state_index].vel)*abs(control_input.val1); 
	}
	if (distance_to_goal < 0.15 && pallet_condition == 1)
	{
		// if pallet pick/drop is less than 15 cm away, control input velocity shall be zero
		control_input.val1 = 0.0;
	}
	if (abs(control_input.val1) > Velocity_Max)
	{
		// this limits the value of control input velocity to Velocity_Max
		control_input.val1 = sign(control_input.val1)*Velocity_Max;
	}
	if (abs(control_input.val2) > 1.3)
	{
		// this limits the value of control input angle to 1.30 radians
		control_input.val2 = sign(control_input.val2)*1.30;
	}
	if (distance(current_state, start_vec[min(start_index, int(start_vec.size())-1)]) <= 1)
	{   // if the robot is has just started after reaching a specified goal point
		if ( control_input.val1 < 0)    // then robot shall have a min of 0.1 vel if it is travelling in reverse direction
		{
			control_input.val1 = -1*max(0.1,abs(control_input.val1));
		}
	}
	if (goal_condition)
	{   // if robot is near a goal, it shall have a velocity of 0.05
		control_input.val1 = sign(control_input.val1)*0.05;
	}
}

//Chekcs the stopping condition given next goal or pallet pose. Depending upon the condition, it stops the robot, assigns new forkstate
//and updates relevant indices (goals_index, start_index, goal_state_index, current_state_index, desired_state_index)
void controller::perform_goal_pallet_check(double distance_to_goal, robot_navigation::control_commands& control_msg)
{
	// stopping condition for the robot at goal or at pallet pick/drop location
	if (distance_to_goal <= 0.12 || (distance_to_goal <= 15 && control_input.val1 == 0) || current_state_index == goal_indices[goals_index] ||
			(abs(dist_to_goal.x) <= 0.07 && abs(dist_to_goal.y) <= 0.07) || (((abs(dist_to_goal.x) <= 0.25 && abs(dist_to_goal.y) <= 0.25)
					||	abs(current_state_index - goal_indices[goals_index]) <= 5)  && goal_current.goal == 1 && pallet_position_st == goal_current.pallet_state))

	{
		control_input.val1 = 0.0;    //setting control input velocity and angle to zero
		control_input.val2 = 0.0;

		if (robot_stopped == 0)       // this condition will be entered the first time the above condition (of goal and pallet distances) is met
		{
			cout<<"robot stop condit...\n";
			robot_stopped = 1;       // we set the robot_stopped flag to 1
			prev_time_stopped = ros::Time::now();   // record time at this instant                                 
		}
		curr_time = ros::Time::now();                      
		duration_stop = curr_time - prev_time_stopped;       //duration_stop stores the time duration between first time the robot stopped
		cout<<"duration: "<<duration_stop.toSec()<<"\n";     //and the next time control_loop (and hence this function, condition) is entered
		cout<<"robot_stopped: "<<robot_stopped<<"\n";      
                if ((duration_stop.toSec() > 1.0 && robot_stopped == 1) || (goal_current.goal == 1 && pallet_position_st == goal_current.pallet_state))
		{      // one second has passed after the robot stopped OR the robot is not near a pallet pick/drop location
			cout<<"duration greater than 1\n";
			robot_stopped = 0;

			if (goals_vec[goals_index].pallet_state != pallet_position_st && goal_reached_count == 0)  // if goal is a pallet pick/drop location
			{
				control_msg.val3 = goals_vec[goals_index].pallet_state;    // set control input forkstate value to goal forkstate value 
				pallet_position_st = goals_vec[goals_index].pallet_state;     // update pallet state 
				cout<<"move forklift to pallet state: "<<goals_vec[goals_index].pallet_state<<"\n";
			}

			if (goals_index == int(goals_vec.size()) - 1)   // if this is the last goal
			{
				goal_reached_count = min(2,goal_reached_count + 1);   // update goal_reached_count
				cout<<"goal reached\n";
				robot_stopped = 1;                                       
				if (goal_reached_count == 2)
				{
					cout<<"goal_reached\n";
					goal_reached = 1;            // set goal_reached flag to one
					if (goal_reached == 1)
					{                     // compute error after reaching goal
						mean_error.x = sum_error.x/iterations;
						mean_error.y = sum_error.y/iterations;
					    mean_error.theta = sum_error.theta/iterations;
						cout<<"max_error x: "<<max_error.x<<" y: "<<max_error.y<<" theta: "<<max_error.theta<<"\n";
						cout<<"mean_error x: "<<mean_error.x<<" y: "<<mean_error.y<<" theta: "<<mean_error.theta<<"\n";
					}
				} 
			}
			else        // if this is not the last goal
			{
				goals_index = goals_index + 1;   // increase index of goal state by one ; goals were stored in a vector initially
				start_index = start_index + 1;   // increase index of start state by one; start points (occuring just after goal) were stored in a vector initially
				//update current_state_index, desired_state_index, and goal_current
				current_state_index = start_indices[min(start_index, int(start_indices.size())-1)];  
				desired_state_index = min(start_indices[min(goals_index, int(start_indices.size())-1)]+10, goal_indices[min(goals_index, int(goal_indices.size())-1)]);
				goal_current = goals_vec[min(goals_index, int(goals_vec.size()) - 1)];
			}  
			goal_state_index = goal_indices[goals_index];  // update current goal index
		}
	}
}

void controller::replan_using_teb_local_planner()
{
	set_start_goal_replan();  
    cout<<"after set start goal replan\n";
	local_planner->setPlan(waypoints);   // for teb llocal planner, sets an initial plan for the teb local planner
	cout<<"after setPlan\n";
	local_planner->getTrajectory(local_trajectory);    // teb local planner computes the trajectory
	cout<<"after getTrajectory\n";
    convert_to_robot_states();     // store output of teb local planner in member variables 
    cout<<"after convert_to_robot_states\n";
    //replanned = 1;
    replan_done = 1;
}


//This function performs computations to generate control inputs given a robot pose
void controller::control_loop(const robot_navigation::robot_state_msg::ConstPtr& msg)
{
	bool goal_condition;     // goal condition indicates if we are a certain distance from our next goal
	bool pallet_condition;   // pallet condition indicates if we are a certain distance from our next pallet pose
	double distance_to_goal, distance_to_pallet = 100.0;
	int curr_state_ind;    // curr_state_ind stores indice of closest point on path from current robot pose
	//goal_condition = sqrt(pow(dist_to_goal.x,2)+pow(dist_to_goal.y,2)) < 0.50 && pallet_position == goal_current.pallet_up;
    distance_to_pallet = sqrt(pow(dist_to_goal.x,2)+pow(dist_to_goal.y,2));   // distance from pallet pose
    distance_to_pallet = sqrt(pow(msg->x - input_path[goal_state_index].x,2)+pow(msg->y - input_path[goal_state_index].y,2));
    goal_condition = sqrt(pow(dist_to_goal.x,2)+pow(dist_to_goal.y,2)) < 0.50 && pallet_position_st == goal_current.pallet_state;
	
	compute_indices_ahead(goal_condition);    // compute lookahead indices using goal_condition  

    compute_replan_goal(curr_state_ind);     // compute goal for replanning

    cout<<"replan_goal x: "<<replan_goal.x<<" y: "<<replan_goal.y<<" theta: "<<replan_goal.theta<<"\n";

    if (test_teb_local_planner)      // This is temporary, remove after teb local planner has been tested.
    { replan_using_teb_local_planner(); }

    control_flow_functions(msg);  // compute desired state, error and the control inputs 

    compute_goal_pallet_condition(distance_to_goal, goal_condition, pallet_condition);   // compute goal, pallet condition based on distance from goal, pallet

	//pub = nc.advertise<robot_navigation::control_commands>("control_input", 1);
	robot_navigation::control_commands control_msg;
	control_msg.val3 = 0;

	perform_velocity_check(goal_condition, pallet_condition, distance_to_goal); // check minimum, maximum limits of velocity and velocity 
    // limits based on distance from on goal or pallet

	cout<<"replanned path size: "<<replanned_path_output.size()<<" replan goal assigned: "<<replan_goal_assigned<<"\n";

	cout<<"current state index replan: "<<current_state_index_replan<<" desired_state_index_replan: "<<desired_state_index_replan<<"\n";
    cout<<"current state x: "<<current_state.x<<" y: "<<current_state.y<<" theta: "<<current_state.theta;
    cout<<"  desired state x: "<<desired_state.x<<" y: "<<desired_state.y<<" theta: "<<desired_state.theta<<"\n";
    cout<<"error x: "<<error_xytheta.x<<" y: "<<error_xytheta.y<<" theta: "<<error_xytheta.theta<<"\n";
	control_msg.val3 = pallet_position_st;

	perform_goal_pallet_check(distance_to_goal, control_msg); // check stopping condition and issue relevant velocity commands

	control_msg.val1 = control_input.val1;
	control_msg.val2 = control_input.val2;
	cout<<"sending speed: "<<control_msg.val1<<" steer: "<<control_msg.val2<<" fork: "<<control_msg.val3<<"\n";
	pub.publish(control_msg);   // publish wheel velocity, steer angle and forkstate
}

void controller::visualize()
{
	ros::Time time_curr_;
	vector<double> p1(3,0.0), p2(3,0.0);
    ofstream points;
	string line, path;
	double color_num;
	double val, x, y, theta, vel, delta_time_, gamma_;
	int points_no = 1, temp_int;
	char temp_char;
	color_num = 0.0;
	vector<double> pos_temp(3,0.0), prev_pos_temp(3,0.0);
	line = "current_desired";
	path = "path_followed";
	cout<<"\ninside visualize\n";	
    points.open("/home/kush/catkin_ws/src/robot_navigation/points_file.txt");
	while(1)
	{
		//cout<<"\ninside visualize\n";
		if (state_feedback == 1)
		{
			p1[0] = current_state.x;
			p1[1] = current_state.y;
			p1[2] = 0.0;
			p2[0] = desired_state.x;
			p2[1] = desired_state.y;
			p2[2] = 0.0;
			pos_temp[0] = current_state.x;
			pos_temp[1] = current_state.y;
			pos_temp[2] = current_state.theta;
			if (abs(current_state.gamma)  >= 0.2) {
				gamma_ = sign(current_state.gamma)*abs(current_state.gamma);
			}
			else
			{
				gamma_ = 0.0;
			}
			path_followed.push_back(pos_temp);
			time_curr_ = ros::Time::now();
			points<<points_no<<". x: "<<pos_temp[0]<<" y: "<<pos_temp[1]<<" theta: "<<pos_temp[2]<<" gamma: "<<gamma_;
			delta_time_ = time_curr_.toSec() - time_prev_vel.toSec();
			time_prev_vel = time_curr_;			
			if (points_no == 1)
			{
				vel = 0.1;
			}
			else
			{
		     	vel = -1*sqrt(pow(pos_temp[0] - path_followed[path_followed.size() - 2][0],2) + pow(pos_temp[1] - path_followed[path_followed.size()-2][1],2))/delta_time_;
			}
            points<<" vel: "<<vel<<"\n";
			points_no++;
			visual->visualizeLine(p1, p2, 10, line);
			usleep(100000);
			if (goal_reached == 1 || robot_stopped == 1)
			{
				visual->visualizeConfiguration(color_num, path_followed, path);
				//points<<"end of file.\n";
		        points.close();
			}
			if (visualize_replan == 1)
			{
				visualize_replanned_path();
			}
		}
	}
}

void controller::visualize_replanned_path()
{
		visual->visualizePoints(replanned_path_output, 0, "replanned_path");
}


