#include <robot_navigation/robot_navigation.h>

using namespace std;

robot_nav::robot_nav(string name, costmap_2d::Costmap2DROS* planner_costmap_ros_)
{
	double turning_vel = 0.3, straight_vel = 0.6, precision_vel = 0.1;  // max velocities of robot at turns, straights, and pallet pick/drop
	int indices_ahead = 10;  // lookahead indices, this number will be added to the current state index
    vector< vector<double> > path_output_;  // stores output for visualization
    vector<state_xytheta_vel> trajectory_robot;  // stores trajectory (x, y, theta, vel) of the robot
	vector<robot_state> path_robot_;  //stores path (x, y, theta, goal, forkstate) of the robot
    robot_navigation::robot_state_msg start_msg;  //for storing robot start state, to be used when taking path from file and published for simulator
    bool using_replan_ = false, use_controller = true;  //flags for using replanning, controller
    use_planner = true;
    if (!use_planner)
    { use_file = true; }
    else 
    { use_file = false; }
    use_file = 0;
    cout<<"using_planner: "<<use_planner<<" using_file: "<<use_file<<" using_replan_: "<<using_replan_<<"\n";

    string file_name = "/home/kush/catkin_ws/src/robot_navigation/points_file_.txt";
	initialize_visualizer("navigation", 0.1, 0.1, 0.0, 0.0, 0.0);   // initializing visualizer
    cout<<"initialized visualizer\n";
    simulate_ = new planner(name, planner_costmap_ros_, n);  // creating object of planner
    
    cout<<"constructed planner\n";
    if (use_file)
    { simulate_->read_waypoints_from_file("/home/kush/catkin_ws/src/robot_navigation/launch/waypoints_file.txt");  
    }

    if (use_file)
    { plan_using_file(file_name, path_robot_, trajectory_robot, path_output_, start_msg); }  //reads points from a file
     // and assigns then to path_robot_, trajectory_robot_, and path_output_
    

    ros::spinOnce();

    if (use_planner)
    { plan_using_planner(path_output_, path_robot_, trajectory_robot, turning_vel, straight_vel, precision_vel); }
      //plan using the planner object 
    
    if (use_controller) 
    { 
    initialize_controller(indices_ahead, n, using_replan_, path_robot_, trajectory_robot, planner_costmap_ros_);

    output_path_size(path_robot_, trajectory_robot, path_output_);  //output size of path_robot_, trajectory_robot, path_output_
    }

    if (use_file) 
    { start_pub_robot_nav = n.advertise<robot_navigation::robot_state_msg>("robot_start_state", 1); // create publisher for publishing start state
      start_pub_robot_nav.publish(start_msg);  // publish start state of robot, to be used by simulator
    }
     
    if (use_controller){
    boost::thread th(&robot_nav::visualize_, this, path_output_);  // commented   // thread for visualizing path
    boost::thread th1(&controller::visualize, trajectory_obj); // commented      // thread for visualizing error and path followed
    }
    ros::spin();     
}

robot_nav::~robot_nav()
{
	delete visual;
	delete simulate_;
    delete trajectory_obj;
}

// this function reads points from a file and assigns then to robot_path_, robot_trajectory_ and path_output_
void robot_nav::plan_using_file(const string file_name, vector<robot_state>& robot_path_, vector<state_xytheta_vel>& robot_trajectory_,
                                vector< vector<double> >& path_output_, robot_navigation::robot_state_msg& start_msg)
{
    read_points_from_file(file_name, robot_path_, robot_trajectory_, path_output_);
    simulate_->path_planned = 1;
    start_msg.x = robot_path_[0].x;
    start_msg.y = robot_path_[0].y;
    start_msg.theta = robot_path_[0].theta;
}

// this function uses the 'make_plan_and_trajectory' to plan a path 
void robot_nav::plan_using_planner(vector< vector<double> >& path_output_, vector<robot_state>& path_robot_, 
         vector<state_xytheta_vel>& trajectory_robot_, double turning_vel, double straight_vel, double precision_vel)
{
    simulate_->specify_start_goal();  // commented    // specify the waypoints used in path
    simulate_->make_plan_and_trajectory(path_output_, path_robot_, trajectory_robot_, turning_vel, straight_vel, precision_vel);  //commented
    simulate_->output_trajectory(trajectory_robot_);  
}

// output size of path_output_, path_robot_ and trajectory_robot_. Helps to check that all are same. They shall be the same
void robot_nav::output_path_size(vector<robot_state>& path_robot_, vector<state_xytheta_vel>& trajectory_robot_, vector< vector<double> >& path_output_)
{
    cout<<"path_output size: "<<path_output_.size()<<"\n";
    cout<<"path_robot size: "<<path_robot_.size()<<"\n";
    cout<<"trajectory size: "<<trajectory_robot_.size()<<"\n";
}

// create controller object and send trajectory_robot_, path_robot_ computed by planner or from the file
void robot_nav::initialize_controller(int indices_ahead, ros::NodeHandle& n, bool using_replan_, vector<robot_state>& path_robot_,
       vector<state_xytheta_vel>& trajectory_robot_, costmap_2d::Costmap2DROS* planner_costmap_ros_)
{
    trajectory_obj = new controller(indices_ahead,n,simulate_,using_replan_, planner_costmap_ros_);  //create controller object
    trajectory_obj->receive_traj(trajectory_robot_);  // send trajectory_robot_ to controller object 
    trajectory_obj->receive_path(path_robot_);  // send path_robot_ to controller object
}

// create visualizer object. The respective class is in navviz.h
void robot_nav::initialize_visualizer(string vis_name, double robotW, double robotH, double OffX, double OffY, double OffZ)
{
	visual = new NavVisualizer(vis_name, robotW, robotH, OffX, OffY, OffZ);
}

//this function visualizes path of the robot once it has been computed
void robot_nav::visualize_(vector< vector<double> >& path_output)
{
	while(1)
	{
		if (simulate_->path_planned == 1)       // checking if path has been computed
	 	{
            usleep(100000);      
			visual->visualizePoints(path_output, 165, "path_output");   // visualize path of the robot 
		}
	}
}

//this function reads path points (closely spaced, not waypoints) from file and stores them in robot_path_, robot_trajectory_, path_output_ 
void robot_nav::read_points_from_file(const string path_name, vector<robot_state>& robot_path_, vector<state_xytheta_vel>& robot_trajectory_,
                                vector< vector<double> >& path_output_) 
{
    ifstream points_;
    points_.open(path_name.c_str());   // open file containing path 
    vector<double> temp(3,0.0);
    string line;
    int temp_int;
    char temp_char;
    double x, y, theta, gamma_, vel;
    robot_state temp_state;
    state_xytheta_vel temp_state_tr;   
    while(!points_.eof())
    {
        points_>>temp_int;
        skip_spaces(points_,3);
        points_>>x;     // read x value
        skip_spaces(points_,2);
        points_>>y;     //read y value
        skip_spaces(points_,6);
        points_>>theta; // read theta value
        skip_spaces(points_, 6);
        points_>>gamma_;   //read gamma value
        skip_spaces(points_,4);
        points_>>vel;      // read value of velocity
        cout<<temp_int<<". x: "<<x<<" y: "<<y<<" theta: "<<theta<<" gamma: "<<gamma_<<" vel: "<<vel<<"\n";
        getline(points_,line);
        assign_robot_state(x, y, theta, gamma_, false, 0, false, temp_state);   // assigns value to variable of type robot_state
        robot_path_.push_back(temp_state);     
        assign_robot_traj_state(x, y, theta, gamma_, vel, temp_state_tr);  // assigns value to variable of type state_xytheta_vel
        robot_trajectory_.push_back(temp_state_tr);
        temp[0] = x;
        temp[1] = y;
        path_output_.push_back(temp);
    }
    robot_path_.pop_back();
    robot_trajectory_.pop_back();
    path_output_.pop_back();
    robot_path_[robot_path_.size() - 1].pallet_state = 2;     // assigning final palllet and goal states
    robot_path_[robot_path_.size() - 1].pallet_up = 1;        // to robot_path. It has been hardcoded here
    robot_path_[robot_path_.size() - 1].goal = 1;             // and not been taken from the file. shall be modified for use with file only
}

// assigns x,y,theta,gamma,vel,goal,forkstate values to variable of type robot_state
void robot_nav::assign_robot_state(double x, double y, double theta, double gamma, bool pallet_up, int pallet_st, bool goal, robot_state& temp_state)
{
    temp_state.x = x;
    temp_state.y = y;
    temp_state.theta = theta;
    temp_state.gamma = gamma;
    temp_state.pallet_state = pallet_st;
    temp_state.pallet_up = pallet_up;
    temp_state.goal = goal;
}

// assigns x,y,theta,vel values to variable of type state_xytheta_vel
void robot_nav::assign_robot_traj_state(double x, double y, double theta, double gamma, double vel, state_xytheta_vel& temp_state)
{
    temp_state.x = x;
    temp_state.y = y;
    temp_state.theta = theta;
    temp_state.gamma = gamma;
    temp_state.vel = vel;
}

// this function is used for reading points from a file. called from read_points_from_file.
void robot_nav::skip_spaces(ifstream& file_stream, int skip_value)
{
    char temp_char;
    for (int i = 0; i < skip_value; i++)
    {
        file_stream>>temp_char;
    }
}

