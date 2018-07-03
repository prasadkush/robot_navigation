#include <ros/ros.h>
#include <robot_navigation/planner.h>
#include <robot_navigation/robot_state_msg.h>
#include <time.h>
#include <iostream>
#include <fstream>

using namespace std;

planner::planner(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(name, costmap_ros);
}

planner::~planner()
{
	delete lattice_planner;
	delete visual;
}

planner::planner(string name, costmap_2d::Costmap2DROS* costmap_ros, ros::NodeHandle& n)
{
	initialize(name, costmap_ros, n);
}

void planner::initialize(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	costmap_ros_ = costmap_ros;
	//map_sub = n.subscribe("/map", 1, &planner::GridCallback, this);
	vis_name = "path_points"; 
	text_points = "path_output";
	robotW = 1.0;
	robotH = 1.0;
	offX = 0.0;
	offY = 0.0;
	offZ = 0.0;
	visual = new NavVisualizer(vis_name, robotW, robotH, offX, offY, offZ);
	path_planned = 0;
	map_received = 0;
	//map_sub = np.subscribe("/map", 1, &planner::GridCallback, this);
}

void planner::initialize(string name, costmap_2d::Costmap2DROS* costmap_ros, ros::NodeHandle& n)
{
	costmap_ros_ = costmap_ros;
	//map_sub = n.subscribe("/map", 1, &planner::GridCallback, this);
	vis_name = "path_points"; 
	text_points = "path_output";
	robotW = 1.0;
	robotH = 1.0;
	offX = 0.0;
	offY = 0.0;
	offZ = 0.0;
	visual = new NavVisualizer(vis_name, robotW, robotH, offX, offY, offZ);
	path_planned = 0;
	map_received = 0;
	sensor_offset = 1.1;
	wheel_base = 1.32;
	curve_factor = 1.3;
	points_count_ = 0;
	//np = n;
	map_sub = n.subscribe("/map", 1, &planner::GridCallback, this);
	start_pub = n.advertise<robot_navigation::robot_state_msg>("robot_start_state", 1);
    pallet_poses_pub = n.advertise<geometry_msgs::PoseArray>("/pallet_poses", 1);
}

void planner::GridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	cout<<"\ninside GridCallback\n";
	bool plan_path;
    streambuf *psbuf, *backup;
    ofstream filestr;
    filestr.open("test_planner.txt");
    backup = cout.rdbuf();
    psbuf = filestr.rdbuf();
    cout.rdbuf(psbuf);
	lattice_planner = new sbpl_lattice_planner::SBPLLatticePlanner("planner",costmap_ros_);   
	double rack_size_x_ = 1.5, rack_size_y_ = 6.0;
    robot_state rack_state;
    rack_state.x = 21.67;
    rack_state.y = 5.62;
    rack_state.theta = 0.0;
    geometry_msgs::Quaternion rack_orientation;
    rack_orientation.x = 0.0;
    rack_orientation.y = 0.0;
    rack_orientation.z = 0.0;
    rack_orientation.w = 1.0;
    clearCostmapWindows(rack_size_x_, rack_size_y_, rack_state, rack_orientation);
    lattice_planner_rack = new sbpl_lattice_planner::SBPLLatticePlanner("planner_for_picking",costmap_ros_);
    cout.rdbuf(backup);
    sleep(5);
	map_received = 1;
	cout<<"\nmap_received: "<<map_received<<"\n";
}

double planner::quaternion_to_angle(geometry_msgs::Quaternion orientation)  // quaternion to yaw angle conversion
{
	double theta;
	theta = 2 * atan2(orientation.z, orientation.w);
	return theta;
}

geometry_msgs::Quaternion planner::angle_to_quaternion(double theta)    // yaw angle to quaternion conversion
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


void planner::assign_pose_orientation(geometry_msgs::PoseStamped& point_, double x, double y, double z, double ox, double oy, double oz, double ow)
{
	point_.pose.position.x = x;
	point_.pose.position.y = y;
	point_.pose.position.z = z;
	point_.pose.orientation.x = ox;
	point_.pose.orientation.y = oy;
	point_.pose.orientation.z = oz;
	point_.pose.orientation.w = ow;
}

void planner::assign_pose_orientation(geometry_msgs::PoseStamped& point_, double x, double y, double z, double theta)
{
	point_.pose.position.x = x;
	point_.pose.position.y = y;
	point_.pose.position.z = z;
	point_.pose.orientation = angle_to_quaternion(theta);
	//point_.pose.orientation.x = ox;
	//point_.pose.orientation.y = oy;
	//point_.pose.orientation.z = oz;
	//point_.pose.orientation.w = ow;
}

void planner::assign_orientation(geometry_msgs::Quaternion& orient, double x_, double y_, double z_, double w_)
{
	orient.x = x_;
	orient.y = y_;
	orient.z = z_;
	orient.w = w_;
}

// This function assigns waypoint values to start and final waypooints on a segment
void planner::assign_seg_values(double start_x, double start_y, double start_theta, double final_x, double final_y, double final_theta, char seg_ind_, bool pallet_up, bool goal_, segment& temp)
{
	temp.start_x = start_x;
	temp.start_y = start_y;
	temp.start_theta = start_theta;
	temp.final_x = final_x;
	temp.final_y = final_y;
	temp.final_theta = final_theta;
	temp.seg_ind = seg_ind_;
	temp.final_pallet_up = pallet_up;
	temp.goal = goal_; 
}

// This function assigns waypoint values to start and final waypooints on a segment
// It assigns start, final sates, method of path generation ('p', 's', or 'm') final pallet_up, final pallet state, final goal state 
// It is currently being used to specify the waypoints
void planner::assign_seg_values(double start_x, double start_y, double start_theta, double final_x, double final_y, double final_theta, char seg_ind_, bool pallet_up, int final_pallet_state, bool goal_, segment& temp)
{
        temp.start_x = start_x;
        temp.start_y = start_y;
        temp.start_theta = start_theta;
        temp.final_x = final_x;
        temp.final_y = final_y;
        temp.final_theta = final_theta;
        temp.seg_ind = seg_ind_;
        temp.final_pallet_up = pallet_up;
        temp.final_pallet_state = final_pallet_state;
        temp.goal = goal_;
}

//In this function we specify waypoints our path shall have. The path is specified segment wise. For eg.
//assign_seg_values(6.77, 7.20, 3.14, 9.86, 7.20, 3.14, 'p', 0, 0, 0, temp_seg); path_segs.push_back(temp_seg);
// This assigns start, end configurations of the segment. Method of path generation. 'p' is for sbpl. 's' is for min
// jerk method and 'm' is when the robot state stays the same and only the fork state changes.
void planner::specify_start_goal()
{
    robot_navigation::robot_state_msg start_msg;
	segment temp_seg;

  // assign segg_values assigns start, end configuration of a segment, method of path generation, final pallet state of segment, goal state 
  // path_segs is a member variable which stores the segment information 


  // (start_x, start_y, start_theta, final_x, final_y, final_theta, method of path generation ('p', 's', 'm'), 
  // final pallet_up, final pallet state, final goal state, temporary variable for storing)
  assign_seg_values(6.77, 7.20, 3.14, 9.86, 7.20, 3.14, 'p', 0, 0, 0, temp_seg);    
        path_segs.push_back(temp_seg);             
 assign_seg_values(9.86, 7.20, 3.14, 11.86, 6.20, 1.57, 's', 0, 0, 0, temp_seg);
       path_segs.push_back(temp_seg);
 assign_seg_values(11.86, 6.20, 1.57, 11.86, 5.20, 1.57, 'p', 0, 0, 0, temp_seg);
       path_segs.push_back(temp_seg);
 //assign_seg_values(9.86, 7.20, 3.14, 11.86, 5.20, 1.57, 's', 0, 0, 0, temp_seg);
 //       path_segs.push_back(temp_seg);
  assign_seg_values(11.86, 5.20, 1.57, 11.86, 3.56, 1.57, 's', 1, 2, 1, temp_seg);
        path_segs.push_back(temp_seg); 


//assign_seg_values(6.0, 7.0, 1.57, 5.8, -1.0, 1.57, 's', 1, 2, 1, temp_seg);
//        path_segs.push_back(temp_seg);

        start_msg.x = path_segs[0].start_x;
        start_msg.y = path_segs[0].start_y;
        start_msg.theta = path_segs[0].start_theta;
        start_pub.publish(start_msg); 
}

// This function reads waypoints (they form segments in the path, consisti of init x,y,thet and final x,y,theta,goal,pallet_state,pallet_up) from file
void planner::read_waypoints_from_file(const string file_name)
{
    cout<<"inside read_waypoints_from_file\n";
    ifstream file_waypoints;
    file_waypoints.open(file_name.c_str());
    int point_no, pallet_state;
    char path_type, temp_char;
    double x_s, y_s, theta_s, x_f, y_f, theta_f;
    bool pallet_up, goal;
    int pallet_st;
    while(!file_waypoints.eof())
    {
        segment temp_seg;
        file_waypoints>>point_no;
        skip_spaces(file_waypoints,5);
        file_waypoints>>x_s;     // read init x value from file
        skip_spaces(file_waypoints,4);
        file_waypoints>>y_s;     // read init y value from file
        skip_spaces(file_waypoints,8);
        file_waypoints>>theta_s;  // read init theta value from file
        skip_spaces(file_waypoints,4);
        file_waypoints>>x_f;     // read final x value from file
        skip_spaces(file_waypoints,4);
        file_waypoints>>y_f;     // read final y value from file
        skip_spaces(file_waypoints,8);
        file_waypoints>>theta_f;  // read final theta value from file
        skip_spaces(file_waypoints,5);
        file_waypoints>>path_type;   // read the method path generation for this segment ('m' for no path generation, 'p' for sbpl, 's' for min jerk trajectory)
        skip_spaces(file_waypoints,10);  
        file_waypoints>>pallet_up;    // read value of pallet_up (0 or 1)
        skip_spaces(file_waypoints,10);
        file_waypoints>>pallet_st;    // read value of pallet_state 
        skip_spaces(file_waypoints,5);
        file_waypoints>>goal;       // read value of goal (0 or 1)
        //cout<<point_no<<". x_s: "<<x_s<<" y_s: "<<y_s<<" theta_s: "<<theta_s<<" x_f: "<<x_f<<" y_f: "<<y_f<<" theta_f: "
        //<<theta_f<<" type: "<<path_type<<" pallet_up: "<<pallet_up<<" pallet_st: "<<pallet_st<<" goal: "<<goal<<"\n";
        assign_seg_values(x_s, y_s, theta_s, x_f, y_f, theta_f, path_type, pallet_up, pallet_st, goal, temp_seg); // assign values to segment 
        path_segs.push_back(temp_seg);   
    }
    path_segs.pop_back();
       
}

//This function skips spaces while reading information from a file object
void planner::skip_spaces(ifstream& file_stream, int skip_value)
{
    char temp_char;
    for (int i = 0; i < skip_value; i++)
    {
        file_stream>>temp_char;
    }
}


//This function clears certain region in the costmap. May be needed to clear rack as an obstacle. If rack is an obstacle
// then forklift will not be able to keep pallet on it or pick from it.
void planner::clearCostmapWindows(double size_x, double size_y, robot_state rack_state, geometry_msgs::Quaternion orientation)
{

    std::vector<geometry_msgs::Point> clear_poly;

    double x = rack_state.x;
    double y = rack_state.y; 
    double theta = rack_state.theta;

    theta = quaternion_to_angle(orientation);

    geometry_msgs::Point pt;

    pt.x = x - (cos(theta)*size_x - sin(theta)*size_y)/ 2;
    pt.y = y - (sin(theta)*size_x + cos(theta)*size_y) / 2;
    clear_poly.push_back(pt);

    pt.x = x + (cos(theta)*size_x - sin(theta)*size_y) / 2;
    pt.y = y - (sin(theta)*size_x + cos(theta)*size_y) / 2;
    clear_poly.push_back(pt);

    pt.x = x + (cos(theta)*size_x - sin(theta)*size_y) / 2;
    pt.y = y + (sin(theta)*size_x + cos(theta)*size_y) / 2;
    clear_poly.push_back(pt);

    pt.x = x - (cos(theta)*size_x - sin(theta)*size_y) / 2;
    pt.y = y + (sin(theta)*size_x + cos(theta)*size_y) / 2;
    clear_poly.push_back(pt);

    costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
}


//This function assigns robot configuration for fisrt segment in the path when it is being formed by 'm' (i.e. only forkstate change)
void planner::assign_points_for_segment_first(vector<double>& temp_points, robot_state& temp_state, state_xytheta_vel& temp_xytheta_vel,
 vector<robot_state>& path_robot_, vector< vector<double> >& path_output_, vector<state_xytheta_vel>& trajectory_, 
        bool& pallet_up, int& pallet_state, vector<string>& next_seg_indices, int j)
{
        temp_points[0] = path_segs[j].start_x;
        temp_points[1] = path_segs[j].start_y;
        temp_state.x = path_segs[j].start_x;
        temp_state.y = path_segs[j].start_y;
        temp_state.theta = path_segs[j].start_theta;
        temp_state.gamma = 0.0;
        temp_state.pallet_up = path_segs[j].final_pallet_up;
        temp_state.pallet_state = path_segs[j].final_pallet_state;
        temp_state.goal = 1;
        path_robot_.push_back(temp_state);
        path_output_.push_back(temp_points);
       //path_output.push_back(temp_points);
        assign_state_xythetavel(path_segs[j].start_x, path_segs[j].start_y, path_segs[j].start_theta, 0.0, temp_xytheta_vel);
        trajectory_.push_back(temp_xytheta_vel);
        trajectory_[trajectory_.size()-1].pallet_up = path_segs[j].final_pallet_up;
        trajectory_[trajectory_.size()-1].pallet_state = path_segs[j].final_pallet_state;
        pallet_up = path_segs[j].final_pallet_up;
        pallet_state = path_segs[j].final_pallet_state;
        next_seg_indices.push_back("not computed in this condition");   
}

//This function assigns robot configuration for a segment in the path generated by 'm' (i.e. only forkstate change)
void planner::assign_points_for_segment(vector<double>& temp_points, robot_state& temp_state, state_xytheta_vel& temp_xytheta_vel, vector<robot_state>& path_robot_,
      vector< vector<double> >& path_output_, vector<state_xytheta_vel>& trajectory_, 
        bool& pallet_up, int& pallet_state, vector<string>& next_seg_indices, int j)
{
       int path_index = path_robot_.size() - 1;
       temp_points[0] = path_segs[j].start_x;
       temp_points[1] = path_segs[j].start_y;
       temp_state.x = path_robot_[path_index].x;
       temp_state.y = path_robot_[path_index].y;
       temp_state.theta = path_robot_[path_index].theta;
       temp_state.gamma = 0.0;
       temp_state.pallet_up = path_segs[j].final_pallet_up;
       temp_state.pallet_state = path_segs[j].final_pallet_state;
       temp_state.goal = 1;
       path_robot_.push_back(temp_state);
       path_output_.push_back(temp_points);
         //path_output.push_back(temp_points);
       assign_state_xythetavel(temp_state.x, temp_state.y, temp_state.theta, 0.0, temp_xytheta_vel);
       trajectory_.push_back(temp_xytheta_vel);
       trajectory_[trajectory_.size()-1].pallet_up = path_segs[j].final_pallet_up;
       trajectory_[trajectory_.size()-1].pallet_state = path_segs[j].final_pallet_state;
       pallet_up = path_segs[j].final_pallet_up;
       pallet_state = path_segs[j].final_pallet_state;
       next_seg_indices.push_back("not computed in this condition");   
}

//This function computes sbpl plan, and assigns it to path_robot, path_output for a particular segment
void planner::assign_sbpl_plan_for_segment(vector<double>& temp_points, robot_state& temp_state, vector< vector<double> >& path_output_, 
               vector<robot_state>& path_robot_, vector<robot_state>& path_, int seg_index, bool pallet_up, int pallet_state)
{
    streambuf *psbuf, *backup;
    ofstream filestr;
    filestr.open("test_planner2.txt");
    geometry_msgs::PoseStamped temp;
    vector<geometry_msgs::PoseStamped> points_(2, temp);
    double theta;
    assign_pose_orientation(points_[0], path_segs[seg_index].start_x, path_segs[seg_index].start_y, 0.0, path_segs[seg_index].start_theta);
    assign_pose_orientation(points_[1], path_segs[seg_index].final_x, path_segs[seg_index].final_y, 0.0, path_segs[seg_index].final_theta);
    backup = cout.rdbuf();
    psbuf = filestr.rdbuf();
    cout.rdbuf(psbuf);
    bool plan_path = lattice_planner_rack->makePlan(points_[0], points_[1], plan);
    cout.rdbuf(backup);
    for (int i = 0; i < plan.size(); i++)
    {
        temp_points[0] = plan[i].pose.position.x;
        temp_points[1] = plan[i].pose.position.y;
        temp_points[2] = plan[i].pose.position.z;
        theta = 2 * atan2(plan[i].pose.orientation.z, plan[i].pose.orientation.w);
        temp_state.x = plan[i].pose.position.x;
        temp_state.y = plan[i].pose.position.y;
        temp_state.theta = theta;
        temp_state.gamma = 0.0;
        temp_state.pallet_up = pallet_up;
        temp_state.pallet_state = pallet_state;
        temp_state.goal = 0;
        temp_points[2] = 0.0;
        path_output_.push_back(temp_points);
        path_robot_.push_back(temp_state);
        path_.push_back(temp_state);
    }
    filestr.close();
}

//This function assgins final state values to path_robot
void planner::assign_final_states_for_path(vector<robot_state>& path_robot_, vector<robot_state>& path_, int seg_index)
{
    path_robot_[path_robot_.size() - 1].pallet_up = path_segs[seg_index].final_pallet_up;  
    path_robot_[path_robot_.size() - 1].pallet_state = path_segs[seg_index].final_pallet_state;   
    path_robot_[path_robot_.size() - 1].goal = path_segs[seg_index].goal;
    path_[path_.size() - 1].pallet_state = path_segs[seg_index].final_pallet_state;
    path_[path_.size() - 1].pallet_up = path_segs[seg_index].final_pallet_up;
    path_[path_.size() - 1].goal = path_segs[seg_index].goal;
}

void planner::assign_final_states_for_path(vector<robot_state>& path_robot_, int seg_index)
{
    path_robot_[path_robot_.size() - 1].pallet_up = path_segs[seg_index].final_pallet_up;  
    path_robot_[path_robot_.size() - 1].pallet_state = path_segs[seg_index].final_pallet_state;   
    path_robot_[path_robot_.size() - 1].goal = path_segs[seg_index].goal;
}

void planner::assign_sbpl_plan(vector<geometry_msgs::PoseStamped>& temp_plan, vector<robot_state>& temp_path)
{
    double theta;
    robot_state temp_state;
    for (int k = 0; k < temp_plan.size(); k++)
    { 
        theta = 2 * atan2(temp_plan[k].pose.orientation.z, plan[k].pose.orientation.w);
        temp_state.x = temp_plan[k].pose.position.x;
        temp_state.y = temp_plan[k].pose.position.y;
        temp_state.theta = theta;
        temp_state.gamma = 0.0;
        temp_path.push_back(temp_state);
        
    }
}

//This function computes index of next segment and assigns them to next_seg_indices
void planner::assign_index_of_next_seg(vector<traj_info>& next_segments, vector<string>& next_seg_indices)
{
    if (next_segments.size() > 0)
    {
        next_seg_indices.push_back(next_segments[0].index);
    }
    else
    {
        next_seg_indices.push_back("none");
    }   
}


//This function computes final vel of a segment given the next segment waypoints
//Using the next segment waypoints (in path_seg), we compute the path (using sbpl) for next segment waypoints
//From the computed path, we compute the segment indices ("f", "fl", etc).
//From current segment information and next segment information, we compute the final velocity for this segment
void planner::compute_final_vel_given_nextseg(double& final_vel, vector<double>& temp_, vector<string>& next_seg_indices, int seg_index,
      vector<traj_info>& path_segments, vector<traj_info>& next_segments, double turning_vel,
    double straight_vel, double precision_vel)
{
    streambuf *psbuf, *backup;
    ofstream filestr;
    backup = cout.rdbuf();
    psbuf = filestr.rdbuf();
    filestr.open("test_planner2.txt");
    geometry_msgs::PoseStamped temp;
    vector<geometry_msgs::PoseStamped> points_(2, temp);
    vector<robot_state> temp_path;
    // check if this is the last segment (pair of waypoints along the path)
    if (seg_index == path_segs.size() - 1)   
    {
        final_vel = 0.0;   
        temp_[1] = final_vel;
        next_seg_indices.push_back("none");
    }
    else
    {   
        // assign start, final state of next segment to points_[0] and points_[1] respectively
        assign_pose_orientation(points_[0], path_segs[seg_index+1].start_x, path_segs[seg_index+1].start_y, 0.0, path_segs[seg_index+1].start_theta);
        assign_pose_orientation(points_[1], path_segs[seg_index+1].final_x, path_segs[seg_index+1].final_y, 0.0, path_segs[seg_index+1].final_theta);
        vector<geometry_msgs::PoseStamped> temp_plan;
        cout.rdbuf(psbuf);
        // Compute plan between start and final state of next segment
        bool plan_path = lattice_planner->makePlan(points_[0], points_[1], temp_plan);
        cout.rdbuf(backup);
        assign_sbpl_plan(temp_plan, temp_path);
        // Compute segment information ("f", "fr", "fl", etc) for next segment
        compute_trajectory_indices(temp_path, next_segments);
        // Compute final velocity of current segment using current segment and next segment information
        final_vel = compute_final_vel(path_segments, next_segments, seg_index, turning_vel, straight_vel, precision_vel);     
        assign_index_of_next_seg(next_segments, next_seg_indices); 
        temp_[1] = final_vel;    
    }
    filestr.close();
}

//this function assigns final pallet states to the computed trajectory
void planner::assign_final_states_for_trajectory(vector<state_xytheta_vel>& trajectory_, int seg_index)
{
   trajectory_[trajectory_.size()-1].pallet_up = path_segs[seg_index].final_pallet_up;   
   trajectory_[trajectory_.size()-1].pallet_state = path_segs[seg_index].final_pallet_state;
}

//This funciton computes final velocity for a smooth trajectory given next segment indices, and straight, turning, precision velocities
void planner::compute_final_vel_smoothtraj_given_nextseg(bool pallet_up, int pallet_state, double& final_vel, double& seg_vel,
     vector<traj_info>& prev_segments, vector<traj_info>& next_segments, int seg_index, string& index_traj,  
     double turning_vel, double straight_vel, double precision_vel)
{
    streambuf *psbuf, *backup;
    ofstream filestr;
    backup = cout.rdbuf();
    psbuf = filestr.rdbuf();
    filestr.open("test_planner3.txt");
    if (seg_index == path_segs.size() - 1 || path_segs[seg_index].final_pallet_up != pallet_up || path_segs[seg_index].final_pallet_state != pallet_state)
    {
        final_vel = compute_final_vel_for_smooth_traj(seg_vel, index_traj, prev_segments, next_segments, seg_index, turning_vel, straight_vel, precision_vel);
        final_vel = 0.0;
        seg_vel = 0.1*sign(seg_vel);
  
    }
    else
    {
        cout.rdbuf(psbuf);
        plan_path_compute_indices(path_segs[seg_index+1].start_x, path_segs[seg_index+1].start_y, path_segs[seg_index+1].start_theta, 
        path_segs[seg_index+1].final_x, path_segs[seg_index+1].final_y, path_segs[seg_index+1].final_theta, next_segments);     
        cout.rdbuf(backup);
        final_vel = compute_final_vel_for_smooth_traj(seg_vel, index_traj, prev_segments, next_segments, seg_index, turning_vel, straight_vel, precision_vel);  
    }
    filestr.close();
}


//This function computes path (x, y, theta, forkstate, goal) and trajectory (x, y, theta, gamma, vel) for the robot
void planner::make_plan_and_trajectory(vector< vector<double> >& path_output_, vector<robot_state>& path_robot_,
	vector<state_xytheta_vel>& trajectory_, double turning_vel, double straight_vel, double precision_vel)
{
	streambuf *psbuf, *backup;
	ofstream filestr;
	filestr.open("test_planner.txt");
    backup = cout.rdbuf();
    psbuf = filestr.rdbuf();
    state_xytheta_vel temp_xytheta_vel;
	cout<<"\ninside make_plan_and_trajectory\n";
	cout<<"\nmap_received: "<<map_received<<"\n";
	bool plan_path, pallet_up = 0, pallet_ = 0, goal = 0;
    int pallet_state = 0, i, j;
	double theta, init_vel, seg_vel, final_vel;
	vector<double> temp_points(3,0.0), temp_(2,0.0);
	vector< vector<double> > init_final_vels;
	vector<string> next_seg_indices;
	robot_state temp_state;
	vector<robot_state> path_;
	vector<traj_info> path_segments, prev_segments, next_segments;
	while (1)
	{
		if (path_planned == 0 && map_received == 1)
		{
			cout<<"\nbefore makePlan\n";
			for (j = 0; j < path_segs.size(); j++)
			{
               // condition for forkstate change (indicated by 'm') or when start and final configurations of the segment are same
               if (path_segs[j].seg_ind == 'm' || (path_segs[j].start_x == path_segs[j].final_x && path_segs[j].start_y == path_segs[j].final_y))
               {  
                  if (path_robot_.size() ==  0)
                  { // assign x,y,theta,goal,forkstate values to path_robot_, path_output_, trajectory_
                    assign_points_for_segment_first(temp_points, temp_state, temp_xytheta_vel, path_robot_, path_output_, trajectory_, 
                                                      pallet_up, pallet_state, next_seg_indices, j); }
                  else 
                  { // assign x,y,theta,goal,forkstate values to path_robot_, path_output_, trajectory_  
                    assign_points_for_segment(temp_points, temp_state, temp_xytheta_vel, path_robot_, path_output_, trajectory_, pallet_up, pallet_state, next_seg_indices, j); }
                  temp_[0] = init_vel;
                  temp_[1] = init_vel;
                }
				else if (path_segs[j].seg_ind == 'p')
				{
                    //compute sbpl plan and assign it to path_output_, path_robot_
                    assign_sbpl_plan_for_segment(temp_points, temp_state, path_output_, path_robot_, path_, j, pallet_up, pallet_state);
                    // assign final states stored in path_segs[j] to path_robot_
                    assign_final_states_for_path(path_robot_, path_, j);

					plan.clear();    // plan is a member variable used for temporarily storing paths received from sbpl function
					//compute path indices ("f", "fr", "fl", "b", "br", "bl") using input path
                    compute_trajectory_indices(path_, path_segments);
					temp_[0] = init_vel;
                    //compute final vel of segment, given initial vel of next segment
                    compute_final_vel_given_nextseg(final_vel, temp_, next_seg_indices, j, path_segments, next_segments, turning_vel, straight_vel, precision_vel);
                    // compute vel for each x,y,theta given straight, turning, precision vel   
					init_vel = compute_trajectory(trajectory_, path_, path_segments, pallet_state, pallet_up, j, turning_vel, straight_vel, precision_vel, init_vel, final_vel);
                    pallet_pose_push(j);   // push pallet pose 
					
                    //assign final states for trajectory
                    assign_final_states_for_trajectory(trajectory_, j);
				    
                    //update value of pallet_uo and pallet_state
                    pallet_up = path_segs[j].final_pallet_up;                    
                    pallet_state = path_segs[j].final_pallet_state;


                    // clear all the temporary variables used to store paths and path segments
					plan.clear();
					path_.clear();
					path_segments.clear();
					prev_segments.clear();
					next_segments.clear();
				}
				else 
				{
					cout<<"inside smooth trajectory generation\n";
					backup = cout.rdbuf();
					psbuf = filestr.rdbuf();
					cout.rdbuf(psbuf);    // used to redirect output to a file buffer and not the output buffer
                    // compute indices ("f", "fr", "fl", ...) of a segment given init and final x,y,theta of the segment 
					plan_path_compute_indices(path_segs[j].start_x, path_segs[j].start_y, path_segs[j].start_theta, path_segs[j].final_x,
						path_segs[j].final_y, path_segs[j].final_theta, prev_segments);
					cout.rdbuf(backup);
					string index_traj;
					//cout<<"init_vel: "<<init_vel<<"\n";
					temp_[0] = init_vel;

                    // compute final vel for smooth trajectory by computing path indices of next segment given next segment information
                    compute_final_vel_smoothtraj_given_nextseg(pallet_up, pallet_state, final_vel, seg_vel, prev_segments, 
                        next_segments, j, index_traj, turning_vel, straight_vel, precision_vel);

					temp_[1] = final_vel;
                    assign_index_of_next_seg(next_segments, next_seg_indices);

                    //compute smooth trajectory using minimum jerk given init vel, final vel, and trajecctory_, path_robot_
					init_vel = compute_smooth_trajectory(trajectory_, path_robot_, pallet_up, pallet_state, path_output_, j, init_vel, final_vel, seg_vel, index_traj);
				
                    //assign final pallet states of the current segment to trajectory_
                    assign_final_states_for_trajectory(trajectory_, j);
                    assign_final_states_for_path(path_robot_, j);

					pallet_up = path_segs[j].final_pallet_up;
                    pallet_state = path_segs[j].final_pallet_state;
                    
                    pallet_pose_push(j);
					
                    //clear temporary variable prev_segments and next_segments
                    prev_segments.clear();
					next_segments.clear();
				}
				init_final_vels.push_back(temp_); 
			}
			path_planned = 1;
		}
		if (path_planned == 1)
		{
			cout<<"\nbefore break\n";
			break;
		}
	}
	filestr.close();
    pallet_poses_pub.publish(pallet_poses);
}

// This is for storing the pallet poses. i.e. poses where the pallet is supposed to be picked or dropped
void planner::pallet_pose_push(int j)
{
    if (j > 0 && (path_segs[j].seg_ind == 'p' || path_segs[j].seg_ind == 's'))
    {
        if (path_segs[j].final_pallet_state != path_segs[j-1].final_pallet_state)
        {
            geometry_msgs::Pose temp_pose;
            temp_pose.position.x = path_segs[j].final_x;
            temp_pose.position.y = path_segs[j].final_y;
            temp_pose.position.z = 0.0;
            temp_pose.orientation = angle_to_quaternion(path_segs[j].final_theta);
            pallet_poses.poses.push_back(temp_pose);
        }
    }
}
// This function computes final velocity for minimum jerk trajectory given prev_segment and next_segment
// It compares the values of last segment of current path and first segment of next path
double planner::compute_final_vel_for_smooth_traj(double& seg_vel, string& index_traj, vector<traj_info>& prev_segments, vector<traj_info>& next_segments, 
	int j, double turning_vel, double straight_vel, double precision_vel)
{
	index_traj = "f";
	for (int i = 0; i < prev_segments.size(); i++)
	{   
		if (prev_segments[i].index == "fr" || prev_segments[i].index == "fl")
		{
			index_traj = "fr";
		}
		else if (prev_segments[i].index == "br" || prev_segments[i].index == "bl")
		{
			index_traj = "br";
		}
		else if (prev_segments[i].index == "f" && (index_traj != "fr" && index_traj != "br"))
		{
			index_traj = "f";
		}
		else if (prev_segments[i].index == "b" && (index_traj != "fr" && index_traj != "br"))
		{
			index_traj = "b";
		}
	}

	seg_vel = compute_vel_of_seg(turning_vel, straight_vel, precision_vel, index_traj);
	if (j != path_segs.size() - 1)
	{
	   if (j == path_segs.size() - 2 || path_segs[j+1].final_pallet_up != path_segs[j].final_pallet_up)
	   {
		   return precision_vel*sign(seg_vel);
	   }
	}
	if (next_segments.size() == 0)
	{
		return 0.0;
	}
	if (compute_vel_of_seg(turning_vel, straight_vel, precision_vel, prev_segments[prev_segments.size()-1].index)*
		compute_vel_of_seg(turning_vel, straight_vel, precision_vel, next_segments[0].index) < 0)
	{
		return 0.0;
	}
	else 
	{
		return compute_vel_of_seg(turning_vel, straight_vel, precision_vel, index_traj);
	}
	return 0.0;
}

// It compares the values of last segment of current path and first segment of next path
double planner::compute_seg_vel_replan(string& index_traj, vector<traj_info>& prev_segments, double turning_vel, double straight_vel, double precision_vel)
{
    double seg_vel;
    index_traj = "f";
    for (int i = 0; i < prev_segments.size(); i++)
    {   
        if (prev_segments[i].index == "fr" || prev_segments[i].index == "fl")
        {
            index_traj = "fr";
        }
        else if (prev_segments[i].index == "br" || prev_segments[i].index == "bl")
        {
            index_traj = "br";
        }
        else if (prev_segments[i].index == "f" && (index_traj != "fr" && index_traj != "br"))
        {
            index_traj = "f";
        }
        else if (prev_segments[i].index == "b" && (index_traj != "fr" && index_traj != "br"))
        {
            index_traj = "b";
        }
    }

    seg_vel = compute_vel_of_seg(turning_vel, straight_vel, precision_vel, index_traj);
    return seg_vel;
}


// This function computes indices of path segments. Given start, end configurations, it plans 
//a path first from start to end configuration. It uses the path to compute the indices and stores them in input_seg
void planner::plan_path_compute_indices(double start_x, double start_y, double start_theta, double final_x,
	double final_y, double final_theta, vector<traj_info>& input_seg)
{              
    streambuf *psbuf, *backup;
    ofstream filestr;
    filestr.open("test_planner2.txt");
    backup = cout.rdbuf();
    psbuf = filestr.rdbuf();
	geometry_msgs::PoseStamped temp;
	vector<geometry_msgs::PoseStamped> points_(2, temp);
	assign_pose_orientation(points_[0], start_x, start_y, 0.0, start_theta);
	assign_pose_orientation(points_[1], final_x, final_y, 0.0, final_theta);
	vector<geometry_msgs::PoseStamped> temp_plan;
	robot_state temp_state;
	double theta;
	vector<robot_state> temp_path;
    cout.rdbuf(psbuf);
	bool plan_path_ = lattice_planner_rack->makePlan(points_[0], points_[1], temp_plan);
    cout.rdbuf(backup);
        for (int k = 0; k < temp_plan.size(); k++)
	{ 
		theta = 2 * atan2(temp_plan[k].pose.orientation.z, temp_plan[k].pose.orientation.w);
		temp_state.x = temp_plan[k].pose.position.x;
		temp_state.y = temp_plan[k].pose.position.y;
		temp_state.theta = theta;
		temp_state.gamma = 0.0;
		temp_path.push_back(temp_state);
	}
	compute_trajectory_indices(temp_path, input_seg);
}

//computes the difference between two input angles
double planner::compute_theta_difference(double theta1, double theta2)
{
    double theta_diff = 0.0;
    if (theta1 <= theta2)
    {
        if (abs(theta1 - theta2) <= abs(theta1 - (theta2 - 2*PI)))
        { theta_diff = theta1 - theta2; }
        else
        {  theta_diff = theta1 - (theta2 - 2*PI); }
    }
    else if (theta1 > theta2)
    {
       if (abs(theta1 - theta2) <= abs(theta2 - (theta1 - 2*PI)))
       {  theta_diff = theta1 - theta2;  }
       else
       {  theta_diff = (theta1 - 2*PI) - theta2;  }
    }
    return theta_diff;
}


// This function computes minimum jerk trajectory given initial and final position and speeds 
// First traversal time for the trajectory is computed, then initial velocity is computed based on some checks
// Trajectory parameters for x, y, theta seperately are computed after that. 
// Using trajectory parameters, x, y, theta, velocity values at fixed time intervals are computed and stored
// The computed x, y, theta, velocity values are then stored in path_output_, path_robot_, and trajectory_
double planner::compute_smooth_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_robot_, bool pallet_up_, int pallet_state,
	vector< vector<double> >& path_output_, int j, double init_vel, double final_vel, double seg_vel, string index_traj)
{                         
    double wheel_base = 1.1;
	robot_state temp_state, state_diff_vec;
	state_xytheta_vel temp_xytheta_vel;
	vector<double> temp_points(3,0.0);
	VectorXd a(6), b(6), c(6);
	double T = 10.0, incr = 0.05, traj_init_vel;
	if (index_traj == "fr" || index_traj == "fl" || index_traj == "br" || index_traj == "bl")
	{
        // Compute traversal time for turning trajectory
		T = 1.4*distance(path_segs[j].start_x,path_segs[j].start_y,path_segs[j].final_x,path_segs[j].final_y)/abs(seg_vel);
	}
	else
	{
        // Compute traversal time for straight trajectory
		T = distance(path_segs[j].start_x,path_segs[j].start_y,path_segs[j].final_x,path_segs[j].final_y)/abs(seg_vel);
	}
	cout<<"index traj of smooth trajectory: "<<index_traj<<"\n";

    // Computation of initial velocity for the trajectory
	if (path_segs[j].final_pallet_state == pallet_state)
	{
		traj_init_vel = 0.3;
	}
	else
	{
		traj_init_vel = 0.1;
	}
        if (init_vel != 0 || j != 0)
        {
	if (index_traj == "fr" || index_traj == "fl" || index_traj =="f")
	{ 
		init_vel = max(traj_init_vel, init_vel);
	}
	else 
	{
		init_vel = min(-1*traj_init_vel, init_vel);
	}
        }
    //Computation of initial velocity of the trajectory done
    // Compute trajectory parameters for x
	a = compute_min_jerk(path_segs[j].start_x, path_segs[j].final_x, init_vel*cos(path_segs[j].start_theta), 
		final_vel*cos(path_segs[j].final_theta), 0.0, 0.0, T);
    // Compute trajectory parameters for y
	b = compute_min_jerk(path_segs[j].start_y, path_segs[j].final_y, init_vel*sin(path_segs[j].start_theta), 
		final_vel*sin(path_segs[j].final_theta), 0.0, 0.0, T);
    // Compute trajectory parameters for theta
	c = compute_min_jerk(path_segs[j].start_theta, path_segs[j].final_theta, 0.0, 0.0, 0.0, 0.0, T);
	vector<robot_state> path_min_jerk;
    // Compute path at discrete time intervals of incr and store it in path_min_jerk
	compute_path_min_jerk(a,b,c,incr,T,path_min_jerk, init_vel);
	cout<<"smooth trajectory no of points: "<<path_min_jerk.size()<<"\n";
	double dir_x, dir_y, prev_x = 0.0, prev_y = 0.0, prev_theta = 0.0, theta_dot = 0.0, theta_diff = 0.0;

    // After computing and storing trajectory values in path_min_jerk, this snippet of code stores the values in 
    // path_output_, path_robot_, and trajectory_
    for (int i = 0; i < path_min_jerk.size(); i++)
	{
		temp_points[0] = path_min_jerk[i].x;
		temp_points[1] = path_min_jerk[i].y;
		temp_points[2] = 0.0;
		//path_output.push_back(temp_points);
		temp_state.x = path_min_jerk[i].x;
		temp_state.y = path_min_jerk[i].y;

        // Computation of rotational velocity omega
        if (i == 0)
		{ temp_state.theta = path_segs[j].start_theta; }
        else if (i > 0 && i < path_min_jerk.size() - 1)
        {   
            if (index_traj == "fr" || index_traj == "f" || index_traj == "fl")
            { dir_x = path_min_jerk[i+1].x - temp_state.x;
             dir_y = path_min_jerk[i+1].y - temp_state.y;
             temp_state.theta = atan2(dir_y, dir_x);
             theta_diff = compute_theta_difference(temp_state.theta, prev_theta);
              theta_dot = theta_diff/incr; }
            else
            { dir_x = path_min_jerk[i+1].x - temp_state.x;
             dir_y = path_min_jerk[i+1].y - temp_state.y;
             temp_state.theta = atan2(dir_y, dir_x) + PI; 
             theta_diff = compute_theta_difference(temp_state.theta, prev_theta);
             theta_dot = theta_diff/incr; }

        }
        else
        { temp_state.theta = path_segs[j].final_theta;
          theta_diff = compute_theta_difference(temp_state.theta, prev_theta); 
          theta_dot = theta_diff/incr; }   
        // Computation of rotational velocity done
        
        // Computation of steer angle using rotational velocity 
		temp_state.gamma = atan(wheel_base*theta_dot/path_min_jerk[i].vel);
                                                                      
        if (abs(path_min_jerk[i].vel) <= 0.01)
        {    temp_state.gamma = 0.0;   }
        temp_state.pallet_up = pallet_up_;
        temp_state.pallet_state = pallet_state;
		temp_state.goal = 0;       
        prev_theta = temp_state.theta;
		path_output_.push_back(temp_points);      // Storing the points from path_min_jerk to path_output_
		path_robot_.push_back(temp_state);        // Storing the points from path_min_jerk to path_robot_
		temp_xytheta_vel.x = temp_state.x;
		temp_xytheta_vel.y = temp_state.y;
		temp_xytheta_vel.theta = temp_state.theta;
		temp_xytheta_vel.gamma = temp_state.gamma;
		temp_xytheta_vel.vel = path_min_jerk[i].vel;
        temp_xytheta_vel.pallet_state = pallet_state;
		trajectory_.push_back(temp_xytheta_vel);  // Storing the points from path_min_jerk to trajectory_
		points_count_ = points_count_ + 1;
	}
    // Storing values in path_output_, path_robot_ and trajectory_ from path_min_jerk done

    // Return the final velocity
	return temp_xytheta_vel.vel;
}

void planner::assign_path_min_jerk_points(vector<robot_state>& path_min_jerk, vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_robot_,
      vector< vector<double> >& path_output_, state_xytheta_vel& temp_xytheta_vel, bool pallet_up_, int pallet_state)
{
    vector<double> temp_points(3,0.0);
    robot_state temp_state;
    for (int i = 0; i < path_min_jerk.size(); i++)
    {
        temp_points[0] = path_min_jerk[i].x;
        temp_points[1] = path_min_jerk[i].y;
        temp_points[2] = 0.0;
        //path_output.push_back(temp_points);
        temp_state.x = path_min_jerk[i].x;
        temp_state.y = path_min_jerk[i].y;
        temp_state.theta = path_min_jerk[i].theta;
        temp_state.gamma = path_min_jerk[i].gamma;
        temp_state.pallet_up = pallet_up_;
                temp_state.pallet_state = pallet_state;
        temp_state.goal = 0;
        path_output_.push_back(temp_points);
        //path_output.push_back(temp_points);
        path_robot_.push_back(temp_state);
        temp_xytheta_vel.x = temp_state.x;
        temp_xytheta_vel.y = temp_state.y;
        temp_xytheta_vel.theta = temp_state.theta;
        temp_xytheta_vel.gamma = temp_state.gamma;
        temp_xytheta_vel.vel = path_min_jerk[i].vel;
                temp_xytheta_vel.pallet_state = pallet_state;
        trajectory_.push_back(temp_xytheta_vel);

        points_count_ = points_count_ + 1;
    }
}


// Computes traversal time for minimum jerk trajectory
void planner::compute_time_for_curve(double& T, string index_traj, const robot_state& start_, const robot_state& final_, double seg_vel)
{
    if (index_traj == "fr" || index_traj == "fl" || index_traj == "br" || index_traj == "bl")
    {
        T = 1.4*distance(start_.x, start_.y , final_.x, final_.y)/abs(seg_vel);
        //cout<<"Curve T: "<<T<<"\n";
    }
    else
    {
        T = distance(start_.x, start_.y, final_.x, final_.y)/abs(seg_vel);
        //cout<<"Straight T: "<<T<<"\n";
    }
}

// Computes initial velocity for minimum jerk trajectory
void planner::compute_init_vel_for_curve(bool pallet_point, string index_traj, double& init_vel)
{
    double traj_init_vel;
    if (!pallet_point)
    {
        traj_init_vel = 0.3;
    }
    else
    {
        traj_init_vel = 0.1;
    }
    if (init_vel != 0)
    {
    if (index_traj == "fr" || index_traj == "fl" || index_traj =="f")
    { 
        init_vel = max(traj_init_vel, init_vel);
    }
    else 
    {
        init_vel = min(-1*traj_init_vel, init_vel);
    }
    }
}

// Computation of parameters of minimum jerk trajectory, computation of x, y, theta, velocity values using the parameters and 
// storing them in path_output_, path_robot_, and trajectory_
// NOTE: This function is currently being called from controller_new.cpp code
double planner::compute_smooth_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_robot_, bool pallet_up_, int pallet_state,
    vector< vector<double> >& path_output_, const robot_state& start_, const robot_state& final_, bool pallet_point, double init_vel, double init_theta_vel, double final_vel,
   double final_theta_vel, double seg_vel, string index_traj)
{                         
    robot_state temp_state, state_diff_vec;
    state_xytheta_vel temp_xytheta_vel;
    vector<double> temp_points(3,0.0);
    VectorXd a(6), b(6), c(6);     // a, b, c will store trajectory parameters for x, y, theta
    // T is the trajectory traversal time. incr is the time step value at which we are computing values of x, y, theta and velocities
    double T = 10.0, incr = 0.05, traj_init_vel;

    
    // Compute traversal time for the robot
    compute_time_for_curve(T, index_traj, start_, final_, seg_vel);
    cout<<"index traj of smooth trajectory: "<<index_traj<<"\n";

    // Compute initial velocity for the trajectory
    compute_init_vel_for_curve(pallet_point, index_traj, init_vel);

    // Compute trajectory parameters for x
    a = compute_min_jerk(start_.x, final_.x, init_vel*cos(start_.theta), 
        final_vel*cos(final_.theta), 0.0, 0.0, T);
    // Compute trajectory parameters for y
    b = compute_min_jerk(start_.y, final_.y, init_vel*sin(start_.theta), 
        final_vel*sin(final_.theta), 0.0, 0.0, T);
    // Compute trajectory parameters for theta    
    c = compute_min_jerk(start_.theta, final_.theta, 0.0, 0.0, 0.0, 0.0, T);

    vector<robot_state> path_min_jerk;
    // Compute x, y, theta, velocity values at fixed time intervals and store the values in path_min_jerk
    compute_path_min_jerk(a,b,c,incr,T,path_min_jerk, init_vel);
    cout<<"smooth trajectory no of points: "<<path_min_jerk.size()<<"\n";
    
    // Assign values computed and stored in path_min_jerk to path_output_, path_robot_ and trajectory_
    assign_path_min_jerk_points(path_min_jerk, trajectory_, path_robot_, path_output_, temp_xytheta_vel, pallet_up_, pallet_state);

    // return the final velocity of the trajectory
    return temp_xytheta_vel.vel;
}

double planner::distance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
}

double planner::distance(robot_state state_1, robot_state state_2)
{
	return sqrt(pow((state_1.x - state_2.x),2) + pow((state_1.y - state_2.y),2));
}

// This function computes final_vel for sbpl path segment
double planner::compute_final_vel(vector<traj_info>& prev_segments, vector<traj_info>& next_segments, int j, double turning_vel,
	double straight_vel, double precision_vel)      
{
	if (compute_vel_of_seg(turning_vel, straight_vel, precision_vel, prev_segments[prev_segments.size()-1].index)*
		compute_vel_of_seg(turning_vel, straight_vel, precision_vel, next_segments[0].index) < 0)
	{
		return 0.0;
	}
	else if (j == path_segs.size() - 1)
	{
		return 0.0;
	}
	else if (j == path_segs.size() - 2 || path_segs[j+1].final_pallet_up != path_segs[j].final_pallet_up)
	{
		return sign(compute_vel_of_seg(turning_vel, straight_vel, precision_vel, next_segments[0].index))*precision_vel;
	}
	else 
	{
		return compute_vel_of_seg(turning_vel, straight_vel, precision_vel, next_segments[0].index);
	}
	return 0.0;
}



void planner::visualize()
{
	cout<<"\ninside visualize\n";	
	while(1)
	{
		//cout<<"\ninside visualize\n";
		if (path_planned == 1)
		{		
			visual->visualizePoints(path_output, 165, text_points);
			usleep(100000);
		}
	}
}

// This function computes parameters for minimum jerk trajectory given start, final conditions and the time for traversing the trajectory
// This function in solving a linear system of equations
VectorXd planner::compute_min_jerk(double x_s, double x_f, double v_s, double v_f, double a_s, double a_f, double T)
{
	MatrixXd m(6,6);
	VectorXd b(6); 
	VectorXd x(6);
	m(0,0) = 0; m(0,1) = 0; m(0,2) = 0; m(0,3) = 0; m(0,4) = 0; m(0,5) = 1;
	m(1,0) = pow(T,5); m(1,1) = pow(T,4); m(1,2) = pow(T,3); m(1,3) = pow(T,2); m(1,4) = T; m(1,5) = 1;
	m(2,0) = 0; m(2,1) = 0; m(2,2) = 0; m(2,3) = 0; m(2,4) = 1; m(2,5) = 0;
	m(3,0) = 5*pow(T,4); m(3,1) = 4*pow(T,3); m(3,2) = 3*pow(T,2); m(3,3) = 2*T; m(3,4) = 1; m(3,5) = 0;
	m(4,0) = 0; m(4,1) = 0; m(4,2) = 0; m(4,3) = 2; m(4,4) = 0; m(4,5) = 0;
	m(5,0) = 20*pow(T,3); m(5,1) = 12*pow(T,2); m(5,2) = 6*T; m(5,3) = 2; m(5,4) = 0; m(5,5) = 0;
	b(0) = x_s; b(1) = x_f; b(2) = v_s; b(3) = v_f; b(4) = a_s; b(5) = a_f;
	x = m.inverse()*b;
	return x;
}

// This function computes x, y, theta values given trajectory parameters and a specific time i
void planner::assign_robot_state(robot_state& temp, const VectorXd& x, const VectorXd& y, const VectorXd& theta, double i)
{
	double vel_x, vel_y, vel_theta, vel, gamma;
	temp.x = x(0)*pow(i,5) + x(1)*pow(i,4) + x(2)*pow(i,3) + x(3)*i*i + x(4)*i + x(5);
	temp.y = y(0)*pow(i,5) + y(1)*pow(i,4) + y(2)*pow(i,3) + y(3)*i*i + y(4)*i + y(5);
	temp.theta = theta(0)*pow(i,5) + theta(1)*pow(i,4) + theta(2)*pow(i,3) + theta(3)*pow(i,2) + theta(4)*i + theta(5); 
	vel_x = 5*x(0)*pow(i,4) + 4*x(1)*pow(i,3) + 3*x(2)*pow(i,2) + 2*x(3)*i + x(4);
	vel_y = 5*y(0)*pow(i,4) + 4*y(1)*pow(i,3) + 3*y(2)*pow(i,2) + 2*y(3)*i + y(4);
	vel = sqrt(pow(vel_x,2) + pow(vel_y,2));
	vel_theta = 5*theta(0)*pow(i,4) + 4*theta(1)*pow(i,3) + 3*theta(2)*pow(i,2) + 2*theta(3)*i + theta(4);
	gamma = atan2(wheel_base*vel_theta, vel);
	temp.gamma = gamma;	
	temp.vel = vel;
}

// Given trajectory parameters computed for x, y, theta. It computes the x, y, theta, velocity values at discrete intervals of time 
void planner::compute_path_min_jerk(const VectorXd& x, const VectorXd& y, const VectorXd& theta, const double incr, double T, vector<robot_state>& path_min_jerk, double init_vel)
{
	robot_state temp, state_diff_vec;
	path_min_jerk.clear();
	int ind_final, sign_vel, point_count;
	double final_theta, theta_dir, diff_magn;
	double i, vel_x, vel_y, acc_x, acc_y, vel_theta, gamma;
	point_count = 0;
	for (i = 0; i <= T; i+=incr)
	{
		assign_robot_state(temp, x, y, theta, i);
	// for computing sign of final vel
		sign_vel = 0.0;
		path_min_jerk.push_back(temp);
		if (point_count > 0)
		{
			state_diff_vec.x = path_min_jerk[point_count].x - path_min_jerk[point_count-1].x;
			state_diff_vec.y = path_min_jerk[point_count].y - path_min_jerk[point_count-1].y;
			final_theta = path_min_jerk[point_count].theta;
			diff_magn = distance(path_min_jerk[point_count], path_min_jerk[point_count-1]);
			theta_dir = atan2(state_diff_vec.y/diff_magn, state_diff_vec.x/diff_magn);
			if (theta_dir < 0)
			{
				theta_dir = theta_dir + 2*PI;
			}
			if (abs(final_theta - theta_dir) <= PI/2 || (abs(final_theta - theta_dir) >= 3*PI/2))
			{
				sign_vel = 1;
			}
			else
			{
				sign_vel = -1;
			}
		}
		else
		{
			sign_vel  = sign(init_vel);
		}
		path_min_jerk[point_count].vel = sign_vel*path_min_jerk[point_count].vel;
		point_count = point_count + 1;
	}
}

// Given trajectory parameters computed for x, y, theta. It computes the x, y, theta, velocity values at discrete intervals of time 
void planner::compute_path_min_jerk(const VectorXd& x, const VectorXd& y, const VectorXd& theta, const double incr, double T, vector<robot_state>& path_min_jerk)
{
	robot_state temp;
	path_min_jerk.clear();
	int ind_final, sign_vel;
	double final_theta, theta_dir, diff_magn;
	double i, vel_x, vel_y, acc_x, acc_y, vel_theta, gamma;
	for (i = 0; i < T; i+=incr)
	{

		assign_robot_state(temp, x, y, theta, i);
	// for computing sign of final vel
		sign_vel = 0.0;
		path_min_jerk.push_back(temp);
		assign_robot_state(temp, x, y, theta, T);
		path_min_jerk.push_back(temp);
	}
}


//This function takes input as a path containing a set of points (x,y,theta) and computes the segment
//indices (forward ("f"), backward ("b"), forward right ("fr"), forward left ("fl"), etc.) along the path
void planner::compute_trajectory_indices(vector<robot_state>& path_, vector<traj_info>& traj_segments_)
{
	double theta_diff, diff_magn, vec_dir_theta;
	string current_ind = "a";
	int num_points = 1, total_points = 0;
	first_assign = 1;
	robot_state state_diff_vec;
	traj_info curr_seg_info;
	traj_segments_.clear();
	assign_all_counts_zero();  // keeps count of the number of points in a segment
	for (int i = 1; i < path_.size(); i++)
	{
		num_points = num_points + 1;   // num_points stores number of points in the current segment
        // check for change in segment (for eg, from "f" to "fr" or from "fl" to "f" etc)
		check_segment_change(path_, traj_segments_, i, current_ind, num_points, total_points);
	}
	curr_seg_info.no_of_points = num_points;  // assigning number of points to final segment
	curr_seg_info.index = current_ind;      // assigning segment index ("f", "fl" etc)
	total_points = total_points + curr_seg_info.no_of_points;   // computing total points till now
	curr_seg_info.total_points = total_points;     // assigning total points on path 
	traj_segments_.push_back(curr_seg_info);
}

// This function computes velocities for a given path (x, y, theta values), straight, turning and precision velocities, desired final velocity
// For each segment in path, it computes segment velocity, final velocity, acceleration points (init points), same velocity points (mid points)
// and final points (decceleration points). Using these, it assigns x, y, theta, vel to each point in the path 
double planner::compute_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_, vector<traj_info>& path_segments, int pallet_state, bool pallet_up_, int k,
	double turning_vel, double straight_vel, double precision_vel, double init_vel, double final_vel_des)  
{
	double increment = 0.05, seg_vel = 0.0, vel = 0.0, sign_incr_init, sign_incr_final, final_vel;
	int index_count = 0, points_init, points_final, points_mid, j = 0, points_count = 0, prev_count = 0, num_points;
	state_xytheta_vel temp;
	vel = init_vel;
	for (int i = 0; i < path_segments.size(); i++)
	{ 
		num_points = path_segments[i].no_of_points;   // number of points in segment of the path
        // computation of seg vel of segment in path
		seg_vel = compute_vel_of_seg(turning_vel, straight_vel, precision_vel, path_segments[i].index);  
        // condition to check if segment is last segment or if there is a pallet pick/drop at end of segment
		if ((k == path_segs.size() - 1 && i == path_segments.size() - 1) || (path_segs[k].final_pallet_up != pallet_up_ && i == path_segments.size() - 1))
		{   
            // assign forward or reverse precision velocity to seg_vel if the above condition is met
			if (path_segments[i].index == "f" || path_segments[i].index == "fr" || path_segments[i].index == "fl")
			{ seg_vel = precision_vel;	}
			else if (path_segments[i].index == "b" || path_segments[i].index == "br" || path_segments[i].index == "bl")
			{ seg_vel = -1*precision_vel;	}
		}
		if (path_segments[i].index == "p") 
		{  seg_vel = precision_vel;   }
        // seg vel computed
    
        // computation of final vel of segment in path
		if (path_segments.size() > i+1)
		{
            // compute final vel of segment given next segment information    
			final_vel = compute_vel_of_seg(turning_vel, straight_vel, precision_vel, path_segments[i+1].index);
			// compare final vel with seg vel. assign it to zero if sign is opposite
            if (final_vel <= 0 && seg_vel > 0)      
			{   final_vel = 0.0;  	}
			else if (final_vel >=0 && seg_vel < 0)
			{    final_vel = 0.0;   }                                         
            // take minimum of final_vel, seg_vel as final_vel
		    else if (final_vel >= 0 && seg_vel >= 0)
			{	final_vel = min(seg_vel, final_vel); }
		    else if (final_vel < 0 && seg_vel < 0)
		    {  final_vel = max(seg_vel, final_vel); }
		}
		else
		{   final_vel = final_vel_des;	}
        // final vel of segment computed

        // Computation of initial, mid and end points in the segment
        // In the initial points robot accelerates or deccelerates, in the mid points robot stays at same speed
        // In the end points robot deccelerates or accelerates

		points_init = (int) nearbyint((abs(seg_vel - init_vel))/increment);
		points_final = (int) nearbyint((abs(seg_vel - final_vel))/increment);
		points_mid = max(0, num_points - points_init - points_final);
		sign_incr_init = sign(seg_vel, init_vel);   // sign_incr_init is +1 if seg_vel >= init_vel else -1
		sign_incr_final = sign(final_vel, seg_vel); // sign_incr_final is +1 if final_vel >= seg_vel else -1
		if (abs(init_vel) >= abs(seg_vel))   
		{
			points_init = (int) nearbyint((abs(init_vel - seg_vel))/increment);
			if (abs(final_vel) >= abs(seg_vel))  // if final vel is greater than seg vel, then number of final points shall be zero
			{  points_final = 0; 	}
			points_mid = max(0, num_points - points_init - points_final);   // recomputation of mid points
		}
		if (points_init >= num_points)         // if computed initial points is greater than number of points
		{
			sign_incr_init = sign(final_vel, init_vel);      
			points_init = min(num_points, points_init);     // limit init points to number of points
			points_mid = 0;         // set mid points as 0
            // condition to check if the vel will exceed final_vel
			if ((points_init*increment*sign_incr_init + init_vel > final_vel && sign_incr_init >= 0)   
				|| (points_init*increment*sign_incr_init + init_vel < final_vel && sign_incr_init < 0))
			{
                // If yes, then reduce the number of init points
				points_init = (int) nearbyint(((abs(final_vel - init_vel))/increment));
				points_mid = num_points - points_init;   // recompute mid points
			}
			points_final = 0;   
		}
        // If sum of init points and final points exceeds number of points
		else if (points_init < num_points && points_init + points_final > num_points)   
		{
			sign_incr_init = sign(final_vel, init_vel);
			if (init_vel >= final_vel)   
			{ 
                //recompute init points
				points_init = (int) nearbyint((abs(init_vel - final_vel))/increment);  
				points_init = min(num_points, points_init);   
				// recompute mid points
                points_mid = max(0, num_points - points_init);
				points_final = 0;
			} 
			else if (init_vel < final_vel)
			{
                // recompute init points
				points_init = (int) nearbyint((abs(final_vel - init_vel))/increment);
				points_init = min(num_points, points_init);
                // recompute mid points
				points_mid = max(0, num_points - points_init);
				points_final = 0;
			}
		}

        // Computation of initial, mid and final points done

        //computation and assignment of x, y, theta, vel values 
		
        // for loop for assigning x, y, theta, vel values to init points
        for (j = prev_count; j < prev_count + points_init; j++)     
		{
			vel = vel + sign_incr_init*increment;  // assigning vel values for init points
			if (j <= path_.size() - 1)
			{  assign_state_xythetavel(path_[j].x,path_[j].y,path_[j].theta,vel,temp);  }
			temp.gamma = compute_gamma_of_seg(path_segments[i].index);
                        temp.pallet_state = pallet_state;

			trajectory_.push_back(temp);
			points_count_ = points_count_ + 1;
			points_count = points_count + 1;   
		}
		prev_count = points_count;
        // for loop for assigning x, y, theta, vel values to mid points
		for (j = prev_count; j < prev_count + points_mid; j++)
		{
			if (j <= path_.size() - 1)
			{	assign_state_xythetavel(path_[j].x,path_[j].y,path_[j].theta,vel,temp); }   // assigning values for mid points
			temp.gamma = path_[j].gamma;
			temp.gamma = compute_gamma_of_seg(path_segments[i].index);
                        temp.pallet_state = pallet_state;
			trajectory_.push_back(temp);
			
			points_count_ = points_count_ + 1;
			points_count = points_count + 1;
		}
		prev_count = points_count;

        // for loop for assigning x, y, theta, vel values to final points
		for (j = prev_count; j < prev_count + points_final; j++)
		{
			vel = vel + sign_incr_final*increment;
			if (j <= path_.size() - 1)
			{   assign_state_xythetavel(path_[j].x,path_[j].y,path_[j].theta,vel,temp);   }   // assigning values for final points
			temp.gamma = path_[j].gamma;
			temp.gamma = compute_gamma_of_seg(path_segments[i].index);
                        temp.pallet_state = pallet_state;
			trajectory_.push_back(temp);
			points_count_ = points_count_ + 1;
			points_count = points_count + 1;
		}
		prev_count = points_count;
		init_vel = vel;
	}

    //return final velocity
	return vel;
}

// This function computes velocities for a given path (x, y, theta values), straight, turning and precision velocities, desired final velocity
// For each segment in path, it computes segment velocity, final velocity, acceleration points (init points), same velocity points (mid points)
// and final points (decceleration points). Using these, it assigns x, y, theta, vel to each point in the path 
// NOTE: This function is being used by controller_new.cpp
double planner::compute_trajectory(vector<state_xytheta_vel>& trajectory_, vector<robot_state>& path_, vector<traj_info>& path_segments, int pallet_state, bool pallet_up_,
    double turning_vel, double straight_vel, double precision_vel, double init_vel, double final_vel_des, bool pallet_segm)  
{
    double increment = 0.05, seg_vel = 0.0, vel = 0.0, sign_incr_init, sign_incr_final, final_vel;
    int index_count = 0, points_init, points_final, points_mid, j = 0, points_count = 0, prev_count = 0, num_points;
    state_xytheta_vel temp;
    vel = init_vel;
    for (int i = 0; i < path_segments.size(); i++)
    { 
        num_points = path_segments[i].no_of_points;
        seg_vel = compute_vel_of_seg(turning_vel, straight_vel, precision_vel, path_segments[i].index);
        if ( pallet_segm == 1 && i == path_segments.size() - 1)
        {
            if (path_segments[i].index == "f" || path_segments[i].index == "fr" || path_segments[i].index == "fl")
            {    seg_vel = precision_vel;  }
            else if (path_segments[i].index == "b" || path_segments[i].index == "br" || path_segments[i].index == "bl")
            {    seg_vel = -1*precision_vel;  }
        }
        if (path_segments[i].index == "p") 
        {    seg_vel = precision_vel;   }
        if (path_segments.size() > i+1)
        {    
            final_vel = compute_vel_of_seg(turning_vel, straight_vel, precision_vel, path_segments[i+1].index);
            if (final_vel <= 0 && seg_vel > 0)      
            {   final_vel = 0.0;  }
            else if (final_vel >=0 && seg_vel < 0)
            {   final_vel = 0.0;  }                                    
            else if (final_vel >= 0 && seg_vel >= 0)
            {   final_vel = min(seg_vel, final_vel);  }
            else if (final_vel < 0 && seg_vel < 0)
            { final_vel = max(seg_vel, final_vel);   }
        }
        else
        {   final_vel = final_vel_des;  }
        points_init = (int) nearbyint((abs(seg_vel - init_vel))/increment);
        points_final = (int) nearbyint((abs(seg_vel - final_vel))/increment);
        points_mid = max(0, num_points - points_init - points_final);
        sign_incr_init = sign(seg_vel, init_vel);
        sign_incr_final = sign(final_vel, seg_vel);
        if (abs(init_vel) >= abs(seg_vel))
        {
            points_init = (int) nearbyint((abs(init_vel - seg_vel))/increment);
            if (abs(final_vel) >= abs(seg_vel))
            {    points_final = 0;   }
            points_mid = max(0, num_points - points_init - points_final);
        }
        if (points_init >= num_points)
        {
            sign_incr_init = sign(final_vel, init_vel);
            points_init = min(num_points, points_init);
            points_mid = 0;
            if ((points_init*increment*sign_incr_init + init_vel > final_vel && sign_incr_init >= 0) 
                || (points_init*increment*sign_incr_init + init_vel < final_vel && sign_incr_init < 0))
            {
                points_init = (int) nearbyint(((abs(final_vel - init_vel))/increment));
                points_mid = num_points - points_init;
            }
            points_final = 0;
        }
        else if (points_init < num_points && points_init + points_final > num_points)
        {
            sign_incr_init = sign(final_vel, init_vel);
            if (init_vel >= final_vel)
            {
                points_init = (int) nearbyint((abs(init_vel - final_vel))/increment);
                points_init = min(num_points, points_init);
                points_mid = max(0, num_points - points_init);
                points_final = 0;
            } 
            else if (init_vel < final_vel)
            {
                points_init = (int) nearbyint((abs(final_vel - init_vel))/increment);
                points_init = min(num_points, points_init);
                points_mid = max(0, num_points - points_init);
                points_final = 0;
            }
        }

        for (j = prev_count; j < prev_count + points_init; j++)
        {
            vel = vel + sign_incr_init*increment;
            if (j <= path_.size() - 1)
            {   assign_state_xythetavel(path_[j].x,path_[j].y,path_[j].theta,vel,temp);   }
            temp.gamma = compute_gamma_of_seg(path_segments[i].index);
                        temp.pallet_state = pallet_state;

            trajectory_.push_back(temp);
            points_count_ = points_count_ + 1;
            points_count = points_count + 1;
        }
        prev_count = points_count;
        for (j = prev_count; j < prev_count + points_mid; j++)
        {
            if (j <= path_.size() - 1)
            {     assign_state_xythetavel(path_[j].x,path_[j].y,path_[j].theta,vel,temp);    }
            temp.gamma = path_[j].gamma;
            temp.gamma = compute_gamma_of_seg(path_segments[i].index);

                        temp.pallet_state = pallet_state;
            trajectory_.push_back(temp);
            points_count_ = points_count_ + 1;
            points_count = points_count + 1;
        }
        prev_count = points_count;
        for (j = prev_count; j < prev_count + points_final; j++)
        {
            vel = vel + sign_incr_final*increment;
            if (j <= path_.size() - 1)
            {     assign_state_xythetavel(path_[j].x,path_[j].y,path_[j].theta,vel,temp);   }
            temp.gamma = path_[j].gamma;
            temp.gamma = compute_gamma_of_seg(path_segments[i].index);
                        temp.pallet_state = pallet_state;
            trajectory_.push_back(temp);
            points_count = points_count + 1;
        }
        prev_count = points_count;
        init_vel = vel;
    }
    return vel;
}

double planner::compute_vel_of_seg(double turning_vel, double straight_vel, double precision_vel, string index)
{
	if (index == "f" || index == "b")
	{
		if (index == "f")
		{
			return straight_vel;
		}
		else if (index == "b")
		{
			return -1*straight_vel;
		}
	}
	else if (index == "fl" || index == "fr" || index == "bl" || index == "br")
	{
		if (index == "fl" || index == "fr")
		{
			return turning_vel;
		}
		else if (index == "bl" || index == "br")
		{
			return -1*turning_vel;
		}
	}
	else if (index == "p")
	{
		return precision_vel;
	}
	return 0.5;
}

double planner::compute_gamma_of_seg(string index)
{
	if (index == "f" || index == "b")
	{
		return 0.0;
	}
	else if (index == "fl") 
	{
		return 0.3;
	}
	else if (index == "fr") 
	{
		return -0.3;
	}
	else if (index == "bl")
	{
		return 0.3;
	}
	else if (index == "br")
	{
		return -0.3;
	}
	else if (index == "p")
	{
		return 0.3;
	}
	return 0.0;
}

// This function checks if there is a change in the path (eg from forward to forward right, or from forward to backward etc)
// It stores the corresponding indices in path_segment
void planner::check_segment_change(vector<robot_state>& path_, vector<traj_info>& path_segment, int i, string& current_ind, int& num_points, int& total_points)
{
	double theta_diff, diff_magn, vec_dir_theta, theta_curr;
	robot_state state_diff_vec;
	theta_curr = path_[i-1].theta;
	theta_diff = path_[i].theta - path_[i-1].theta;
	state_diff_vec.x = path_[i].x - path_[i-1].x;
	state_diff_vec.y = path_[i].y - path_[i-1].y;
	diff_magn = distance(path_[i], path_[i-1]);
	vec_dir_theta = atan2(state_diff_vec.y/diff_magn, state_diff_vec.x/diff_magn);
	if (vec_dir_theta < 0)
	{
		vec_dir_theta = vec_dir_theta + 2*PI;
	}
    // condition for segment changing to "f" (forward)
	if (theta_diff == 0.00 && (abs(path_[i-1].theta - vec_dir_theta) <= PI/2 || abs(path_[i-1].theta - vec_dir_theta) >= 3*PI/2) && current_ind != "f")
	{
		forward_count = forward_count + 1;
		if (forward_count > 3)
		{
		assign_segment_index(current_ind, path_segment, "f", num_points, i, total_points);  // add a segment and assing number of points to it
		if (i > 4)
		{
			num_points = 1;
            // initialize all counts (counting number of points in the segment) to zero
			assign_all_counts_zero();
		}
     	}
	}
    // condition for segment changing to "b" (reverse)
	else if (theta_diff == 0.00 && (abs(path_[i-1].theta - vec_dir_theta) > PI/2 && abs(path_[i-1].theta - vec_dir_theta) < 3*PI/2) && current_ind != "b")
	{
		backward_count = backward_count + 1;
		if (backward_count > 3)
		{
		assign_segment_index(current_ind, path_segment, "b", num_points, i, total_points); // add a segment and assing number of points to it
	    if (i > 4)
		{
			num_points = 1;
            // initialize all counts (counting number of points in the segment) to zero
			assign_all_counts_zero();
		}
	    }
	}
    // condition for segment changing to "fl" (forward left)
	else if (theta_diff > 0.00 && (abs(path_[i-1].theta - vec_dir_theta) <= PI/2 || abs(path_[i-1].theta - vec_dir_theta) >= 3*PI/2) && current_ind != "fl")
	{
		fl_count = fl_count + 1;
		if (fl_count > 3)
		{
		assign_segment_index(current_ind, path_segment, "fl", num_points, i, total_points);  // add a segment and assing number of points to it
		if (first_assign > 2)
		{
            // initialize all counts (counting number of points in the segment) to zero
            assign_all_counts_zero();
			num_points = 1;
		}
	    }

	}
    // condition for segment changing to "br" (reverse right)
	else if (theta_diff > 0.00 && (abs(path_[i-1].theta - vec_dir_theta) > PI/2 && abs(path_[i-1].theta - vec_dir_theta) < 3*PI/2)  && current_ind != "br")
	{
		br_count = br_count + 1;
		if (br_count > 3)
        {
		assign_segment_index(current_ind, path_segment, "br", num_points, i, total_points);  // add a segment and assing number of points to it
		if (first_assign > 2)
		{
            // initialize all counts (counting number of points in the segment) to zero
			assign_all_counts_zero();
			num_points = 1;
		}
	    }
	}
    // condition for segment changing to "fr" (forward right)
	else if (theta_diff < 0.00 && (abs(path_[i-1].theta - vec_dir_theta) <= PI/2 || abs(path_[i-1].theta - vec_dir_theta) >= 3*PI/2) && current_ind != "fr")
	{
		fr_count = fr_count + 1;
		if (fr_count > 3)
		{
		assign_segment_index(current_ind, path_segment, "fr", num_points, i, total_points);  // add a segment and assing number of points to it
		if (first_assign > 2)
		{
            // initialize all counts (counting number of points in the segment) to zero
			assign_all_counts_zero();
			num_points = 1;
		}
	    }
	}
    // condition for segment changing to "bl" (reverse left)
	else if (theta_diff < 0.00 && (abs(path_[i-1].theta - vec_dir_theta) > PI/2 && abs(path_[i-1].theta - vec_dir_theta) < 3*PI/2) && current_ind != "bl")
	{
		bl_count = bl_count + 1;
		if (bl_count > 3)
		{
		assign_segment_index(current_ind, path_segment, "bl", num_points, i, total_points);   // add a segment and assing number of points to it
		if (first_assign > 2)
		{
            // initialize all counts (counting number of points in the segment) to zero
			assign_all_counts_zero();
			num_points = 1;
		}
	    }
	}
}

void planner::assign_all_counts_zero()
{
	forward_count = 0;
	backward_count = 0;
	fl_count = 0;
	bl_count = 0;
	fr_count = 0;
	br_count = 0;
}

//This function assigns segment index (fr, fl, f, b etc) to a segment in the path
void planner::assign_segment_index(string& curr_ind, vector<traj_info>& path_segment, string ind, int num_points, int i, int& total_points)
{
	//if (num_points > 0 && i != 1)
	if (num_points > 0 && first_assign != 1)
	{
		traj_info curr_seg_info;
		curr_seg_info.no_of_points = num_points;
		//if (total_points == 0)
		//{
			curr_seg_info.no_of_points = num_points - 1;
		//}
		curr_seg_info.index = curr_ind;
		total_points = total_points + curr_seg_info.no_of_points;
		curr_seg_info.total_points = total_points;
		path_segment.push_back(curr_seg_info);
        first_assign = first_assign + 1;
	}
	curr_ind = ind;
	if (first_assign == 1)
	{
		first_assign = first_assign + 1;
	}
}

void planner::assign_state_xythetavel(double state_x, double state_y, double state_theta, double state_vel, state_xytheta_vel& state)
{
	state.x = state_x;
	state.y = state_y;
	state.theta = state_theta;
	state.vel = state_vel;
}

void planner::output_trajectory(vector<state_xytheta_vel>& trajectory_)
{
	int i;
	cout<<"\nTrajectory:\n";
	for (i = 0; i < trajectory_.size(); i++)
	{
		cout<<i+1<<". x: "<<trajectory_[i].x<<" y: "<<trajectory_[i].y<<" theta: "<<trajectory_[i].theta;
		cout<<" gamma: "<<trajectory_[i].gamma<<" vel: "<<trajectory_[i].vel<<" state: "<<trajectory_[i].pallet_state<<"\n";
	}
}

void planner::output_path(vector< vector<double> >& path_for_output)
{
	for (int i = 0; i < path_for_output.size(); i++)
	{
		cout<<i+1<<" x: "<<path_for_output[i][0]<<" y: "<<path_for_output[i][1]<<" theta: "<<path_for_output[i][2]<<"\n";
	}
}

inline int planner::sign(double x, double y)
{
	return (x >= y ? 1: -1);
}

inline int planner::sign(double x)
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

void planner::write_points_to_file()
{
	geometry_msgs::PoseStamped temp;
	vector<geometry_msgs::PoseStamped> points_(2, temp);
	int i, no_of_points;
	assign_pose_orientation_(points_[0], 9.82, -0.46, 0.00);
	assign_pose_orientation_(points_[1], 11.27, 2.60, 1.54);
	no_of_points = points_.size();
	int no_points_;
	string data;
	double temp1, x, y, theta, x_next, y_next, theta_next;
	ofstream outfile;
	outfile.open("/home/kush/catkin_ws/src/robot_navigation/kush.txt", ios::out);
	outfile<<no_of_points<<"\n";
	for (i = 0; i < points_.size(); i++)
	{
		cout<<i+1<<". x: "<<points_[i].pose.position.x<<" y: "<<points_[i].pose.position.y<<" theta: "<<points_[i].pose.position.z<<"\n";
		outfile<<i+1<<". x: "<<points_[i].pose.position.x<<" y: "<<points_[i].pose.position.y<<" theta: "<<points_[i].pose.position.z<<"\n";
	}
	outfile.close();
	segment temp_seg;
	ifstream infile;
	infile.open("/home/kush/catkin_ws/src/robot_navigation/kush.txt", ios::in);
	infile>>no_points_;
	infile>>data;
	infile>>data;
	infile>>x_next;
	infile>>data;
	infile>>y_next; 
	infile>>data;
	infile>>theta_next;
	x = x_next;
	y = y_next;
	theta = theta_next;
	cout<<1<<". x: "<<x_next<<" y: "<<y_next<<" theta: "<<theta_next<<"\n";
	for (i = 1; i < no_points_; i++)
	{
		infile>>data;
		infile>>data;
		infile>>x_next;
		infile>>data;
		infile>>y_next; 
		infile>>data;
		infile>>theta_next;
		assign_seg_values(x, y, theta, x_next, y_next, theta_next, 'p', 0, 0, temp_seg);
		path_segs.push_back(temp_seg);
		x = x_next;
		y = y_next;
		theta = theta_next;
		cout<<i+1<<". x: "<<x_next<<" y: "<<y_next<<" theta: "<<theta_next<<"\n";
	}
	infile.close();
	vector< vector<double> > path_output_;
	vector<robot_state> path_robot_;
	vector<state_xytheta_vel> trajectory_;
	outfile.open("/home/kush/catkin_ws/src/robot_navigation/points.txt", ios::out);
	make_plan_and_trajectory(path_output_, path_robot_, trajectory_, 0.3, 0.5, 0.1);
	no_of_points = path_robot_.size();
	outfile<<no_of_points<<"\n";
	for (i = 0; i < no_of_points; i++)
	{
		outfile<<i+1<<". x: "<<path_robot_[i].x<<" y: "<<path_robot_[i].y<<" theta: "<<path_robot_[i].theta<<"\n";
	}
	outfile.close();
}

void planner::assign_pose_orientation_(geometry_msgs::PoseStamped& point, double x_, double y_, double theta_)
{
	point.pose.position.x = x_;
	point.pose.position.y = y_;
	point.pose.position.z = theta_;
}  



