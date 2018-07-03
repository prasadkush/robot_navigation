#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#define PI 3.14159

#include <string>

using namespace std;

struct robot_state
{
	double x, y, z, theta, gamma, time, vel, velx, velydd;   //gamma is the steer angle;
	bool pallet_up, pallet_down, goal;
        int pallet_state;
	robot_state();
	robot_state(double, double);
	robot_state(double, double, double);
	robot_state(double x_, double y_, double theta_, bool pallet_up_, bool pallet_down_);
        robot_state(double x_, double y_, double theta_, int pallet_state);
};

struct segment
{
	double start_x, start_y, start_theta, start_gamma;
	double final_x, final_y, final_theta, final_gamma;
	char seg_ind;
	bool final_pallet_up;      // state of pallet at final point of the segment
	int final_pallet_state;
    bool goal;
    bool pallet;
};

struct seg_indices
{
	int start_ind;
	int final_ind;
	char seg_ind;
};

struct traj_info
{
	string index;
	int no_of_points, total_points;
};

robot_state::robot_state()
{
	x = 0.0;
	y = 0.0;
	z = 0.0;
	theta = 0.0;
	gamma = 0.0;
	time = 0.0;
	vel = 0.0;
	goal = 0;
}

robot_state::robot_state(double x_, double y_)
{
	x = x_;
	y = y_;
	z = 0.0;
}

robot_state::robot_state(double x_, double y_, double theta_)
{
	x = x_;
	y = y_;
	z = 0.0;
	theta = theta_;
}

robot_state::robot_state(double x_, double y_, double theta_, bool pallet_up_, bool pallet_down_)
{
	x = x_;
	y = y_;
	theta = theta_;
	pallet_up = pallet_up_;
	pallet_down = pallet_down_;
}

struct state_xytheta_vel
{
	double x;
	double y;
	double theta;
	double vel;
	double gamma;
	bool pallet_up;
	bool pallet_down;
        int pallet_state;
};

struct robot_dimensions
{
	double wheel_base;
};

class robot_information
{
public:

	robot_information();

	robot_information(double base);

    robot_dimensions input_dimensions;

private:

    void initialize(double base);
};

#endif
