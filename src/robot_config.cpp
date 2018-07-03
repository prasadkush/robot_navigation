#include <robot_navigation/robot_config.h>

robot_information::robot_information()
{
	input_dimensions.wheel_base = 0.5
}

robot_information::robot_information(double base)
{
	initialize(base);
}

void robot_information::initialize(double base)
{
	input_dimensions.wheel_base = base;
}

