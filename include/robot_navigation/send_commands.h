#include <ros/ros.h>
#include <robot_navigation/control_commands.h>
#include <boost/thread/thread.hpp>
//#include <mutex>

//using namespace std;

class send_commands
{
public:
	ros::NodeHandle nh;

	ros::Subscriber sub;

	int sockfd, newsockfd, portno;
    
    socklen_t clilen;
    
    char buffer[256];
    
    struct sockaddr_in serv_addr, cli_addr;
    
    int n;
    
    float speed, steer, feedback_speed, feedback_steer;

    send_commands(int portno_);

    void ControlCallback(const robot_navigation::control_commands::ConstPtr& msg);

    void sendThread();

    //std::mutex mtx;
};

