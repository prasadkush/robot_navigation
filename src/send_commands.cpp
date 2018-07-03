#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <send_commands/send_commands.h>

using namespace std;

void send_commands::ControlCallback(const robot_navigation::control_commands::ConstPtr& msg)
{
    std::stringstream abc, test;
    mtx.lock();
    speed = msg->val1;
    steer = msg->val2;
    cout<<"from robot_navigation speed: "<<speed<<" steer: "<<steer<<"\n";
    mtx.unlock();
}

void send_commands::readmsg()
{
		float speed_local, steer_local;
		char recvBuffer[256];
		n = read(newsockfd, recvBuffer,16);
		//printf("Recvd data %s\n", recvBuffer);
		sscanf(recvBuffer,"%f,%f\r", &speed_local, &steer_local);
		feedback_speed = speed_local;
		feedback_steer = steer_local;
		printf("Speed : %f Steering %f\n", feedback_speed, feedback_steer);
		robot_navigation::control_commands feedback_vals;
		feedback_vals.val1 = feedback_speed;
		feedback_vals.val2 = feedback_steer;
		pub.publish(feedback_vals);
}

void send_commands::sendThread()
{
	while(1)
	{
	    std::stringstream abc, test;
	    mtx.lock();
        //abc << std::setfill('0') << std::setw(4) << speed;
        //test << std::setfill('0') << std::setw(4) << steer;
        abc <<std::setprecision(4)<<speed;
        test<<std::setprecision(4)<<steer;
		mtx.unlock();
		bzero(buffer,256);
//		fgets(buffer,255,stdin);
		sprintf(buffer, "%s,%s\r", abc.str().c_str(), test.str().c_str());
//		sprintf(buffer, "%0.2f,%0.2f\r", speed, steer);
		printf("Sent Data %s\n", buffer);
		n = write(newsockfd,buffer,strlen(buffer));
		if (n < 0) 
        {
            cout<<"ERROR writing to socket\n";
        }
		readmsg();
		usleep(2000);
	}
}

send_commands::~send_commands()
{
    cout<<"\ninside destructor\n";
    //delete th1;
}

send_commands::send_commands(int portno_, int sockfd_, int newsockfd_)
{
    //send_commands::instance = this;
	send_commands::portno = portno_;
    send_commands::sockfd = sockfd_;
    newsockfd = newsockfd_;
	speed = 0.272478;
	steer = 0.00464708;
    //close_forklift = 0;
    signal(SIGTERM, send_commands::exit_gracefully);       //Ctrl + C handling
    signal(SIGHUP, send_commands::exit_gracefully);
    signal(SIGINT, send_commands::exit_gracefully);
    signal(SIGPIPE, send_commands::exit_gracefully);   
    pub = nh.advertise<robot_navigation::control_commands>("feedback_speed_steer", 1);
	sub = nh.subscribe("control_input", 1, &send_commands::ControlCallback, this);
}

bool send_commands::close_forklift = 0;

int send_commands::sockfd = 0;

int send_commands::newsockfd = 0;

int send_commands::portno = 12346;

//send_commands* send_commands::instance = NULL;

boost::thread* send_commands::th1 = NULL;
