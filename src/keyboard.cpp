/*************
 * keyboard teleop control
 * this is the c++ version keyboard controller used in ardrone 2.0 control.
 * 
 * input: console keyboard key press
 * output: public to the topic /cmd_vel
 * refer to:
 * http://blog.csdn.net/hcx25909/article/details/9004617
 * http://wiki.ros.org/teleop_twist_keyboard
 * http://wiki.ros.org/turtlebot_teleop
 * modified by: Guojiangtao ChenMinhua
 * email: guojt@shanghaitech.edu.cn, chenmh@shanghaitech.edu.cn
 **************/

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

class SmartCarKeyboardTeleopNode
{
private:
	double walk_vel_;
	double run_vel_;
	double yaw_rate_;
	double yaw_rate_run_;
	double time_start_;
	double vel_base_;
	
	geometry_msgs::Twist cmdvel_;
	geometry_msgs::Twist velocity_msg_kb;
	ros::NodeHandle n_;
	ros::Publisher velocity_pub_;
	ros::Publisher takeoff_pub_;
	ros::Publisher land_pub_;
	ros::Publisher bool_vel_pub_;// for coperation between other cmd_vel controller
	std_msgs::Empty emp_msg;
	bool bool_vel;
	
public:
	SmartCarKeyboardTeleopNode()
	{
		//velocity_pub_= n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		velocity_pub_=n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);//velocity pub node		
		takeoff_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);//take off pub node
		land_pub_ = n_.advertise<std_msgs::Empty>("/ardrone/land",1);//land
		
		
		//init
		bool_vel = false;
		vel_base_ = 0.4;
		
		ros::NodeHandle n_private("~");
		n_private.param("walk_vel", walk_vel_, 1.0);
		n_private.param("run_vel", run_vel_, 1.0);
		n_private.param("yaw_rate", yaw_rate_, 1.0);
		n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
	}
	
	~SmartCarKeyboardTeleopNode() { }
	void keyboardLoop();
	
	void stopRobot()
	{
		cmdvel_.linear.x = 0.0;
		cmdvel_.linear.y = 0.0;
		cmdvel_.linear.z = 0.0;
		cmdvel_.angular.z = 0.0;
		velocity_pub_.publish(cmdvel_);
	}
	
	void takeoff()
	{
		time_start_=(double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec()< time_start_+3.0) { 
			takeoff_pub_.publish(emp_msg); 
			ROS_INFO("Taking off");
		}
	}
	void land()
	{
		time_start_=(double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec()< time_start_+4.0) { 
			land_pub_.publish(emp_msg);//ardrone land wrapper.
			ROS_INFO("Landing... ");
		}				
	}
	void pub_velocity_msg_kb(double x, double y, double z)
	{
		//for key board safe velocity_control
		bool x_bad = x>1 | x < -1;
		bool y_bad = y>1 | y < -1;
		bool z_bad = z>1 | z < -1;
		if(x_bad | y_bad | z_bad)
		{
			ROS_ERROR("pub_velocity_msg_kb: bad_x or bad_y or bad_z!");
		}
		else
		{			
			velocity_msg_kb.linear.x=x;
			velocity_msg_kb.linear.y=y;
			velocity_msg_kb.linear.z=z;
			velocity_msg_kb.angular.x=0.00;
			velocity_msg_kb.angular.y=0.00;
			velocity_msg_kb.angular.z=0.00;			
			time_start_=(double)ros::Time::now().toSec();
			while ((double)ros::Time::now().toSec()< time_start_+0.02)
			{ 
				velocity_pub_.publish(velocity_msg_kb);
			}	
		} 
	}	
};

SmartCarKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	SmartCarKeyboardTeleopNode tbk;
	
	boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));
	
	ros::spin();	
	t.interrupt();
	t.join();
	tbk.land();
	tcsetattr(kfd, TCSANOW, &cooked);
	
	return(0);
}

void SmartCarKeyboardTeleopNode::keyboardLoop()
{
	char c;
	double max_tv = walk_vel_;
	double max_rv = yaw_rate_;
	bool dirty = false;
	int speed = 0;
	int turn = 0;
	
	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	
// 	puts("Reading from keyboard");
// 	puts("Use WASD keys to control the robot");
// 	puts("Press Shift to move faster");
	puts("Reading from keyboard");
	puts("Use:");
	puts("i,k,j,l: for forward, backward, left, right, value = 0.3");
	puts("q,a: for up, down, value = 0.4");
	puts("blank space: for stop and hover. ");
	puts("h: for hover");
	puts("t,d: for takeoff and land");
	
	struct pollfd ufd;
	ufd.fd = kfd;
	ufd.events = POLLIN;
	
	for(;;)
	{
		boost::this_thread::interruption_point();
		
		// get the next event from the keyboard
		int num;
		
		if ((num = poll(&ufd, 1, 250)) < 0)
		{
			perror("poll():");
			return;
		}
		else if(num > 0)
		{
			if(read(kfd, &c, 1) < 0)
			{
				perror("read():");
				return;
			}
		}
		else
		{
			if (dirty == true)
			{
				stopRobot();
				dirty = false;
			}
			
			continue;
		}
		
		std::cout<<"get key:"<<c<<std::endl;
		
		switch(c)
		{
			case 'i'://forward
				pub_velocity_msg_kb( vel_base_, 0.0, 0.0);
				dirty = true;
				break;
			case 'k'://backward
				pub_velocity_msg_kb(-vel_base_, 0.0, 0.0);
				dirty = true;
				break;
			case 'j'://left
				pub_velocity_msg_kb(0.0,  vel_base_, 0.0);
				dirty = true;
				break;
			case 'l'://right
				pub_velocity_msg_kb(0.0, -vel_base_, 0.0);
				dirty = true;
				break;
			case 'q'://up
				pub_velocity_msg_kb(0.0, 0.0,  vel_base_);
				dirty = true;
				break;
			case 'a'://down
				pub_velocity_msg_kb(0.0, 0.0, -vel_base_);
				dirty = true;
				break;
			case ' '://stop
				stopRobot();
				break;
			case 'h'://hover
				stopRobot();
				break;
			case 'd'://land
				land();
				break;
			case 't'://take off
				takeoff();
				break;
				
			default:
				dirty = true;
				puts("default");
		}

	}
}