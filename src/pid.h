#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string>
#include <iostream>
#include <iomanip>


//ardrone autonomy navdata
#include <ardrone_autonomy/Navdata.h>



#include <drone_land_ar_pid/State.h>
#include <drone_land_ar_pid/PidParameterConfig.h>
#include <dynamic_reconfigure/server.h>
using namespace std;

//refer to http://vision.in.tum.de/teaching/ss2013/visnav2013,exercise3.
class PidController {
private:
	float error_old;
	float ierror;
	ros::Time t_old;

public:
	float c_proportional;
	float c_integral;
	float c_derivative;

	PidController() {
		c_proportional = c_integral = c_derivative = 0;
		error_old = 0;
		ierror = 0;
		t_old = ros::Time::now();
		reset();
	}

	float getCommand(const ros::Time& t, float error) {
		// derivative
		float derror = 0;
		ros::Duration diff = t - t_old;

		if (diff.toSec()) {
			derror = (error - error_old) / diff.toSec();
			error_old = error;
			return getCommand(t, error, derror);
		}
		return 0;
	}

	float getCommand(const ros::Time& t, float error, float derror) {
		ros::Duration diff = t - t_old;
		//implement PID control law
		ierror = ierror + diff.toSec() * error;	
		//ierror = std::min(std::max(ierror, -0.5f), 0.5f);// limit (-0.5,0.5)

		float u; // Control input
		u = c_proportional * error + c_derivative * derror
				+ c_integral * ierror;

		t_old = t;
		return u;
	}

	// resets the internal state
	void reset() {
	}

};


class ArdroneController {
private:
	ros::NodeHandle nh;
	ros::Publisher pid_pose_pub;
	ros::Publisher global_pose_pub;
	ros::Time time_pid;
	
	dynamic_reconfigure::Server<drone_land_ar_pid::PidParameterConfig> reconfigure_server;
	drone_land_ar_pid::PidParameterConfig current_cfg;
	
	geometry_msgs::Twist twist_msg;
	drone_land_ar_pid::State state;
	

	
	float goal_x, goal_y, goal_yaw;
  
  
public:
	
	PidController pid_x, pid_y, pid_yaw;
	
	ArdroneController(ros::NodeHandle nh): 
	nh(nh), reconfigure_server() {
			
			//pid_pose_pub = node.advertise<geometry_msgs::Twist>("/pid_pose",1);
			//global_pose_pub = node.advertise<geometry_msgs::Twist>("/global_pose",1);
			time_pid=ros::Time::now();
			reconfigure_server.setCallback(boost::bind(&ArdroneController::onConfig, this, _1, _2));
			twist_msg.linear.x = twist_msg.linear.y = twist_msg.linear.z = 0;
			twist_msg.angular.x = twist_msg.angular.y = twist_msg.angular.z = 0;
			//setGoalPose(0.04,-0.1,0);
		}
		ArdroneController(): reconfigure_server(){
			time_pid=ros::Time::now();
			
		}
		
		void init(ros::NodeHandle& node){
			nh = node;
			reconfigure_server.setCallback(boost::bind(&ArdroneController::onConfig, this, _1, _2));
			twist_msg.linear.x = twist_msg.linear.y = twist_msg.linear.z = 0;
			twist_msg.angular.x = twist_msg.angular.y = twist_msg.angular.z = 0;
		}
		
		void onConfig(drone_land_ar_pid::PidParameterConfig& cfg, uint32_t level)
		{
			current_cfg = cfg;
			setPidParameters(cfg);
		}
		//set  the PID parameters
		void setPidParameters(drone_land_ar_pid::PidParameterConfig& config)
		{
			pid_x.c_proportional = config.c_prop_trans_x;
			pid_x.c_integral     = config.c_int_trans_x;			
			pid_x.c_derivative    = config.c_deriv_trans_x;
			
			pid_y.c_proportional = config.c_prop_trans_y;
			pid_y.c_integral     = config.c_int_trans_y;
			pid_y.c_derivative   = config.c_deriv_trans_y;
			
			pid_yaw.c_proportional = config.c_prop_yaw;
			pid_yaw.c_integral     = config.c_int_yaw;
			pid_yaw.c_derivative   = config.c_deriv_yaw;
		}
		
		void setGoalPose(float x, float y, float yaw)
		{
			goal_x = x;
			goal_y = y;
			goal_yaw = yaw;
		}
		
		void setPose(drone_land_ar_pid::State pose_state, const ros::Time& t)
		{
			state = pose_state;
			time_pid = t;
		}
		
		void calculateControlCommand(const ros::Time& t)
		{
			
			tf::Pose marker_in_camera;
			tf::Pose camera_in_marker;
			tf::poseMsgToTF(state.pose, camera_in_marker);
			marker_in_camera = camera_in_marker.inverse();
			
			
			
			float e_x, e_y, e_yaw;
			e_x = marker_in_camera.getOrigin().x();
			e_y = marker_in_camera.getOrigin().y();
			//e_yaw = goal_yaw - state.yaw;
			// use this yaw to rotate commands from global to local frame
			float yaw = -(state.yaw + M_PI_2);
			
			float u_x = pid_x.getCommand(t, e_x);
			float u_y = pid_y.getCommand(t, e_y);
			
			//float u_x = pid_x.getCommand(t, e_x, -state.vx);
			//float u_y = pid_y.getCommand(t, e_y, -state.vy);
			
			ROS_INFO_STREAM("v_x,v_y:"<<state.vx<<","<<state.vy);
			ROS_INFO_STREAM("u_x,u_y:"<<u_x<<","<<u_y);
			
// 			twist_msg.linear.x = cos(yaw) * u_x - sin(yaw) * u_y;
// 			twist_msg.linear.y = sin(yaw) * u_x + cos(yaw) * u_y;
// 			twist_msg.linear.z = state.z;
			
			twist_msg.linear.x = u_y;
			twist_msg.linear.y = -u_x;
			twist_msg.linear.z =0.;
			
// 			float u_yaw = pid_yaw.getCommand(t, e_yaw);
			
			// normalize angular control command
			//twist_msg.angular.z = atan2(sin(u_yaw), cos(u_yaw));
		}
		
		geometry_msgs::Twist get_msg()
		{
			
			calculateControlCommand(time_pid);
			//ROS_INFO_STREAM("pid_out_msg:["<<twist_msg.linear.x<<","<<twist_msg.linear.y<<"]");
			return twist_msg;
		}		
				
};




