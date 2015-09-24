/******************************************
* Robot Lab 2015 project of AR.Drone 2.0 landing, using the ar_tag ros package for tag tracking.
* 
* depends on ros package for AR tag recognition   http://wiki.ros.org/ar_track_alvar
* 
*/

//std C++ headers
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <string>
#include <iostream>
#include <iomanip>
//ros headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
//this three headers depend on your package config
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//ros marker for ar_tag
//http://wiki.ros.org/rviz/Tutorials/Markers:%20Basic%20Shapes
#include <visualization_msgs/Marker.h>

//ardrone autonomy navdata
#include <ardrone_autonomy/Navdata.h>


#include "pid.h"

#include <drone_land_ar_pid/State.h>
#include <drone_land_ar_pid/PidParameterConfig.h>


using namespace std;
using namespace cv;


static const std::string OPENCV_WINDOW="Image window";

//tool function
void print_msg(geometry_msgs::Twist velocity_msg)
{
	cout<<"vx,vy,vz:"<<velocity_msg.linear.x<<","<<velocity_msg.linear.y<<","<<velocity_msg.linear.z<<endl;
}




class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	double last_dx;
	double last_dy;	

	
	ros::Publisher velocity_pub;
	ros::Publisher takeoff_pub;
	ros::Publisher land_pub;
	ros::Publisher pid_pub;
	ros::Publisher global_pose_pub;
	
	ros::NodeHandle node;
	ros::Subscriber nav_sub;
	ros::Subscriber marker_sub;

	geometry_msgs::Twist velocity_msg;
	geometry_msgs::Twist twist_msg_hover;
	geometry_msgs::Twist velocity_msg_kb;
	geometry_msgs::Twist pid_msg;
	geometry_msgs::Twist global_pose_msg;
	geometry_msgs::Twist pid_velocity_msg;
	ardrone_autonomy::Navdata nav_data_msg;
	
	drone_land_ar_pid::State state;
	ros::Time global_stamp;
	bool stamp_sync ;
	bool first_navdata ;
	bool first_marker ;
	bool Marker_back;
	double ardrone_battery, ardron_altd;
	double last_yaw;
	ArdroneController controller;//pid control construct
	
	//to deal with camera image
	cv::Mat img_raw;
	cv_bridge::CvImagePtr cv_ptr;
	CvFont font;

	double tag_image_x,tag_image_y;//wrt. the image coordinate, the left upper corner is (0,0), and must be int
	char showMsg[50];
	char showMsg2[50];
	char showMsg3[50];
	char showMsg4[50];
	char showMsg5[50];
	double time_diff;
	double max_velocity_limit;
	std_msgs::Empty emp_msg;
	double vel_base;
	//global position of the Drone in the ar_tag frame, the center of the tag on the ground is (0,0,0).
	double global_x, global_y, global_z;
	int key ; //get keyboard input 	
	double time_start;
	ros::Timer timer;
	tf::TransformListener tfListener;

	public:
	ImageConverter()
	:it_(nh_), stamp_sync(false), first_navdata(true), first_marker(true), Marker_back(false) 
	{	
		parameterInit();//parameter init
		controller.init(node);//pid controller init
		
		image_sub_ = it_.subscribe("/ardrone/image_raw",1,&ImageConverter::imageCb,this);//image raw topic
		nav_sub = node.subscribe("/ardrone/navdata",1, &ImageConverter::nav_callback,this);//navdata topic
		marker_sub = node.subscribe("/visualization_marker",1,&ImageConverter::Marker_callback,this);//ar_tag marker topic


		velocity_pub=node.advertise<geometry_msgs::Twist>("/cmd_vel",1);//velocity pub node		
		takeoff_pub = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);//take off pub node
		
		pid_pub = node.advertise<geometry_msgs::Twist>("/pid_pose",1);
		global_pose_pub = node.advertise<geometry_msgs::Twist>("/global_pose",1);
		land_pub = node.advertise<std_msgs::Empty>("/ardrone/land",1);//land
		
		timer = node.createTimer(ros::Duration(0.08), &ImageConverter::onTimerTick, this);
		//ImageConverter::takeoff();//launches the drone		
		
		
		double time_start=(double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec()< time_start+2.){ 
			drone_up();
			cout<<"pub velocity for 0.05s"<<endl;
			}
		
		ROS_INFO("I AR.Drone launched and am ready !");	
	}
  
	~ImageConverter(){
		cv::destroyWindow(OPENCV_WINDOW);
	}
	
	void parameterInit()
	{
		time_diff = 0;
		max_velocity_limit = 0.15f;
		vel_base = 0.1;	
		state.x = 0.0;
		state.y = 0.0;
		state.z = 0.0;
		state.vx = 0.0;
		state.vy = 0.0;
		state.yaw = 0.0;
		pid_velocity_msg = init_msg(pid_velocity_msg);	
	}
	
	void takeoff()
	{
		time_start=(double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec()< time_start+3.0) { 
			takeoff_pub.publish(emp_msg); 
			ROS_INFO("Taking off");
		}
	}
	void land()
	{
		time_start=(double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec()< time_start+4.0) { 
			land_pub.publish(emp_msg);//ardrone land wrapper.
			ROS_INFO("Landing... ");
		}		
	 
	}
	geometry_msgs::Twist init_msg(geometry_msgs::Twist msg)
	{
		msg.linear.x = 0.;
		msg.linear.y = 0.;
		msg.linear.z = 0.;
		msg.angular.x = 0.;
		msg.angular.y = 0.;
		msg.angular.z = 0.;
		return msg;
	}
	void hover()
	{
			//hover message
		twist_msg_hover.linear.x=0.0; 
		twist_msg_hover.linear.y=0.0;
		twist_msg_hover.linear.z=0.0;
		twist_msg_hover.angular.x=0.0; 
		twist_msg_hover.angular.y=0.0;
		twist_msg_hover.angular.z=0.0;
		velocity_pub.publish(twist_msg_hover); 
	}
	void forward()
	{
		pub_velocity_msg_kb(vel_base, 0.00, 0.00);//move forward, the front camera is positive	  
	}
	void backward()
	{
		pub_velocity_msg_kb(-vel_base, 0.00, 0.00);	
	}
	void left()
	{
		pub_velocity_msg_kb(0.00,vel_base, 0.00);	
	}
	void right()
	{
		pub_velocity_msg_kb(0.00,-vel_base, 0.00);
	}
	void drone_up()
	{
		pub_velocity_msg_kb(0.00, 0.00, 0.8);
	}
	void drone_down()
	{
		pub_velocity_msg_kb(0.00, 0.00, -0.4);
	}
	void keyboard_detect(char c)
	{
		if(c=='h'){
			hover();		  
		}	
		if (c=='i'){
			forward();	
		}
		if (c=='k'){
			backward();
		}
		if (c=='j'){
			left();
		}
		if (c=='l'){
			right();
		}
		if (c=='q'){ 
			drone_up();
		}
		if (c=='a'){
			drone_down();
		}
		if (c=='d' | c==' '){
			land();//land 
		}
	}
	  
	
	void pub_velocity_msg(double x, double y, double z)
	{
	  //for velocity control
	  bool x_bad = x>1 | x < -1;
	  bool y_bad = y>1 | y < -1;
	  bool z_bad = z>1 | z < -1;
	  if(x_bad | y_bad | z_bad){
		ROS_ERROR("pub_velocity_msg: bad_x or bad_y or bad_z!");
	  }else{
		
		velocity_msg_kb.linear.x=x;
		velocity_msg_kb.linear.y=y;
		velocity_msg_kb.linear.z=z;
		velocity_msg_kb.angular.x=0.00;
		velocity_msg_kb.angular.y=0.00;
		velocity_msg_kb.angular.z=0.00;
		velocity_pub.publish(velocity_msg_kb);
		
	  } 
	}
	void pub_velocity_msg_kb(double x, double y, double z)
	{
	  //for key board velocity_control
	  bool x_bad = x>1 | x < -1;
	  bool y_bad = y>1 | y < -1;
	  bool z_bad = z>1 | z < -1;
	  if(x_bad | y_bad | z_bad){
		ROS_ERROR("pub_velocity_msg_kb: bad_x or bad_y or bad_z!");
	  }else{
		
		velocity_msg_kb.linear.x=x;
		velocity_msg_kb.linear.y=y;
		velocity_msg_kb.linear.z=z;
		velocity_msg_kb.angular.x=0.00;
		velocity_msg_kb.angular.y=0.00;
		velocity_msg_kb.angular.z=0.00;
		
		double time_start=(double)ros::Time::now().toSec();
		//while ((double)ros::Time::now().toSec()< time_start+0.5) { 
			velocity_pub.publish(velocity_msg_kb);
	//		cout<<"pub velocity for 0.05s:[vx,vy,vz]=["<<x<<","<<y<<","<<z<<"]"<<endl;
	//	}	
	  } 
	}
	
	//this velocity control use the global pose with respect to the tag center.
	void velocity_control_global(double dx, double dy, double dz)
	{		  
	}	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		
		try{
			cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge Exception: %s" ,e.what());
			return;
		}
			
		img_raw = cv_ptr->image;//image transport
		
		//tag_image_x = -int(global_x / 0.36 * 0.56/global_z * 320) + 320;
		//tag_image_y = -int(global_y / 0.24 * 0.56/global_z * 180) + 180;	
		//cv::Point tag_point(tag_image_x, tag_image_y);// tag center in image
		cv::circle(img_raw,cv::Point(320,180),10,CV_RGB(255,0,0));
		//cv::Point center(320,180);//Image center circle
		cv::circle(img_raw,cv::Point(320,180),10,CV_RGB(255,0,0));//center of the image.
			
		//assert(showMsg);
		sprintf(showMsg,"camera_position_in_Marker:(%.2f,%.2f,%.2f)",
				        state.x, state.y,state.z);
		sprintf(showMsg5,"camera_in_marker_RPY:(%.2f, %.2f, %.2f)",
				        state.roll * 1., state.pitch * 1., state.yaw * 1.);
		sprintf(showMsg3,"PID x para:(%.2f,%.2f,%.2f)",
		                controller.pid_x.c_proportional, 
		                controller.pid_x.c_integral, 
		                controller.pid_x.c_derivative);
		//sprintf(showMsg4,"PID y para:(%.2f,%.2f,%.2f)",\
		                controller.pid_y.c_proportional, \
		                controller.pid_y.c_integral, \
		                controller.pid_y.c_derivative);
		sprintf(showMsg2,"pid_pose:(%.2f,%.2f), pub velocity:(%.2f,%.2f)",\
						pid_velocity_msg.linear.x, pid_velocity_msg.linear.y,\
						velocity_msg.linear.x, velocity_msg.linear.y);
		
		display_text(img_raw, cv::Point(20,20), showMsg);
		display_text(img_raw, cv::Point(20,40), showMsg5);		
		display_text(img_raw, cv::Point(20,60), showMsg2);		
		display_text(img_raw, cv::Point(20,80), showMsg3);
		//display_text(img_raw, cv::Point(20,100), showMsg4);
		cv::imshow(OPENCV_WINDOW, img_raw);
		key=cv::waitKey(10); //waitkey
/*		if(key != -1){
		  cout<<"key:["<<char(key)<<"]"<<endl;	
		 // keyboard_detect(char(key));	
		}*/		
		//cout<<"[x,y,z]:=["<<real_x<<","<<real_y<<","<<real_z<<"]"<<endl;
	}
	
	void display_text(cv::Mat image_to_show, cv::Point text_point,  char * showMsg_temp)
	{
		//init font
		cv::putText(image_to_show, showMsg_temp, text_point, CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0));		
	}
	
	void velocity_control(double v_x, double v_y, double z)
	{
		
		v_x = min(max(v_x , -max_velocity_limit), max_velocity_limit);
		v_y = min(max(v_y , -max_velocity_limit), max_velocity_limit);
		velocity_msg.linear.x = v_x;
		velocity_msg.linear.y = v_y;
		if(false){
			land();
		}else{
			pub_velocity_msg(v_x,v_y,-0.0);
		}	
	}
	
	void onTimerTick(const ros::TimerEvent& e)
	{
		
		//ROS_INFO_STREAM("Timer back:"<<Marker_back);
		if(Marker_back){
			ROS_INFO("Marker_back is True and pub velocity");
			velocity_control(pid_velocity_msg.linear.x, pid_velocity_msg.linear.y, state.z);
		}else if(false){
			land();
		}else		
		{
			ROS_INFO("Marker_back is False and hover()");
			//hover();
		}
		Marker_back = false;
	}
	
	bool getMarkerPose(tf::Pose &pose){
		tf::StampedTransform transform;
		try{
		    tfListener.lookupTransform ("ar_marker_2","ardrone_base_frontcam", ros::Time(0), transform);
			pose = transform;
			return true;
		}catch(...){
		   ROS_ERROR("No tf ");	
		}
		return false;
	}

	void Marker_callback(const visualization_msgs::Marker& msg )
	{
		Marker_back = true;
		//global frame, the center of marker is (0,0,0)
		cout<<"Marker in:"<<msg.header.stamp.toSec()<<endl;
		
		
		tf::Pose marker_in_camera;
		tf::Pose camera_in_marker;
		
// 		if(!getMarkerPose(marker_in_camera)){
// 			return;
// 		}
		tf::poseMsgToTF(msg.pose, marker_in_camera);
		camera_in_marker = marker_in_camera.inverse();
		
		state.x = camera_in_marker.getOrigin().x();//m,  position wrt. tag center
		state.y = camera_in_marker.getOrigin().y();//m,  position wrt. tag center
		state.z = camera_in_marker.getOrigin().z();//m
		state.yaw = tf::getYaw(camera_in_marker.getRotation() );
		
		tf::Matrix3x3 m(camera_in_marker.getRotation());
		double t_r, t_p, t_y;
		m.getRPY(t_r, t_p, t_y);
		state.roll  = t_r * 180 / M_PI; 
		state.pitch = t_p * 180 / M_PI;
		state.yaw   = t_y * 180 / M_PI;
		tf::poseTFToMsg(camera_in_marker, state.pose);
		
		
		
		if(first_marker){
			first_marker = false;
			global_stamp = msg.header.stamp;
			return;
		}
		 
		time_diff = (msg.header.stamp - global_stamp).toSec();	
		global_stamp = msg.header.stamp;
		state.header = msg.header;
		state.vx = (state.x - msg.pose.position.x) / time_diff;//state velocity
		state.vy = (state.y - msg.pose.position.y) / time_diff;//state velocity
		
		controller.setPose(state,state.header.stamp);//pid controller
		pid_velocity_msg = controller.get_msg();// return pid out
	
		//velocity_control(pid_velocity_msg.linear.x, pid_velocity_msg.linear.y, state.z);
		
		pid_pub.publish(pid_velocity_msg);
		pub_global_pose();
	}
	
	
	void nav_callback(const ardrone_autonomy::Navdata& msg_in)
	{
		//ROS_INFO("getting sensor reading");
		if(first_navdata){
			last_yaw = msg_in.rotZ;
			first_navdata = false;
			return;
		}		
		//ROS_INFO("stamp_sync is true!");
		stamp_sync = true;
		//state.vx = msg_in.vx / 1000;//m/s
		//state.vy = msg_in.vy / 1000;//m/s
		//state.yaw = msg_in.rotZ  / 180. * M_PI;
		// transport it to rad
		last_yaw = msg_in.rotZ;				
		ardrone_battery = msg_in.batteryPercent;
		ardron_altd = msg_in.altd;		
		//cout<<"ardron state:batteryPercent:"<<ardrone_battery<<endl;
		//cout<<"            :ardron_altd:   "<<ardron_altd<<endl;		
	}
	void pub_global_pose()
	{
		global_pose_msg.linear.x = state.x;
		global_pose_msg.linear.y = state.y;
		global_pose_msg.linear.z = state.z;
		global_pose_msg.angular.z= state.yaw;
		global_pose_pub.publish(global_pose_msg);
		//cout<<"pose,[x,y,z]="<<"["<<state.x<<","<<state.y<<","<<state.z<<"]"<<endl; 
	}
	
};

int main(int argc, char **argv) 
{
	ros::init(argc,argv,"Robot_landing_ar_tag");
	ImageConverter IC;
	ros::spin();
	return 0;
}









