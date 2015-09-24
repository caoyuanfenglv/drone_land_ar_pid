###PROJECT READ ME
Automatic Landing on a Moving Target
This is course project in Mobile Robotics Lab Spring 2015, Shanghaitech University.



---
*project handbook*
*author: guojiangtao chenminhua*
*mail: guojt@shanghaitech.edu.cn, chenmh@shanghaitech.edu.cn*

####What we do?
This project is about that landing the UAV(AR.Drone 2.0) on a moving target with special marker,(now, we just move the marker manually)(if necessary use the turtlebot as the moving target), here the AR_tag has been used.

####Hardware in our project：

-  [AD.Drone2.0](http://cdn.ardrone2.parrot.com/)
-  Laptop with Ubuntu 14.04 and [ROS indigo](http://www.ros.org/)
-  [Turtlebot](http://wiki.ros.org/Robots/TurtleBot)（second stage as the moving platform, not yet implement）

####Software in our project：
-  [ROS indigo](http://www.ros.org/)
-  [Ubuntu 14.04](http://www.ubuntu.com)
-  AD.Drone2.0 onboard drivers
-  [ros package `ar_track_alvar`](http://wiki.ros.org/ar_track_alvar) for AR_tag recognition
-  ros package[`ardrone_autonomy`](http://wiki.ros.org/ardrone_autonomy) and [ardrone_autonomy Lab](https://github.com/AutonomyLab/ardrone_autonomy/tree/autolab), AR.Drone 2.0 driver ros wrapper
-  ros package[`tum_ardrone`](http://wiki.ros.org/tum_ardrone), a very nice AR.Drone project, and give us much help, such as the PID controller, EKF, odometry estimation.

####How to use our project
Here, you should already have the above all software and hardware available.
Our package is a standard ROS package, you should first copy this package to `your_own_ros_catkin_ws`, and use `catkin_make` compile it. Please refer to ROS wiki.

**Calibrate the AR.Drone camera**

I have used the ROS calibration tools. For more detail, please refer to:

- [http://ardrone-autonomy.readthedocs.org/en/latest/FAQ.html](http://ardrone-autonomy.readthedocs.org/en/latest/FAQ.html)
- [ros wiki camera calibration](http://wiki.ros.org/camera_calibration)
- [Monocular Calibration tutorial in ROS](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

After camera calibration, for AR.Drone, you should get the `.yaml` camera info file, and put the file into the folder `~/.ros/camera_info/`. if no, put it here.

**Before launch the node ar_track_alvar**

we have used our own launch file, the marker size used in our experiments is the Marker_ID 2 and size 13.2 centimeter. You should change to your own real size and for more detail you should refer to the [`ar_track_alvar` package wiki](http://wiki.ros.org/ar_track_alvar)
	
    <launch>
        <arg name="marker_size" default="13.2" />
        <arg name="max_new_marker_error" default="0.08" />
        <arg name="max_track_error" default="0.2" />	
        <arg name="cam_image_topic" default="/ardrone/front/image_raw" />
        <arg name="cam_info_topic" default="/ardrone/front/camera_info" />	
        <arg name="output_frame" default="/ardrone_base_frontcam" />

        <node name="ar_track_alvar" pkg="ar_track_alvar" type="individualMarkersNoKinect" respawn="false" output="screen" args="$(arg marker_size) $(arg max_new_marker_error) $(arg max_track_error) $(arg cam_image_topic) $(arg cam_info_topic) $(arg output_frame)" />
    </launch>
copy this launch file and put it in your `ar_track_alvar` package launch folder, then compile again.


**Quick Launch**

first, we should launch the roscore server
	
    $ roscore
second, we launch the `ardrone_autonomy` AR.Drone driver

	$ rosrun ardrone_autonomy ardrone_driver
third, launch the marker recognition node

	$ roslaunch ar_track_alvar Marktest_no_kinect.launch (here is mine, change to your own launch file name)

forth, launch the keyboard control node in our package.

	$ rosrun drone_land_ar_pid drone_land_tele_keyboard_node 
you will get the info about how to control AR.Drone and the key bondings.

    Reading from keyboard
    Use:
    i,k,j,l: for forward, backward, left, right, value = 0.3
    q,a: for up, down, value = 0.4
    blank space: for stop and hover. 
    h: for hover
    t,d: for takeoff and land

You can put the mouse focus on this terminal to control AR.Drone. You should test to control the AR.Drone with keyboard before move on.

fifth, launch the main node to track AR_tag marker

	$ rosrun drone_land_ar_pid drone_land_ar_pid

Then, you can move the tag, let Drone tracking it.


#### Update frequencies and data transport

**Drone Update Frequencies**: The drone’s data transmission update frequency depends on *navdata_demo* parameter. When it is set to 1, the transmission frequency is set 15Hz, otherwise transmission frequency is set to 200Hz. (*navdata_demo* is a numeric parameter not Boolean, so use 1 and 0 (not True/False) to set/unset it)

**Driver Update Frequencies**: The driver can operate in two modes: real-time or fixed rate. When the `realtime_navdata` parameter is set to True, the driver publishes any received information instantly. When it is set to False, the driver caches the received data first, then sends them at a fixed rate. This rate is configured via looprate parameter. The default configuration is: `realtime_navdata=False` and `looprate=50`.

refer to the `ardrone_autonomy` driver doc [Update frequencies](http://ardrone-autonomy.readthedocs.org/en/latest/reading.html)




