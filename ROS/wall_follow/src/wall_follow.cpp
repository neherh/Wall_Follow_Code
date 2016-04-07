/***********************************************************************
 * Source File:
 *    Arduino Wall Follow : Allow an RC car to read IR values in an array
 *                          and output motors controls to an arduino
 * Author:
 *    Helmut Neher
 * Summary:
 *    Read IR Values from ROS and YAML file of parameters, and odometry
 *    from odometry (encoders/kinect) to calulate heading and speed and 
 *    publish using ROS to arduino.
 ************************************************************************/
#include <wall_follow/wall_follow.h>

/******************************************
 * ArduinoWallFollow : Callback
 * Initializes subscriber and begins wall
 * following
 *****************************************/
 wall_follow::wall_follow() : ticks_r_old(0),ticks_l_old(0),x(0),y(0),phi(0)
{
   // get Parameters from Ros server
   ROS_INFO("Getting Parameters");
   getParam(nh_);

   // Subscribe to values
   ROS_INFO("Subscribing to topics");
   ir_sub   = nh_.subscribe(laser_scan_topic, 10, &wall_follow::irCb, this);     // subscribe to ir values
   odom_sub = nh_.subscribe(odometry_topic, 10, &wall_follow::odomCb, this);   // subscribe to odom values
   traj_angle = nh_.advertise<std_msgs::Float32>(traj_angle_topic,10);

 //  image_pub_ = it_.advertise("/image_converter/output_video", 1); 
}

/******************************************
 * Wall Follow : Get Param
 * Gets all parameters from Ros server and
 * provides default if unable to get values
 *****************************************/
 void wall_follow::getParam(ros::NodeHandle nh)
 {

   if (nh.getParam("wall_follow/ir/min_ir_val", min_ir_val))
      ROS_INFO_STREAM("Got param min_ir_val: " << min_ir_val);
   else{
      ROS_INFO("Failed to get wall_follow/ir/min_ir_val");
      min_ir_val = 0.5;
   }

   if (nh.getParam("wall_follow/ir/max_ir_val", max_ir_val))
      ROS_INFO_STREAM("Got param max_ir_val: " << max_ir_val);
   else{
      ROS_INFO("Failed to get wall_follow/ir/max_ir_val");
      max_ir_val = 4.0;
   }

   if (nh.getParam("wall_follow/dimensions/tick_rev", tick_rev))
      ROS_INFO_STREAM("Got param tick_rev: " << tick_rev);
   else{
      ROS_INFO("Failed to get wall_follow/dimensions/tick_rev");
      tick_rev = 128;
   }

   if (nh.getParam("wall_follow/dimensions/dia_wheel", dia_wheel))
      ROS_INFO_STREAM("Got param dia_wheel: " << dia_wheel);
   else{
      ROS_INFO("Failed to get wall_follow/dimensions/dia_wheel");
      dia_wheel = 0.1;
   }

   if (nh.getParam("wall_follow/dimensions/width_wheel", width_wheel))
      ROS_INFO_STREAM("Got param width_wheel: " << width_wheel);
   else{
      ROS_INFO("Failed to get wall_follow/dimensions/width_wheel");
      width_wheel = 0.3;
   }

   if (nh.getParam("wall_follow/wall_follow/des_wall_dist", des_wall_dist))
      ROS_INFO_STREAM("Got param des_wall_dist: " << des_wall_dist);
   else{
      ROS_INFO("Failed to get wall_follow/wall_follow/des_wall_dist");
      des_wall_dist = 0.55;
   }

   if (nh.getParam("wall_follow/wall_follow/min_front_dist", min_front_dist))
      ROS_INFO_STREAM("Got param min_front_dist: " << min_front_dist);
   else{
      ROS_INFO("Failed to get wall_follow/wall_follow/min_front_dist");
      min_front_dist = 0.5;
   }

   if (nh.getParam("wall_follow/wall_follow/max_front_dist", max_front_dist))
      ROS_INFO_STREAM("Got param max_front_dist: " << max_front_dist);
   else{
      ROS_INFO("Failed to get wall_follow/wall_follow/max_front_dist");
      max_front_dist = 0.8;
   }

   if (nh.getParam("wall_follow/wall_follow/min_side_dist", min_side_dist))
      ROS_INFO_STREAM("Got param min_side_dist: " << min_side_dist);
   else{
      ROS_INFO("Failed to get wall_follow/wall_follow/min_side_dist");
      min_side_dist = 0.5;
   }

   if (nh.getParam("wall_follow/wall_follow/max_side_dist", max_side_dist))
      ROS_INFO_STREAM("Got param max_side_dist: " << max_side_dist);
   else{
      ROS_INFO("Failed to get wall_follow/wall_follow/max_side_dist");
      max_side_dist = 0.8;
   }

   if (nh.getParam("wall_follow/wall_follow/right_array_val", right_array_val))
      ROS_INFO_STREAM("Got param max_side_dist: " << right_array_val);
   else{
      ROS_INFO("Failed to get wall_follow/wall_follow/right_array_val");
      max_side_dist = 12;
   }

   if (nh.getParam("wall_follow/wall_follow/center_array_val", center_array_val))
      ROS_INFO_STREAM("Got param max_side_dist: " << center_array_val);
   else{
      ROS_INFO("Failed to get wall_follow/wall_follow/center_array_val");
      max_side_dist = 17;
   }

   if (nh.getParam("wall_follow/controller/kp", kp))
      ROS_INFO_STREAM("Got param kp: " << kp);
   else{
      ROS_INFO("Failed to get wall_follow/controller/kp");
      kp = 1;
   }

   if (nh.getParam("wall_follow/odometry_topic", odometry_topic))
      ROS_INFO_STREAM("Got param odometry_topic: " << odometry_topic);
   else
      ROS_INFO("Failed to get wall_follow/odometry_topic");


   if (nh.getParam("wall_follow/laser_scan_topic", laser_scan_topic))
      ROS_INFO_STREAM("Got param laser_scan_topic: " << laser_scan_topic);
   else
      ROS_INFO("Failed to get wall_follow/laser_scan_topic");

   if (nh.getParam("wall_follow/traj_angle_topic", traj_angle_topic))
      ROS_INFO_STREAM("Got param laser_scan_topic: " << traj_angle_topic);
   else
      ROS_INFO("Failed to get wall_follow/traj_angle_topic");
 }

/******************************************
 * irCB : Infrared Callback
 * Read IR values and store in array
 *****************************************/
void wall_follow::irCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  right_laser_scan  = msg->ranges[right_array_val];
  center_laser_scan = msg->ranges[center_array_val];
  laser_msg = *msg;
  ROS_INFO("I heard laserscan message");
//  ROS_INFO_STREAM("right_array_val  " << right_laser_scan);
//  ROS_INFO_STREAM("center_array_val " << center_laser_scan);
}

/******************************************
 * odomCB : Odometry Callback
 * Read Odometry values and store in array
 *****************************************/
void wall_follow::odomCb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ticks_l = msg->data[0];	// array of encoder values (just 2 values sent from the arduino)
  ticks_r = msg->data[1];
  ROS_INFO_STREAM("I heard and accepted ticks_l: " << ticks_l << " and ticks_r: " << ticks_r);

 getTrajAngle();
}

/******************************************
 * odomCB : Odometry Callback
 * Read Odometry values and store in array
 *****************************************/
void wall_follow::getTrajAngle()
{

	// calculate encoder counts between each loop.
	dTicks_l    = ticks_l - ticks_l_old;
	ticks_l_old = ticks_l;

	dTicks_r    = ticks_r - ticks_r_old;
	ticks_r_old =  ticks_r;

	// update pose of the car
	//calculate distance
	left_dist   = pi * dia_wheel * dTicks_l / tick_rev;
	right_dist  = pi * dia_wheel * dTicks_r / tick_rev;
	center_dist = (left_dist + right_dist) / 2;

	// calculate new pose
	x   = x + center_dist * cos(phi);
	y   = y + center_dist * sin(phi);
	phi = phi + (right_dist - left_dist) / width_wheel;
	phi = atan2(sin(phi),cos(phi));

	// calculate forward and side vector and angle
	if (center_laser_scan > max_ir_val)
		center_laser_scan = max_ir_val;
	else if (center_laser_scan < min_ir_val)
		center_laser_scan = min_ir_val;

	if (right_laser_scan > max_ir_val)
		right_laser_scan = max_ir_val;
	else if (right_laser_scan < min_ir_val)
		right_laser_scan = min_ir_val;

	uF    = center_laser_scan;
	uS    = sqrt(2) * des_wall_dist - right_laser_scan;
	phi_w = atan2(uS,uF);

	// set trajectory msg and publish msg
	traj_angle_msg.data = phi_w;
	traj_angle.publish(traj_angle_msg);

	ROS_INFO_STREAM("Angle Tracking COmpleted: " << traj_angle_msg);
}

/********************************************************************************************************

SUDO CODE FOR TRAVERSING ARRAY

get irVal

if irVal[12] <= threshold && irVal[11-9] && irVal[13-15] >= threshold then
   SET follow_state = TRUE;

The idea is to get the IRvals, determine when the right? or left? val is is the only one that is slightly
under the threshold, is where we want the car distance from the wall to be set. 

If the leftmost is > than the threshold, it'll correct to get closer. If leftmost < min_dist < threshold,
then it starts veering from the wall

If there is a 90 degree corner in



 ********************************************************************************************************/