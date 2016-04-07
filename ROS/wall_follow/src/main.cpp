/***********************************************************************
 * Main File:
 *    Arduino Wall Follow : Allow an RC car to read IR values in an array
 *                          and output motors controls to an arduino
 * Author:
 *    Helmut Neher
 * Summary:
 *    Main file to initialize Wall Follow Class
 ************************************************************************/

#include <ros/ros.h>
//#include <std_msgs/String.h>
#include <wall_follow/wall_follow.h>


/****************************************
 * getIRValues (array)
 * Gets IR Values from the ROS Callback
 * function or is this the callback function?
 ****************************************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_follow_arduino");
  wall_follow awf;
  ros::spin();

  return 0;
}