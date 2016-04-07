/***********************************************************************
 * Header File:
 *    Arduino Wall Follow : Allow an RC car to read IR values in an array
 *                          and output motors controls to an arduino
 * Author:
 *    Helmut Neher
 * Summary:
 *    Read IR Values from ROS and YAML file of parameters, and odometry
 *    from odometry (encoders/kinect) to calulate heading and speed and 
 *    publish using ROS to arduino.
 ************************************************************************/

#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
//#include <string.h>


/*********************************************
 * ARDUINOWALLFOLLOW
 * Wall Following Class which interfaces with
 * arduino  
 *********************************************/
class wall_follow
{
public:
   // constructor
   wall_follow();

   // destructor
   ~wall_follow()           { }

   // getters
  /* float getX()       const { return x;              }
   float getY()       const { return y;              }
   bool  getCheck()   const { return check;          }
   bool  isDead()     const { return dead;           }
   float getXMin()    const { return xMin;           }
   float getXMax()    const { return xMax;           }
   float getYMin()    const { return yMin;           }
   float getYMax()    const { return yMax;           }
   bool  getWrap()    const { return wrap;           }

   // setters
   void setX(float x);
   void setY(float y);
   void addX(float dx)      { setX(getX() + dx);     }
   void addY(float dy)      { setY(getY() + dy);     }
   void setCheck(bool f)    { check = f;             }
   void setWrap(bool f)     { wrap = f;              }
   void kill()              { dead = true;           }
   void resurrect()         { dead = false;          }
*/
private:
   void irCb  (const sensor_msgs::LaserScan::ConstPtr& msg);
   void odomCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
   void getParam(ros::NodeHandle nh);
   void getTrajAngle();

  // ROS variables
  ros::NodeHandle nh_;
  ros::Subscriber ir_sub;
  ros::Subscriber odom_sub;
  ros::Publisher  traj_angle;

  // wall_follow variables:
  //ubfloat ir [];                    // store IR values 
  double min_ir_val, max_ir_val;             // max and min ir values can be read
  int ticks_l,ticks_r, ticks_r_old,
  ticks_l_old, dTicks_l, dTicks_r;           // change in ticks on left and right encoders   // subscribing to arduino
  int tick_rev;                              // ticks for revolution
  double dia_wheel, width_wheel;             // diameter of wheel and width between wheels
  double left_dist, right_dist, center_dist; // distanced traveled from left and right wheel and robot center       
  double x,y,phi;                            // current estimate of robot (m,m,radians)
  double phi_des, v_des, w_des;             // desired robot variables
  int v_left, v_right;                      // velocity of left and right encoder
  int kp;                                   // prop. conrol gain
  double pi;                                // pi = 3.1459
 
  int min_dist, obj_front, obj_left,
  obj_right, x0_rel, y0_rel, phi_rel;       // avoidance behavor characteristics
  
  double des_wall_dist, min_front_dist,      // wall follow variables
  max_front_dist, min_side_dist,
  max_side_dist;

  int right_array_val, center_array_val;
  double right_laser_scan, center_laser_scan;

  double uF, uS,phi_w;                      // forward, side vector and angle

  std::string odometry_topic, laser_scan_topic, traj_angle_topic;

  sensor_msgs::LaserScan laser_msg;
  std_msgs::Float32 traj_angle_msg;
};


#endif // ARDUINOWALLFOLLOW_H