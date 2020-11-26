/* Controller to trace the path of the detected object as closely as possible */

#include "ros/ros.h"
#include <vector>
#include <iostream>
#include "visual_nav/pos_data.h"
#include "visual_nav/pos_vector.h"
#include <cmath>
#include "/usr/local/include/eigen3/Eigen/Dense"  // Newer version of the eigen library 3.37, support for orthogonal decomposition, installed from github https://gitlab.com/libeigen/eigen.git
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"


class SubPub
{

public:

   SubPub()
       {   
            
	    sub = n.subscribe("/kalman_pred_3d", 1, &SubPub::get_kalman_pred_3d_callback, this);
	    sub2 = n.subscribe("/is_obj_detected", 1, &SubPub::is_obj_detected_callback, this);
            pub = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1000);
       }
     
    
    void correct_yaw()
       {  
	  
	  std::cout << " correcting yaw  \n";
	  msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0;
          msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0;
	  pub.publish(msg);
	  msg.angular.z = 1;
          pub.publish(msg);
       }

    void pdControl()
       {  

	  if (!is_obj_detected) 
	  { 
	  
	  Ux = 0;
	  Uy = 0;

	  error << 0,
		   0;
	  prev_error << 0,
			0;

          msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0;
          msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0;
	  pub.publish(msg);
	    
	  return;
 	  }

	  else
	  {
	 
          std::cout << " moving to position \n ";
	  std::cout << "x world: " << x_world << " " << "y_world: " << y_world << " \n" ; 
		
	  desPos << 0,
		    0;

	  curPos << x_world,
		    y_world;

	  error = desPos - curPos;
	  Ux = Kp_x * error.coeff(0,0) + Kd_x * (prev_error.coeff(0,0) - error.coeff(0,0));
	  Uy = Kp_y * error.coeff(1,0); + Kd_y * (prev_error.coeff(1,0) - error.coeff(1,0));
		     
	  msg.linear.x = -(Uy); msg.linear.y = Ux; msg.linear.z = 0;  		
          msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = -0.1;
          
	  pub.publish(msg);		
	  
          }  
       }
     
    void get_kalman_pred_3d_callback (const visual_nav::pos_vector::ConstPtr& world_msg)
       {
            const visual_nav::pos_data &data = world_msg->posVector[0];
            x_world = data.x_pos;
            y_world = data.y_pos;     
       }

    void is_obj_detected_callback(const std_msgs::Bool& is_obj_det)
       {
           is_obj_detected = is_obj_det.data;
	   (!is_obj_detected) ? correct_yaw() : pdControl(); 
       }

private:

ros::NodeHandle   n;                                                       
ros::Subscriber sub;	
ros::Subscriber sub2;
ros::Publisher  pub;
geometry_msgs::Twist msg;
bool is_obj_detected = false;

float Kp_x = 0.1; 
float Kp_y = 0.1;    	                                                     
float Kd_x = 0.1;
float Kd_y = 0.1;    
float x_world = 0;
float y_world = 0;

Eigen::MatrixXf error = Eigen::MatrixXf::Zero(2, 1);
Eigen::MatrixXf prev_error = Eigen::MatrixXf::Zero(2, 1);
Eigen::Matrix < float, 2, 1 > curPos;
Eigen::Matrix < float, 2, 1 > desPos;                 
	  	                                                      
float Ux = 0;
float Uy = 0;
};



int main (int argc, char **argv)
{

ros::init(argc, argv, "controller" );
SubPub controller;
ros::AsyncSpinner spinner(1);
spinner.start();
ros::waitForShutdown();
return 0;

}                                                                  

