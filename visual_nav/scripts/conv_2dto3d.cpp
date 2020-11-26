
/* Convert 2d coordinates of the object location to world coordinates in 3d */

#include "ros/ros.h"
#include <vector>
#include <iostream>
#include "visual_nav/pos_data.h"
#include "visual_nav/pos_vector.h"
#include "/usr/local/include/eigen3/Eigen/Dense"  // Newer version of the eigen library 3.37, support for orthogonal decomposition, installed from github https://gitlab.com/libeigen/eigen.git
#include <cmath>
#include "rospy_tutorials/Floats.h"

using namespace Eigen;

// Global Declarations 
float d_real = 6.0;          // Length of the diagonal of the bounding box enclosing the vehicle  as seen from the rear end ~ 6.0 mm
float d_img = 0;             // Length of the diagonal of the bouding box obtained from the object detection node
float depth = 0;             // Y world, depth or distance of the object from the camera 
float x_world = 0;           // X world, distance from the camera in x direction. (-) object on the left side of the drone, (+) object on the right side of the drone
float y_world = 0;
float focal_length = 385.5;  // Focal length of the camera in pixels. From datasheet; F in mm = 1.8 mm, Horizontal FOV = 80, Image width = 640. F in pixels =  (640 * 0.5)/ (tan 80 * 0.5 * (PI/180))
std::vector <float> img_cdnts;
std::vector <float> pos_3d = {0,0};

// Intrinsic parameters of the camera obtained from calibration datasheet
float Fx = 537.29; // focal length in x direction; F * Sx
float Cx = 427.33; // focal length in y direction; F * Sy
float Fy = 527.00; // camera principal points, center of the image X coordinate
float Cy = 240.22;  // camera principal points, center of the image y coordinate


std::vector <float> conv_2dto3d ( float x_img, float y_img, float d_img )
{
   /* convert image coordinates to world coordinates */
   if (d_img !=0)
   {
	std::cout<<"d image: " << d_img << "\n";
  	depth   = ((d_real * focal_length)/d_img);

   	std::cout<<"depth: " << depth << "\n";
   	x_world = (((x_img - Cx) * depth)/Fy);
        y_world = depth;

   	std::cout<<"x_world: " << x_world << "\n";
   	std::cout<<"y_world: " << y_world << "\n";

	pos_3d[0] = x_world;
   	pos_3d[1] = y_world;
	return pos_3d;   
   }
   
   else                                                  // resolve divide by zero error when kalman pred is zero in case no object is detected
   {
	pos_3d[0] = 0; //0                             // prev values
        pos_3d[1] = 0; //0
        return pos_3d; 
   }	
}


class SubPub
{

public:

    SubPub()
	{
		pub = n.advertise<visual_nav::pos_vector>("/kalman_pred_3d", 10000);                     // Declare a Publisher for kalman Prediction    
		sub2 = n.subscribe("/obj_cdnts", 1, &SubPub::get_obj_cdnts_callback, this);              // subscribe to /obj_cdnts to get diagonal of the bounding box
                sub1 = n.subscribe("/Kalman_pred", 1, &SubPub::get_kalman_pred_callback, this);          // Subscribe to the /'K'alman_pred topic
                
	}
    
    void get_kalman_pred_callback(const visual_nav::pos_vector::ConstPtr& msg)
	{      
	       // convert kalman_pred 2d coordinates to world coordinates

               ROS_INFO_ONCE(" Converting 2d position to 3d ");
            
               const visual_nav::pos_data &data = msg->posVector[0];
           
               std::vector <float> pos_3d = conv_2dto3d(data.x_pos, data.y_pos, d_img);

               std::cout<<"pos_3d: " << pos_3d[0] << " " << pos_3d[1] << "\n";

               visual_nav::pos_data data_;
               visual_nav::pos_vector pos_msg_3d;
               data_.x_pos = pos_3d[0];
               data_.y_pos = pos_3d[1];
               pos_msg_3d.posVector.push_back(data_);
               pub.publish(pos_msg_3d);  
        }

        void get_obj_cdnts_callback(const rospy_tutorials::Floats msg)
	{      
	       // calculate the approximate diagonal of the bounding box

               float xmin = 0; float ymin = 0; float xmax = 0; float ymax = 0;
               if(!msg.data.empty()) {xmin = msg.data[0]; ymin = msg.data[1]; xmax = msg.data[2]; ymax = msg.data[3];}  // Avoid segmentation fault in case of empty message (nothing detected)
               d_img  = std::sqrt(std::pow(xmax-xmin,2) + std::pow(ymax-ymin,2)); 
        }

private:

ros::NodeHandle   n;   // Declare node handle object
ros::Publisher  pub;
ros::Subscriber sub1;
ros::Subscriber sub2;	
};



int main (int argc, char **argv)
{

ros::init(argc, argv, "conv_2dto3d");
SubPub conv_2dto3d;
ros::spin();

return 0;
}
