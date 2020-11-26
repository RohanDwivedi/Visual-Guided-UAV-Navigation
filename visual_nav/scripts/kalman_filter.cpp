
/* Kalman Filter Node */

#include "ros/ros.h"
#include "rospy_tutorials/Floats.h"                     // custom ROS message type Float32 defined in rospy_tutorials package
#include <vector>
#include <iostream>
#include "/usr/local/include/eigen3/Eigen/Dense"        // Newer version of the eigen library 3.37, support for orthogonal decomposition, installed from github  https://gitlab.com/libeigen/eigen.git
#include <cmath>
#include "visual_nav/pos_data.h"
#include "visual_nav/pos_vector.h"
#include "std_msgs/Bool.h"

using namespace Eigen;

// Global Declarations

float dt = 0.1;            				// time step T(k) - T(k-1)                      :TODO: Tune the value
float lost = 0;
float q_var = 0.0;        				// variance caused due to external disturbances :TODO: Tune the value

float sensor_var_x = 0.19;                              // values obtained by experimentation, see visual_nav/data_logs/log.csv
float sensor_var_y = 0.62;              		// variance in the sensor values               

Matrix < float, 4, 1 > X;  				// [pos_x; pos_y; vel_x; vel_y]
Matrix < float, 4, 4 > P; 				// covariance Matrix
Matrix < float, 4, 4 > F;  				// Prediction Matrix
Matrix < float, 2, 4 > H;  				// Sensor Model Matrix
Matrix < float, 2, 2 > R;  				// Covariance Matrix of sensor uncertainity (sensor --> object detection prediction) 
Matrix < float, 4, 4 > Q;  				// Covariance Matrix of world uncertainity ( Noise added due to slippage, wind or other disturbances)
Matrix < float, 4, 1 > x_predict; 			// values obtained for [pos_x; pos_y; vel_x; vel_y] at the end of the predict cycle
Matrix < float, 4, 4 > p_predict; 			// values obtained for P at the end of predict cycle
Matrix < float, 2, 1 > inn;  				// innovation in sensor meseurement
Matrix < float, 4, 1 > x_update; 			// values obtained for [pos_x; pos_y; vel_x; vel_y] at the end of the update cycle
Matrix < float, 4, 4 > p_update;                        // values obtained for P at the end of update cycle
Matrix < float, 4, 4 > I;	                        // eye(rows,cols)



std_msgs::Bool is_obj_det;

std::vector<float> kalman_filter(std::vector<float> obj_center,  Matrix < float, 4, 1 > &X,
								 Matrix < float, 4, 4 > &P,
								 Matrix < float, 4, 4 > &F,
								 Matrix < float, 2, 4 > &H,
								 Matrix < float, 2, 2 > &R,
								 Matrix < float, 4, 4 > &Q,
								 Matrix < float, 4, 4 > &I )

{
   /* + This function updates 1 pass of the kalman filter algorithm 
      + tracks the center of the object (point)
      + Predicts the path of the point over time
      + returns the prediction to be published for the UAV controller */



MatrixXf sensor_val = Map<Matrix<float, 2, 1> >(obj_center.data());                  // changing std::vector to Eigen::vector
//std::cout<<"Sensor value: " << sensor_val << "\n";                       

if (!sensor_val.isZero(0))                                                           // if object is detected then correct the measurement
{
  lost = 0;
// --PREDICT CYCLE--

x_predict = F * X;
//std::cout << " x_predict: " << x_predict << "\n";     

p_predict = F * P * F.transpose() + Q;
//std::cout << " p_predict: " << p_predict << "\n";    


// --UPDATE CYCLE--

CompleteOrthogonalDecomposition <MatrixXf> cqr(H * P * H.transpose() + R);          // pseudo inverse using orthgonal decomposition; pseudo inverse incase matrix has zero determinant
MatrixXf pinv = cqr.pseudoInverse();

Matrix < float, 4, 2> kalman_gain = P * H.transpose() * pinv;                       // calculate Kalman Gain
//std::cout<<"Kalman Gain: " << kalman_gain << "\n";                         

inn = sensor_val - (H * X);                                                         // calculate innovation/residual
//std::cout<<"Innovation: " << inn << "\n";                                 

x_update = x_predict + (kalman_gain * inn); 
//std::cout << " x_update: " << x_update << "\n";  
                       
p_update = (I - (kalman_gain * H)) * P;
//std::cout << " p_update: " << p_update << "\n";                         

X = x_update;
P = p_update;
std::cout<< "P: "<< "\n" << P <<"\n";

}

else                                                                               // if object is not detected then rely on predicted value                                                                 
{

std::cout<<"No object detected"<< "\n";

if (lost > 2)
{   
    lost = 0;
    is_obj_det.data = false;

    X.fill(0);
    P <<    100, 0, 0, 0,                //:TODO: Tune the value
             0,100, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;
    x_predict << 0,
		 0,
		 0,
                 0;
    
    p_predict << 0,0,0,0,
                 0,0,0,0,
                 0,0,0,0,
                 0,0,0,0;
}

else
{   
    lost = lost + 1;
    X = X;
    P = P;                //:TODO: Tune the value
}

}

std::vector <float> pred_obj_center (x_predict.data(), x_predict.data() + x_predict.rows() * x_predict.cols());   
// convert eigen matrix to std::vector // std::vector <float> pred_obj_center (X.data(), X.data() + X.rows() * X.cols());

return pred_obj_center;                                                           // return tracked position,
}


std::vector<float> find_center(float xmin, float ymin, float xmax, float ymax)
{	/* This function takes the coordinates of the bouding box of the detected object and finds its center
		returns a vector of x and y coordinates of the center of the bounding box */

	float x_center = xmin + ((xmax - xmin)/2);
	float y_center = ymin + ((ymax - ymin)/2);
	std::vector<float> obj_center(2);
	obj_center[0] = x_center;
	obj_center[1] = y_center;
	return obj_center;
}


class SubPub
{

public:

    SubPub()
	{
		pub = n.advertise<visual_nav::pos_vector>("/Kalman_pred", 1000);                   // Declare a Publisher for kalman Prediction
                pub2 = n.advertise<std_msgs::Bool>("/is_obj_detected", 1000);                      // publish true if object has been detected    
                sub = n.subscribe("/obj_cdnts", 1, &SubPub::get_obj_cdnts_callback, this);         // Subscribe to the /obj_cdnts topic
	}
    
    void get_obj_cdnts_callback(const rospy_tutorials::Floats msg)
	{
               
                /* This function runs callback to subscribe to obj_cdnts topic 
                 + Acts like a sensor value update
                 + Pass the sensor value to the Kalman Filter to update 1 pass of the Algorithm
                 + Publish kalman prediction */

                 //ROS_INFO_STREAM("object coordinates:" << msg); 
                   ROS_INFO_ONCE("Kalman Filter Node started");
    
                   float xmin = 0; float ymin = 0; float xmax = 0; float ymax = 0;
                   
                   if(!msg.data.empty()) 
                     {
                       xmin = msg.data[0]; ymin = msg.data[1]; xmax = msg.data[2]; ymax = msg.data[3];
                       is_obj_det.data = true;
                     } 	// Avoid segmentation fault in case of empty message (nothing detected)
                   
                   std::vector<float> obj_center = find_center(xmin, ymin, xmax, ymax);		
                   std::cout << "Sensor value:- " << "X Center: " << obj_center[0] << " " << "Y center: " << obj_center[1] << "\n";                    
                   
                   std::vector <float> pred_obj_center = kalman_filter(obj_center, X, P, F, H, R, Q, I);  // Call 1 pass of Kalman Filter
                   std::cout << "kalman Prediction:- " << "X Center: " << pred_obj_center[0] << " " << "Y center: " << pred_obj_center[1] << "\n";      
                   std::cout << "--------------------------------------------------" << "\n";                                                        
    
                   visual_nav::pos_data data;
                   visual_nav::pos_vector pos_msg;
                   data.x_pos = pred_obj_center[0];
                   data.y_pos = pred_obj_center[1];
                   pos_msg.posVector.push_back(data);
                   pub.publish(pos_msg);  // Publish Kalman Predictions to topic /kalman_pred
                   pub2.publish(is_obj_det);
           }

private:

ros::NodeHandle n;       // Declare node handle object
ros::Publisher pub;
ros::Publisher pub2;
ros::Subscriber sub;

};




int main (int argc, char **argv)
{

X.fill(0);

P <<   100,   0, 0, 0,                    //:TODO: Tune the value
        0,  100, 0, 0,
        0,   0, 0, 0,
        0,   0, 0, 0;

F << 1, 0, dt,  0,
     0, 1,  0, dt,
     0, 0,  1,  0,
     0, 0,  0,  1;

I << 1, 0, 0, 0,
     0, 1, 0, 0,
     0, 0, 1, 0,
     0, 0, 0, 1;

H << 1, 0, 0, 0,
     0, 1, 0, 0;
 
R << sensor_var_x, 0,                    //:TODO: Tune the value
     0, sensor_var_y;

Q << q_var, 0,     0, 0,                 //:TODO: Tune the value
         0, 0, q_var, 0,
         0, 0,     0, 0,
         0, 0,     0, 0;

ros::init(argc, argv, "kalman_filter_tracking");                                  // Initialise the kalman filter node
SubPub Kalman_filter;                                                             // Declare Class PubSub Object
ros::spin();                                                                      // Lets the callback run in loop

return 0;

}

