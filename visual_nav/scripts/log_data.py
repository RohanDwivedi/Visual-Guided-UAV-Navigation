#!/usr/bin/env python2.7

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import os


from visual_nav.msg import pos_data
from visual_nav.msg import pos_vector
import pandas as pd

# Do not runs this file from rosrun.
# This code is intended to obtain the position data 
# and generate a csv file to calculate variance 
# in obtaining position data of the object
# in order to tune the covaraince matrix R
# for Kalman filter tuning


def find_center(x_min,y_min,x_max,y_max):
	
	x_center = x_min + ((x_max - x_min)/2)
	y_center = y_min + ((y_max - y_min)/2)
	return [x_center, y_center]
	

def pos_callback(msg):

        global data_measure

	if( msg.data != []):
		x_min = msg.data[0]
        	y_min = msg.data[1]
        	x_max = msg.data[2]
        	y_max = msg.data[3]
		x_center, y_center = find_center(x_min,y_min,x_max,y_max)
        	data_measure.append([x_center, y_center])
	else:
		data_measure.append([0, 0])

	

def kalman_callback(msg):

        global data_predict
	
 	x_pred = msg.posVector[0].x_pos
        y_pred = msg.posVector[0].y_pos
        data_predict.append([x_pred, y_pred])
	
	

if __name__ == '__main__': 
        
	cwd = os.getcwd() 
        file_path = os.path.dirname(cwd) + '/Desktop/catkin_ws/src/visual_nav/DataLogs/data/log.csv'
        data_measure = []
        data_predict = []

	rospy.init_node("log_data")
	rospy.loginfo("Logging pos data")	
        
	sub = rospy.Subscriber("/obj_cdnts", numpy_msg(Floats), pos_callback, queue_size = 1)
	sub2 = rospy.Subscriber("/Kalman_pred", pos_vector, kalman_callback, queue_size = 1)
        rospy.spin()
        
	print("writing data")
	df1 = pd.DataFrame(data_measure, columns = ['x_measure','y_measure'], dtype = float) 	    
  	df2 = pd.DataFrame(data_predict, columns = ['x_predict','y_predict'], dtype = float)
        df = df1.join(df2)
	df.to_csv(file_path)
