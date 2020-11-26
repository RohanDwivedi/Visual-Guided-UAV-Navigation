
# importing OpenCV
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

# importing ros packages
import rospy
from sensor_msgs.msg import Image
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError

# importing packages from TrainYourOwnYOLO Package #
sys.path.append('../../TrainYourOwnYOLO/2_Training/src/')
from keras_yolo3.yolo import YOLO, detect_video
sys.path.append('../../TrainYourOwnYOLO/Utils/')
from utils import load_extractor_model, load_features, parse_input, detect_object

# importing misc packages
#from PIL import Image
import numpy as np
import pandas as pd

#import weights and classes for the model
model_weights = "../../TrainYourOwnYOLO/Data/Model_Weights/trained_weights_final.h5"
model_classes = "../../TrainYourOwnYOLO/Data/Model_Weights/data_classes.txt"
anchors_path  = "../../TrainYourOwnYOLO/2_Training/src/keras_yolo3/model_data/yolo_anchors.txt"

def img_callback(img_msg):
 '''
 ->subscribe to the bebop/img_raw topic
 ->publish img with bounding box draw around the detect objects(s) 
 ->publish bouding box coordinate data 
 '''
 rospy.loginfo(img_msg.header)
 try:
  cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8") # convert image in ros msg format to opencv format
 except CvBridgeError as e:
  rospy.logerr("CvBridge Error: {0}".format(e))
 
 rospy.loginfo("Hey!! Node Received Image, starting detection !")
 pred = detector(cv_image) # call detector function
 pred_image = drawBox(cv_image,pred) 
 pub_image.publish(bridge.cv2_to_imgmsg(pred_image, encoding="passthrough"))
 data = np.array(pred,dtype = np.float32) # numpy.array.flatten() converts 2d array to 1d array since numpy_msg format doesnt support 2d arrays.
 pub_data.publish(data.flatten()) 

def drawBox (cv_img,pred):
    '''
    ->draws bounding box over the predictions
    '''
 # pred coordinates format xmin 0 ,ymin 1,xmax 2,ymax 3
 # bounding box start point top left corner, end point bottom right corner
 
    detections = len(pred)
    for i in range (0, detections): 
        cv_img = cv2.rectangle(cv_img,(pred [i][0],pred[i][3]),(pred[i][2],pred[i][1]),(0,0,255),2)
    return cv_img

def detector(img):
   '''
   ->takes input image from the callback 
   ->YOLOV3 detects the object in the image 
   ->returns the coordinates of the prediction
   ''' 
   img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
   img = cv2.medianBlur(img,5) # remove noise from the image to reduce false detections
   pred, img = detect_object(yolo, img, save_img = False) # call the YOLOV3 detector
   rospy.loginfo('Prediction bounding box coordinates below: ')
   rospy.loginfo(pred)
   return pred

if __name__ == '__main__': 
	
# define YOLO detector
	yolo = YOLO(
       **{
    "model_path": model_weights,
    "anchors_path": anchors_path,
    "classes_path": model_classes,
    "score": 0.25,
    "gpu_num": 1,
    "model_image_size": (416, 416),
         }
	)	

	rospy.init_node("object_detection")
	rospy.loginfo("Detection Node started")
	bridge = CvBridge()
	sub = rospy.Subscriber("/bebop/image_raw", Image, img_callback, queue_size=1, buff_size=2**32) # dont mess with buffer size, causes drop in fps
	pub_image = rospy.Publisher("/obj_image", Image, queue_size =1)
	pub_data = rospy.Publisher("/obj_cdnts", numpy_msg(Floats), queue_size = 100)
	rospy.spin()

