# Visual guided UAV Navigation
##  Machine-vision to track moving objects

### PROJECT ABSTRACT

Quadrotor navigation using computer vision is an open research problem. To
be able to track objects using quadrotors has very important applications.
This thesis discusses a project to develop a computer vision-based tracking
mechanism to detect and track objects from a commercially available quad-
rotor just based on a monocular camera. The project utilises YOLOV3 to
detect objects by training the model on synthetic training data and imple-
ments a Kalman filter and PID to track the vehicle. The project uses the
sphinx simulation environment to test the implementation instead of test-
ing in the real world. It utilises the open-source implementations of bebop
autonomy; a ROS driver developed by autonomy lab of Simon Fraser based
on the Ardrone SDK provided by parrot. It also uses an open-source imple-
mentation of YOLOV3 to train the model and modifies its detector to pass
the coordinates of the bounding box as a ROS message. We discuss a method
for the inverse projection of 2d image coordinates to 3d world coordinates
in this thesis. 

### MAJOR DEPENDENCIES

1. Ros Kintetic Kame  - http://wiki.ros.org/kinetic
2. Bebop Autonomy ROS Driver for parrot bebop 2 - https://bebop-autonomy.readthedocs.io/en/latest/
3. Parrot Sphinx simulation environment based on Gazebo - https://developer.parrot.com/docs/sphinx/firststep.html
4. Build Eigen Library for linear algebra operations https://gitlab.com/libeigen/eigen.git
5. Build CVBridge  http://wiki.ros.org/cv_bridge 

### TROUBLESHOOT
1.  https://forum.developer.parrot.com/t/using-bebop-autonomy-with-sphinx-on-same-machine/6726/5
2.  https://answers.ros.org/question/290660/import-cv2-error-caused-by-ros/
3.  YOLO Model from TrainYourOwnYOLO repository was retrained on synthetic dataset of the vehicle used (polaris ranger) for the project which was generated using simulation in Gazebo. Training and testing dataset is not included due to its size; the trained weights can be directly used with the detector. The detector was modified to work with ROS script for this project. 

### RUNNING THE PROJECT
1. Build the dependencies
2. clone the repo and catkin build
3. roslaunch visual_nav visualNav.launch

