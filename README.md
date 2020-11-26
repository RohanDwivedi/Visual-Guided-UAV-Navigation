#  Machine-vision to track moving objects
## Visual guided UAV Navigation

### ABSTRACT

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
