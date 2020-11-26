# Implementing machine-vision to track moving objects
## Developing software for visual guided UAV Navigation

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
in this thesis. Fairly good tracking was observed on the simulated tests, the
performance of the tracking and its limitation is thoroughly discussed.
The document is organised in a way that we first discuss what and why
in the introduction section. It is followed by an extensive literature review
to obtain an understanding of the problem, concepts and available solutions
and their limitations. Further, the document gives a brief explanation of the
technical concepts required to be known before attempting to understand
the methodology discussed in the section that follows. The final sections
deal with illustrating the performance of the implementation followed by an
analysis of the weaknesses and challenges. The document concludes with a
note on the future direction of this work.
