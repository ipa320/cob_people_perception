Please also have a look at our more detailed documentation at http://wiki.ros.org/cob_people_detection .

Installation
-----------

1. Clone cob_openni2_tracker in catkin workspace:

	git clone https://github.com/ros-drivers/cob_openni2_tracker.git

2. Install ros packages and dependencies (or replace indigo by your favorite ROS version):

	ros-indigo-openni2-camera <br>
	ros-indigo-openni2-launch <br>
	ros-indigo-rgbd-launch <br>
	ros-indigo-nodelet
	
  or clone the git repositories:
  
	git clone https://github.com/ros-drivers/openni2_camera.git <br>
	git clone https://github.com/ros-drivers/openni2_launch.git <br>
	git clone https://github.com/ros-drivers/rgbd_launch.git <br>
	
3. Download and install libopenni2.

4. Run catkin_make


Quick Start
----------
     
Run the launch file:

	roslaunch cob_openni2_tracker body_tracker_nodelet.launch

