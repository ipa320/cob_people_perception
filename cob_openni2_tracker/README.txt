Please also have a look at our more detailed documentation at http://wiki.ros.org/cob_people_detection .

Installation
-----------

1. Clone cob_openni2_tracker in catkin workspace:

   $ git clone https://github.com/ros-drivers/cob_openni2_tracker.git

2. Install ros packages and dependencies:

	ros-hydro-openni2-camera
	ros-hydro-openni2-launch
	ros-hydro-rgbd-launch
	ros-hydro-nodelet 

or clone the git repositories:

   $ git clone https://github.com/ros-drivers/openni2_camera.git
   $ git clone https://github.com/ros-drivers/openni2_launch.git
   $ git clone https://github.com/ros-drivers/rgbd_launch.git

3. Download and install OpenNI 2.2.

4. Run catkin_make


Quik Start
----------
     
Run the launch file:

   $ roslaunch cob_openni2_tracker body_tracker_nodelet.launch

