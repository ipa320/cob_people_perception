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

Alternative if you wish to have tf published for skeleton tracker:  

Ensure than the file: `cob_people_perception/cob_openni2_tracker/launch/body_tracker_params.yaml` has the `drawFrames` to true.  

Run the `openni2_launch`:  
```
roslaunch openni2_launch openni2.launch depth_registration:=true
```  

Run the `cob_openni2_tracker`:  
```
roslaunch cob_openni2_tracker body_tracker_nodelet.launch
```
