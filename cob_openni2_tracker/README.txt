 Installation

1. Clone cob_openni2_tracker and other repositories from Github (hydro-dev branch) in a catkin workspace:

    $ cd catkin_ws/src

   $ git clone https://github.com/ros-drivers/cob_openni2_tracker.git
   $ git clone https://github.com/ros-drivers/openni2_camera.git
   $ git clone https://github.com/ros-drivers/openni2_launch.git
   $ git clone https://github.com/ros-drivers/rgbd_launch.git
   
2.  Run catkin_make 

3.1. Download OpenNI2 or clone the OpenNI2 repository from Github:
    
	3.1.1. (if downloaded) Download and unpack OpenNI2 2.2.0.33 from www.http://structure.io/openni 
     
      $  cd OpenNI
      $ sudo ./install.sh
       
	3.1.2 (if cloned) Clone      
       
  	 $ git clone git@github.com:OpenNI/OpenNI2.git :
  	 
    
  	 $ cd OpenNI
  	 $ mkdir build
   	$ cd build
   	$ cmake .. -DBUILD_OPENNI2_DRIVER=ON
   	$ make

3.2. Install Nite2 and unpack:
    
   $ cd NiTE-Linux-x64-2.2
   $ sudo ./install.sh

4. Try to run the launch file:

   $ roslaunch cob_openni2_tracker openni2_tracker.launch


5. Run an example in ~/NiTE-Linux-x64-2.2/Samples/Bin/
   If these don't work, then something went wrong with the installation. Check your directories and libraries in
   in the CMakeLists.txt file inside the cob_openni2_tracker package.

//compile pcl
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j4