 Installation

1. Clone cob_openni2_tracker and other repositories from Github (hydro-dev branch) in a catkin workspace:

    $ cd catkin_ws/src

   $ git clone https://github.com/ros-drivers/cob_openni2_tracker.git
   $ git clone https://github.com/ros-drivers/openni2_camera.git
   $ git clone https://github.com/ros-drivers/openni2_launch.git
   $ git clone https://github.com/ros-drivers/rgbd_launch.git
   
2.  Run catkin_make 
   
3.  Look for OpeNI2 and NiTE-Linux-x64-2.2 packages in cob_openni2_tracker/include. If empty:

3.1. Download OpenNI2 or clone the OpenNI2 repository from Github:
    
3.1.1. (if downloaded) Download and unpack OpenNI2 2.2.0.33 from www.http://structure.io/openni into your catkin/ws/cob_openni2_tracker/include folder:
     
      $  cd OpenNI
      $ sudo ./install.sh
       
3.1.2 (if cloned) Clone      
       
   $ git clone git@github.com:OpenNI/OpenNI2.git 
    
   $ cd OpenNI
   $ mkdir build
   $ cd build
   $ cmake .. -DBUILD_OPENNI2_DRIVER=ON
   $ make

3.2. Install Nite2 (I used NiTE2 from www.http://eras.readthedocs.org/en/latest/servers/body_tracker/doc/setup.html) and unpack to catkin/ws/cob_openni2_tracker/include folder.
     To unpack .tar.bzz data use:
       
   $ sudo tar -xjvf NiTE-Linux-x64-22.tar.bzz
    
   $ cd NiTE-Linux-x64-2.2
   $ sudo ./install.sh

4. Try to run the launch file:

   $ roslaunch cob_openni2_tracker openni2_tracker.launch

  If there is an error message:

      [ERROR] Could not find data file ./NiTE2/s.dat
       current working directory = ~/.ros
 
  Set the link to ./NiTE2/s.dat (This command run only once !) :
 
     $  rosrun cob_openni2_tracker create_nite2_link.bash 
     
     or
     
     $ ln -s ..catkin_ws/src/cob_openni2_tracker/include/NiTE-Linux-x64-2.2/Redist/NiTE2/ ~/.ros/NiTE2

5. Run an example in ~/NiTE-Linux-x64-2.2/Samples/Bin/
   If these don't work, then something went wrong with the installation. Check your directories and libraries in
   in the CMakeLists.txt file inside the cob_openni2_tracker package. Especially if you cloned OpenNI2 from Github. 
   You will probably need to change and configure the path where 
   CMake will look for OpenNI2 and NiTE2, because it needs to pint to their directories.

