Build Procedure
===============

Download the source for the dependency
`git clone https://github.com/ipa320/cob_perception_common.git`
Dowload the source for this code
`git clone https://github.com/ipa320/cob_people_perception.git`

Add the paths to these files into your `~/.bashrc` so that ROS knows that they exist. For example, if you cloned the file into your home directory, you would add the following lines to your `~/.bashrc`. Make sure it's after the `source` declaration you made when you installed ROS:
<br>`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/cob_perception_common`
<br>`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/cob_people_perception`

At this point your going to have to update your `~/.bashrc` either by closing your current terminal and opening a new one or using the command ` bash --login`

Finally, go to the `cob_people_detection` directory by executing the command
`roscd cob_people_detection`
and build the package and all of its dependencies by typing
`rosmake`. 

If you cannot execute `roscd` then you probably did the step of adding the files to your `~/.bashrc` incorrectly. If you get an error while trying to run `rosmake` about not having sufficient permissions to make the `build` directory, then switch to the root profile by executing `sudo -s` and the try running `rosmake` again.

To Run
======
Please see the [wiki](http://wiki.ros.org/cob_people_detection).
