Travic-CI: [![Build Status](https://travis-ci.org/ipa320/cob_people_perception.svg?branch=indigo_dev)](https://travis-ci.org/ipa320/cob_people_perception)


Packages for people detection and identification with a Kinect.

Please also have a look at our more detailed documentation at http://wiki.ros.org/cob_people_detection .

Quick Start
-----------
see cob_people_detection/readme

Useful Information
-----------
- If you get errors when you `catkin_make` then ensure that in your `~/catkin_ws/src/` you have both `cob_perception_common` and `cob_people_perception`.
Also ensure in both `cob_people_perception` and `cob_perception_common` you ran the following command:
```
rosdep install -r --from-paths .
```
Then run again `catkin_make`.
