cob_people_perception
===========

## ROS Distro Support

|         | Indigo | Jade | Kinetic |
|:-------:|:------:|:----:|:-------:|
| Branch  | [`indigo_dev`](https://github.com/ipa320/cob_people_perception/tree/indigo_dev) | [`indigo_dev`](https://github.com/ipa320/cob_people_perception/tree/indigo_dev) | [`kinetic_dev`](https://github.com/ipa320/cob_people_perception/tree/kinetic_dev) |
| Status  |  supported | not supported |  supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=cob_people_perception) | [version](http://repositories.ros.org/status_page/ros_jade_default.html?q=cob_people_perception) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=cob_people_perception) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.org/ipa320/cob_people_perception.svg?branch=indigo_dev)](https://travis-ci.org/ipa320/cob_people_perception)

## ROS Buildfarm

|         | Indigo Source | Indigo Debian | Jade Source | Jade Debian |  Kinetic Source  |  Kinetic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| cob_people_perception | [![not released](http://build.ros.org/buildStatus/icon?job=Isrc_uT__cob_people_perception__ubuntu_trusty__source)](http://build.ros.org/view/Isrc_uT/job/Isrc_uT__cob_people_perception__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__cob_people_perception__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__cob_people_perception__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__cob_people_perception__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__cob_people_perception__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__cob_people_perception__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__cob_people_perception__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__cob_people_perception__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__cob_people_perception__ubuntu_xenial__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__cob_people_perception__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__cob_people_perception__ubuntu_xenial_amd64__binary/) |


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
