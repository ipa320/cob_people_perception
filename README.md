Travic-CI: [![Build Status](https://travis-ci.org/ipa320/cob_people_perception.svg?branch=indigo_dev)](https://travis-ci.org/ipa320/cob_people_perception)

Collection of packages for detecting and identifying faces, detecting and tracking humans with RGB-D sensors or laser scanners, and fusing detections.

Following functionalities are found in the packages:
cob_people_perception: 
cob_people_detection: Detects persons through head and face detection and identifies the detected faces. The results are tracked over time to increase confidence. The system is optimized to be used with a Microsoft Kinect or Asus Xtion Pro Live sensor but could also handle different sensors as long as depth and color images are provided. See http://wiki.ros.org/cob_people_detection for details


cob_dual_leg_tracker: Tracks persons as pairs of legs in a laser scan
cob_leg_detection: detects and tracks legs in a laser scan (used by cob_dual_leg_tracker)
cob_people_tracking_filter: tracking of detections (used by cob_leg_detection and cob_dual_leg_tracker)

cob_people_fusion: fuse people deections from various sources 

cob_openni2_tracker: Unused??
libnite2: Unused??

Please also have a look at our more detailed documentation at http://wiki.ros.org/cob_people_detection .

Quick Start
-----------
Kinnect detections: see cob_people_detection/readme

Laser scan based detection: see cob_dual_leg_tracker/readme

Fusion: see cob_people_fusion/readme 
