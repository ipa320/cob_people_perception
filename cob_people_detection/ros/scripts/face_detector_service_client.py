#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_people_detection')

import sys

import rospy
from cob_people_detection.srv import *

def recognition_service_client(running, doRecognition, display):
    rospy.wait_for_service('/cob_people_detection/face_detection/recognize_service_server')
    try:
        recognition_service = rospy.ServiceProxy('/cob_people_detection/face_detection/recognize_service_server', Recognition)
        res = recognition_service(running, doRecognition, display)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [running, doRecognition, display]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        running = bool(sys.argv[1])
        doRecognition = bool(sys.argv[2])
        display = bool(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print recognition_service_client(True, True, True)

