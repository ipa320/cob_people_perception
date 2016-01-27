# People Fusion Node
This node is designed to fuse multiple nodes publishing people_detections messages(cob_perception_msgs::DetectionArray) and fuse these detections together.

The used detectors need to be defined within /cfg/detector_config.yaml with to following rules:
- Every parameters has to be defined since there are no default parameters.
- Each detector needs a unique name which needs to be identical with the detector field of the cob_perception_msgs::Detection message

In RVIZ the detections are highlighted using cylinders with different colors. Each color represents a detection type.

Because the detectors have different detection rates the usage of a timesequencer is indispensable. By nature of this sequencer a delay is introduced into the system.

## Offene Fragestellungen
- Empty detections lack the detector information. Note: Currently fixed by wrapping within own message type

## Bekannte Probleme
- There is a issue with a rotated sensorring when using the kinect. This may be related to a wrong tf transformation. It is recommended to develop with the sensorring facing the front. This bug must be fixed.
