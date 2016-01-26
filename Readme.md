# People Fusion Node
This node is designed to fuse multiple nodes publishing people_detections messages(cob_perception_msgs::DetectionArray) and fuse these detections together.

The used detectors need to be defined within /cfg/detector_config.yaml with to following rules:
- Every parameters has to be defined since there are no default parameters.
- Each detector needs a unique name which needs to be identical with the detector field of the cob_perception_msgs::Detection message

## Offene Fragestellungen
- Empty detections lack the detector information