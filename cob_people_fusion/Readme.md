# People Fusion Node
This node is designed to fuse multiple nodes publishing people_detections messages(cob_perception_msgs::DetectionArray) and fuse these detections together. The current implementation associates the detections to each other (ignoring the timestamp difference) and increases the detection score when the detection is seen by multiple sensors and at multiple time steps. The state information is currently not fused; saved state information ist replaced by newer state information. State information from past times is not taken into account.

The used detectors need to be defined within /cfg/detector_config.yaml with to following rules:
- Every parameters has to be defined since there are no default parameters.
- Each detector needs a unique name which needs to be identical with the detector field of the cob_perception_msgs::Detection message

Quick Start
-----------
make sure that the topics listed in cfg detector_config_all.yaml are being published and that the above mention rules are taken into consideration.

Start the fusion with: roslaunch cob_people_fusion people_fusion.launch


In RVIZ the detections are highlighted using cylinders with different colors. Each color represents a detection type.

## Open Questions
- Empty detections lack the detector information. Note: Currently fixed by wrapping within own message type

## Known Problems
- There is a issue with a rotated sensorring when using the kinect. This may be related to a wrong tf transformation. It is recommended to develop with the sensorring facing the front. This bug must be fixed.
- On occasions it can happen that a person is represented by multiple trackers which update by detections of different detectors. This is caused by a bias of the detections. To tackle this issue trackers which are spatially extreme close should be fused on such conditions.

## Next Todos
- Currently the fusion node relies on the detections to be in the "odom_combined" frame which is sufficient constant to perform tracking. Here a automatic transformation should take place if the detector uses a different frame.
- The tracking is only roughly implemented by using a global nearest neightbor association along with a euclidean distance condition. Better tracking performance can be expected if more precise model such as linear velocity is implemented and the tracking is performed using kalman filters.
- If a underlying detector allready performs tracking itself this information should be useful. E.g. the association costs should be increased if a label_switch would occur or lower if the label(s) of the underlying detectors would stay the same.
