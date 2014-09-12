Rect head_detection		# bounding box of detected head (coordinates in relation to whole image coordinate system)
sensor_msgs/Image color_image	# color image patch at bounding box head_detection (i.e. same size as head region, not only face region)
sensor_msgs/Image depth_image	# depth image patch at bounding box head_detection
Rect[] face_detections		# faces detected in color image inside the head's bounding box (coordinates in relation to head's bounding box)

