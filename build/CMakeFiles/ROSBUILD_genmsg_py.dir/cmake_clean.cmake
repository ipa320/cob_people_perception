FILE(REMOVE_RECURSE
  "../src/cob_people_detection/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/cob_people_detection/msg/__init__.py"
  "../src/cob_people_detection/msg/_FaceDetectorAction.py"
  "../src/cob_people_detection/msg/_FaceDetectorGoal.py"
  "../src/cob_people_detection/msg/_FaceDetectorActionGoal.py"
  "../src/cob_people_detection/msg/_FaceDetectorResult.py"
  "../src/cob_people_detection/msg/_FaceDetectorActionResult.py"
  "../src/cob_people_detection/msg/_FaceDetectorFeedback.py"
  "../src/cob_people_detection/msg/_FaceDetectorActionFeedback.py"
  "../msg/FaceDetectorAction.msg"
  "../msg/FaceDetectorGoal.msg"
  "../msg/FaceDetectorActionGoal.msg"
  "../msg/FaceDetectorResult.msg"
  "../msg/FaceDetectorActionResult.msg"
  "../msg/FaceDetectorFeedback.msg"
  "../msg/FaceDetectorActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
