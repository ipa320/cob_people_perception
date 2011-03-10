FILE(REMOVE_RECURSE
  "../src/cob_people_detection/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
