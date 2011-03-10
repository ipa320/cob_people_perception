FILE(REMOVE_RECURSE
  "../src/cob_people_detection/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/cob_people_detection/FaceDetectorAction.h"
  "../msg_gen/cpp/include/cob_people_detection/FaceDetectorGoal.h"
  "../msg_gen/cpp/include/cob_people_detection/FaceDetectorActionGoal.h"
  "../msg_gen/cpp/include/cob_people_detection/FaceDetectorResult.h"
  "../msg_gen/cpp/include/cob_people_detection/FaceDetectorActionResult.h"
  "../msg_gen/cpp/include/cob_people_detection/FaceDetectorFeedback.h"
  "../msg_gen/cpp/include/cob_people_detection/FaceDetectorActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
