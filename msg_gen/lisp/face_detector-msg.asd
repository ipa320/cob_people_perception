
(cl:in-package :asdf)

(defsystem "face_detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :people_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FaceDetectorActionFeedback" :depends-on ("_package_FaceDetectorActionFeedback"))
    (:file "_package_FaceDetectorActionFeedback" :depends-on ("_package"))
    (:file "FaceDetectorAction" :depends-on ("_package_FaceDetectorAction"))
    (:file "_package_FaceDetectorAction" :depends-on ("_package"))
    (:file "FaceDetectorGoal" :depends-on ("_package_FaceDetectorGoal"))
    (:file "_package_FaceDetectorGoal" :depends-on ("_package"))
    (:file "FaceDetectorResult" :depends-on ("_package_FaceDetectorResult"))
    (:file "_package_FaceDetectorResult" :depends-on ("_package"))
    (:file "FaceDetectorActionResult" :depends-on ("_package_FaceDetectorActionResult"))
    (:file "_package_FaceDetectorActionResult" :depends-on ("_package"))
    (:file "FaceDetectorFeedback" :depends-on ("_package_FaceDetectorFeedback"))
    (:file "_package_FaceDetectorFeedback" :depends-on ("_package"))
    (:file "FaceDetectorActionGoal" :depends-on ("_package_FaceDetectorActionGoal"))
    (:file "_package_FaceDetectorActionGoal" :depends-on ("_package"))
  ))