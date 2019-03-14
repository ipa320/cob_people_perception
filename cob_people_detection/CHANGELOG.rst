^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_people_detection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2019-03-14)
------------------
* Merge pull request `#78 <https://github.com/ipa320/cob_people_perception/issues/78>`_ from ipa-fxm/fix/pluginlib_macro
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* Merge pull request `#75 <https://github.com/ipa320/cob_people_perception/issues/75>`_ from ipa320/remove_obsolete_cmake_modules
  remove obsolete dependencies to cmake_modules
* remove obsolete dependencies to cmake_modules
* Merge pull request `#74 <https://github.com/ipa320/cob_people_perception/issues/74>`_ from ipa-rmb/indigo_dev
  OpenCV 2+3 compatibility cleanup
* Merge pull request `#1 <https://github.com/ipa320/cob_people_perception/issues/1>`_ from ipa-rmb/indigo_dev
  Fixes for opencv 2 in indigo
* updates for having the software ready with opencv2 and 3
* Merge pull request `#73 <https://github.com/ipa320/cob_people_perception/issues/73>`_ from ipa-rmb/indigo_dev
  Fixed bug with untrained system not starting the people detection client
* Merge branch 'indigo_dev' of https://github.com/pierrekilly/cob_people_perception into kinetic_dev
* fixed bug with untrained system not starting the people detection client
* Simplification
* This should also work and is simpler.
* adapted people detection for retro compatibility with OpenCV 2
* Make the repo compile under ROS Lunar (OpenCV 3.2.0)
  This Github issue helped a lot:
  https://github.com/wg-perception/people/issues/44
* Merge pull request `#70 <https://github.com/ipa320/cob_people_perception/issues/70>`_ from ipa-fxm/add_launch_arguments
  Add launch arguments
* introduce launch arguments to start parts of the pipeline
* Publishers in face detector only publish if subscribers are present
* Update head_detector_node.cpp
* Merge pull request `#68 <https://github.com/ipa320/cob_people_perception/issues/68>`_ from ipa-fxm/fix/return_value_handling
  fix return value handling
* Merge branch 'indigo_dev' of github.com:ipa320/cob_people_perception into merge
* Update face_recognizer_node.cpp
* Merge branch 'indigo_dev' of github.com:ipa320/cob_people_perception into merge
* added cartesian face coordinate output to face_detector - must be finished with making this output optional on subscribers
* fix return value handling
* Merge pull request `#63 <https://github.com/ipa320/cob_people_perception/issues/63>`_ from ipa-fxm/use_external_project
  Multi distro travis indigo
* obey read-only file system in cob_people_detection
* Merge pull request `#62 <https://github.com/ipa320/cob_people_perception/issues/62>`_ from ipa-rmb/indigo_dev
  added libxi and libxmu as dependencies for ros build farm
* cleaned up spaces/tabs
* Merge pull request `#48 <https://github.com/ipa320/cob_people_perception/issues/48>`_ from ipa-rmb/indigo_dev
  Bugfix for running openni2_tracker and openni2_launch simultaneously
* updated readmes
* Merge branch 'indigo_dev' of github.com:ipa-rmb/cob_people_perception into indigo_dev
* fixed topic names in people client
* Merge pull request `#38 <https://github.com/ipa320/cob_people_perception/issues/38>`_ from ipa-fmw/indigo_dev
  set correct frame_if for tracker
* Merge branch 'fix/output_messages' into indigo_dev
* Merge branch 'indigo_dev' of https://github.com/ipa320/cob_people_perception into indigo_dev
* fix frame_id for empty detection arrays
* Merge pull request `#35 <https://github.com/ipa320/cob_people_perception/issues/35>`_ from ipa-fmw/fix/output_messages
  fix output messages
* fix frame_id for header
* fix header of output messages
* Merge branch 'fix/output_messages' of github.com:ipa-fmw/cob_people_perception into indigo_dev
* use nodename as detector (neccessary for multiple detectors)
* fix headers
* Merge pull request `#34 <https://github.com/ipa320/cob_people_perception/issues/34>`_ from ipa-fmw/fix/remove_global_ns
  remove global namespaces
* Update detection_tracker.launch
* Update face_capture.launch
* fix systax
* fix launch files without global ns
* delete global namespaces, use private nodehandles and adapt remappings in launch files
* remove global namespaces
* Merge pull request `#33 <https://github.com/ipa320/cob_people_perception/issues/33>`_ from ipa-fmw/fix/frame_ids
  use frame_ids from input message
* use frame_ids from input message
* Merge pull request `#31 <https://github.com/ipa320/cob_people_perception/issues/31>`_ from ipa-fmw/indigo_dev
  alphabetic order
* Merge branch 'indigo_dev' of https://github.com/ipa-fmw/cob_people_perception into indigo_dev
  Conflicts:
  cob_leg_detection/CMakeLists.txt
  cob_leg_detection/package.xml
* alphabetic order
* Merge pull request `#30 <https://github.com/ipa320/cob_people_perception/issues/30>`_ from ipa-rmb/indigo_dev
  conversion to package format 2 and little fixes
* cleaning and conversion to package format 2
* added missing header to pose in /cob_people_detection/face_recognizer/face_recognitions
* added missing install commands
* Merge pull request `#1 <https://github.com/ipa320/cob_people_perception/issues/1>`_ from ipa320/indigo_dev
  updates from ipa320
* Merge pull request `#27 <https://github.com/ipa320/cob_people_perception/issues/27>`_ from ipa-rmb/indigo_dev
  updates from rmb
* changed camera namespace to standard
* bugfixes for new image_flip version
* removed cob_image_flip from standard configuration (most users won't need it)
* hopefully corrected dependencies to message generation in cob_perception_msgs
* fixing dependencies
* updated package.xml information
* Merge branch 'hydro_dev' of github.com:rmb-om/cob_people_perception into rmb-om-hydro_dev
* little changes
* working on functions for image transform
* added body_tracker nodelet
* added install flag to prevent repetition of downloading and building libnite
* merge with rmb indigo_dev
* bugfixing
* fixed an install bug with a config file
* Merge branch 'indigo_dev' into rmb-om-hydro_dev
* merge with rmb-om
* tarball md5sum added and link points to ipa320
* merge with rmb/indigo_dev
* a few updates for people_perception from rmb-om
* Merge branch 'hydro_dev' of github.com:rmb-om/cob_people_perception into rmb-om-hydro_dev
* bugfixing cob_people_perception
* fixed dependencies
* merge with rmb-om
* moved the msgs package to cob_perception_common/cob_perception_msgs
* fixed bug with wrong brackets format
* added recent changes from groovy_dev version
* improved the background removal procedure in face detector and added a complimentary method to keep the filtered background region after radiometric illumination normalization (keep those white pixels white)
* fixed a potential source of errors on image recording
* merge with latest rmb status
* Merge pull request `#23 <https://github.com/ipa320/cob_people_perception/issues/23>`_ from ipa-rmb/groovy_dev_catkin
  package now transferred to groovy catkin: build and function tested successfully
* upadted readme
* merging
* Merge pull request `#22 <https://github.com/ipa320/cob_people_perception/issues/22>`_ from ipa-rmb/groovy_dev
  fixed a bug in face_recognizer.cpp
* fixed a bug in face_recognizer.cpp
* fixed a bug in face_recognizer
* unified coding style (indentation, whitespaces, appearance)
* package now transferred to catkin: build and function tested successfully
* catkinizing cob_people_perception
* catkinizing cob_people_perception
* Merge pull request `#21 <https://github.com/ipa320/cob_people_perception/issues/21>`_ from max90727/patch-1
  Update manifest.xml
* Update manifest.xml
* Merge pull request `#15 <https://github.com/ipa320/cob_people_perception/issues/15>`_ from ipa-rmb/groovy_dev
  Some feedback on documentation
* Update readme
* specified some documentation items
* Merge pull request `#12 <https://github.com/ipa320/cob_people_perception/issues/12>`_ from Seanny123/patch-1
  I will merge your instructions and correct the inaccuracies. Thanks for your valuable contribution!
* Update readme
  Corrected openni_launch command.
* Merge pull request `#7 <https://github.com/ipa320/cob_people_perception/issues/7>`_ from ipa-rmb/groovy_dev
  latest update from rmb
* preparations for release
* Merge pull request `#1 <https://github.com/ipa320/cob_people_perception/issues/1>`_ from accompany-cob3-6/groovy_dev
  Groovy dev
* tuned parameters for operation
* merge with recent code from rmb
* parameter adaptations
* fixed a bug in tracking_eval_node.cpp
* removed an output, changed back parameters to standard
* fixed some bugs
* reorganized launch files for more intuitive usage and simpler argument definition
* merge with rmb
* fixed bugs in tracking evaluator
* tracking evaluator added + launchfile
* cleaned up the launch files for the nodelet version with use of parameters, added the functionality to remove background from detected face images so that background neither affects training data nor recognition data
* merge with own recent work on robot
* small changes
* added a parameter for controlling the publish behavior of currently not visible faces which are still tracked
* commit before trying on robot
* completed published detection with orientation
* added face_align_test
* moved and renamed ssaNEW->face_recognition_algorithms_test and fn_test->face_normalizer_test. moved db_eval to ros/scripts and removed classifier selection from gui.
* commit for completed code restructuring 01
* introduced rosbag_mode  - error in detection tracking is avoided
* fixed bugs in nodelet launchfiles
* load and save works for 2D methods as well
* loading and saving models works for 1D methods
* saving model works - reading tbd
* added parameters and changed behavior when training fails with ROS_ERROR
* changed face_recognition to face_recognizer_algorithms and put it in namespace ipa_PeopleDetector
* setting for data_storage_directory can now be set in launch file
* fixed bug in face_recognizer
* added face_recognition
* started doxygen documentation in face_recognition.h
* face_recognizer compiles with new structure - testing and cleaning remains
* moved files a lot
* PCA 2d works for new structure
* 1D methods work also with dynamic allocation
* Fisherfaces works in new structure
* Eigenfaces works in new structure
* restructuring subspace analysis - therefore added ssaNEW files
* added doxygen docu to face normalizer
* removed virtual camera ( obsolete)
* cleaned up face_normalizer
* first modifications - cleanup
* adapting for groovy
* introduced EIGENSOLVER - way better Fisherfaces
* changed timer instantiation
* added Boost version number to CMAKELISTS
* Merge remote branch 'origin/experimental' into experimental
* before merging
* nodelets working on cob3-3
* before use on robot
* before reverting
* accelarated recognition time for 2D methods
* end of a day commit
* 2D LDA and PCA work
* some small changes
* merged with rmb
* before merge with rmb
* Merge pull request `#6 <https://github.com/ipa320/cob_people_perception/issues/6>`_ from ipa-rmb/fuerte_dev
  same updates as for electric_dev: merge with new code from goa-tz, tested for electric and fuerte
* Merge pull request `#5 <https://github.com/ipa320/cob_people_perception/issues/5>`_ from ipa-rmb/electric_dev
  merged with work of goa-tz, tested quite well, works with fuerte as well
* Merge pull request `#4 <https://github.com/ipa320/cob_people_perception/issues/4>`_ from ipa-rmb/master
  merged with work of goa-tz, tested quite well, works with fuerte as well
* merged with latest fuerte adaptations
* junk change
* a couple of adaptations to run people detection on fuerte as well
* attacking the 'unstable' problem of Jenkins
* obviously solved the endless loop bug in munkres (assignment problem sometimes hang up on certain costs matrices)
* fixed the crash on adding new data after restarting the node, tracking bug remains to be solved
* fixed merge conflict
* a few bugfixes
* merge with latest code
* testing
* merge with goa-tz
* bugfixing
* fixed merge bugs
* configuration works
* merged with experimental_fuerte branch!
* Merge remote branch 'origin/experimental_fuerte' into experimental_fuerte
* pre-megre with fuerte branch
* pre merge
* little debugging, code styling
* merge with goa-tz
* small changes
* works on ipa fuerte pc
* Merge remote-tracking branch 'origin/experimental_fuerte' into experimental_fuerte
* before merge
* optimized face_normalizer
* Merge branch 'experimental' into experimental_fuerte
* Merge remote-tracking branch 'origin/experimental' into experimental
* before merge
* adapted db eval
* Merge branch 'experimental' into experimental_fuerte
* fixed merge bug
* works with ocv fisher
* merged with home
* changed ill corr
* experimental commit
* 2office
* fixed namespace problem for cob_image_flip
* detection tracker improved with global optimal assignment of previous and current detections using Hungarian method, needs more testing
* improved illumination correction , included yale protocols in db gui
* back to office
* worked on face normalizer - weekend
* integrated xyz normalizing workflow in ssa_test and db_eval GUI
* end fo day commit
* calibrated Kinect3d database
* back2office
* added yale and unkown testing
* working state
* test state
* save ans load interface works
* loadModel works+ working on new interface
* added random forest - parameter tuning tbd
* added scene publisher
* working on scene publisher
* fixed segfault
* implemented threshold verification of all classification methods
* implemented first version of new thresholing
* bugfix
* implemented single processing and cross validation for gui
* introducesd unknown to gui and ssa
* normalization works - code restructuring to be done
* working on normalizer
* y-axis still unresolved but rest works
* working on pose correction
* implemented threshold
* showable commit , geometric normalization deactivated
* kurzfristig
* transfer commit
* fixed bug in fn test
* face radiometry normalization breakthrough - bug fixed in eval tool
* small bugfixes
* included threading in gui
* configuration in renamed script
* changed ssatest to command line interface
* added print eval file
* added leave out halt
* load_script with new structure and protocols
* db preparation script
* color and/or depth processing works
* integrated normalizer in classification -still bug, whenn normalization succeds
* classification works - normalizing and reducing tbd
* added xml-test
* cleaned up face_recognizer --- bg in classes for depth
* added resizing to depth in normalizer
* piped depth images until init of model
* integrated depth in CAPTURING only
* started intgration of depth map in capture process
* integrated fidheigfaces in recognizer
* integrated FishEigFaces
* face normalizer has normalized depth map as output
* included normalized depth map
* working on homography processing in face normalizer
* added file visualization to db eval script gui
* multiple probe images possible
* integrated visualization and processing in script
* modified python scripts
* added evaluation script
* implemented fallback to eigenfaces and diffs if necessary, included deallocation functions
* classifier works - label bug fixed
* removed average from projection - knn and svm still dont work
* included SVM and KNN - but bug when multiple classes
* integrated fisherfaces in face recognizer
* Fisherfaces works - testing to be done
* avg error prevents lda from working
* classification implemented/integrated - testing to be done
* integrated ss-calculation,projection,DFFS in face_recognizer-->classification missing
* bug in retrive function in ssa
* integrated in face_recognizer
* minimized copyTo
* pca works with decompose and decompose2
* fixed ssa_test imread
* integrating cv::PCA
* merged with home fuerte
* deleted croparray
* solvePnP discrepancy
* temp
* Merge remote branch 'origin/home_fuerte' into merge
* added helper
* Merge remote branch 'origin/home_fuerte' into merge
* added decomp and ssa_test
* merged with home_fuerte
* enum in virtual camera..
* back to the office
* reconstruction
* changed SSA to subspace_analysis
* added namespace ssa , included eigen decomp
* worked in SSA
* added SSA
* adding vc_test
* ported to ROS-fuerte
* class restructured - minimum search areas adaptive to input size
* cleaned up class
* cleaned up class
* changed macros to types - added namespace -..
* Merge pull request `#3 <https://github.com/ipa320/cob_people_perception/issues/3>`_ from ipa-fmw/master
  fix for launch file test: setting default value for arg
* fix for jenkins launch files tests: setting default value for arg
* pipeline works except display
* transfer to recognizer node works - runtime error
* inserted pull request, should run in fuerte now
* Merge pull request `#2 <https://github.com/ipa320/cob_people_perception/issues/2>`_ from mintar/master
  Update CMakeLists.txt to be compatible with fuerte, switch tinyxml to rosdep
  Thanks a lot for your useful contribution!
* modularized fn test to test folder
* added captureScene function
* works with virtual camera  - panchrom filtering activated
* added virtual camera class
* switched normalization steps, works with resizing
* rgb filtering works
* virtual view works on most scenes - filtering implemented
* PnP better with ident face correct result
* fixed flipping - works with ident face - apart from aliasing
* fixed flipping - works with ident face - apart from aliasing
* works on rgb image
* detected seg fold - works on grayscale img
* detected seg fold - works on grayscale img
* end of day - segfault
* added bugfixing outputs, specified desired startup configuration in kinect driver
* end of day
* 2 faces appead in reproj image
* 2 faces appead in reproj image
* woriking on reprojection
* switch tinyxml to rosdep dependency
  In fuerte, the tinyxml ROS package doesn't exist any more; instead, a
  rosdep dependency is required. This has only been tested on fuerte, so
  it might break compilation in previous versions.
* link against boost::signals
  this is required to compile under fuerte
* added missing dependency to image view
* added suitable remap for points topic (cam3d/rgb/points)
* parameters
* parametrized screen outputs
* face_normalizer node recieves images
* added nodelets for data pre-processing
* added further outputs for timing measurements
* renamed variables in face_normalizer
* NOT WORKING restructuring of face normalizer
* tested and tuned people detection on robot
* added timing measurements, tweaks for speed up
* added timing analysis
* changes for usage with image flip and on robot
* modifications done at uh
* cleaning
* cleaned class FN and added DCT
* changed grid position of eyes
* dynamic norm face works
* implemented hsv hist eq and working onb dyn_face_norm
* restructuring and integrating in recognition process
* normalization integrated in capturing process
* restructuring in order to normalizer images
* normalizing images with affine trafo works
* added face normalizer
* experimenting with detection
* added isnan check in display node
* Merge branch 'master' into review-320
* changed the people detection display node to get the image from an incoming point cloud
* worked on affine trafo
* before new branch
* increased the eigenvalues array to fix bug in oneiric
* maybe fixed oneiric error
* maybe fixed the oneiric bug
* little corrections in comments etc.
* Merge pull request `#1 <https://github.com/ipa320/cob_people_perception/issues/1>`_ from ipa320/review-320
  New structure
* added further missing files to the repo
* added two missing files to the repo
* additional stack.xml and manifest.xml description
* process of restructuring cob_people_detection finished, current version tested successfully
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* in the process of restructuring cob_people_detection
* merge
* output debug hints
* removed unnecessary files in include and src, added some comments, moved bin and lib to root folder
* updated manifest.xml and stack.xml
* added openni.launch for people detection
* fixed people_detection, function tested.
* removed several dependencies, test of function succesful
* Merge branch 'master' of github.com:ipa-rmb/cob_people_perception
* moved to new repo
* Contributors: Felix Messmer, Florian Weisshardt, Florian Weißhardt, Martin Günther, Pierre Killy, Richard Bormann, Seanny123, Thomas Zwoelfer, Thomas Zwölfer, accompany-cob3-6, ipa-fmw, ipa-fxm, ipa-goa-tz, ipa-jsf, ipa-rmb, max90727, rmb-om, rmb-tz, tom
