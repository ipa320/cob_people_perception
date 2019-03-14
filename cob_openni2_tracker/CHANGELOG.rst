^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_openni2_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'indigo_dev' of github.com:ipa320/cob_people_perception into merge
* Merge pull request `#63 <https://github.com/ipa320/cob_people_perception/issues/63>`_ from ipa-fxm/use_external_project
  Multi distro travis indigo
* fix cob_openni_tracker
* Merge pull request `#62 <https://github.com/ipa320/cob_people_perception/issues/62>`_ from ipa-rmb/indigo_dev
  added libxi and libxmu as dependencies for ros build farm
* cleaned up spaces/tabs
* added libxi and libxmu as dependencies for ros build farm
* Merge pull request `#61 <https://github.com/ipa320/cob_people_perception/issues/61>`_ from ipa-rmb/indigo_dev
  bugfix in CMakeLists.txt that hindered catkin build
* update of CMakeLists
* Merge pull request `#57 <https://github.com/ipa320/cob_people_perception/issues/57>`_ from ipa-rmb/indigo_dev
  A few updates and bugfixes on the tracker
* Merge branch 'indigo_dev' of github.com:ipa320/cob_people_perception into indigo_dev
* bugfixes with coordinate system definitions
* Merge pull request `#58 <https://github.com/ipa320/cob_people_perception/issues/58>`_ from papallas/indigo_dev
  Added some further information to readme
* Minor change
* Minor change
* Added more information about quick start
* bugfix in bounding box display
* Merge branch 'indigo_dev' of github.com:ipa320/cob_people_perception into indigo_dev
* updating tf output
* Merge pull request `#53 <https://github.com/ipa320/cob_people_perception/issues/53>`_ from ipa320/ipa-rmb-patch-1
  Changed publisher queue size from inf to 1
* Changed publisher queue size from inf to 1
  according to suggestion in issue `#52 <https://github.com/ipa320/cob_people_perception/issues/52>`_
* corrected coordinate frame of output point cloud with segmented people
* Update readme.md
* Update readme.md
* Merge pull request `#48 <https://github.com/ipa320/cob_people_perception/issues/48>`_ from ipa-rmb/indigo_dev
  Bugfix for running openni2_tracker and openni2_launch simultaneously
* updated readmes
* bugfix for running openni2_tracker and openni2_launch simultaneously
* Merge branch 'indigo_dev' of github.com:ipa320/cob_people_perception into indigo_dev
* working on running tracker and driver simultaneously
* Merge pull request `#42 <https://github.com/ipa320/cob_people_perception/issues/42>`_ from mintar/fix_tracked_users
  Fix segfault from tracked_users\_ initialization
* Fix segfault from tracked_users\_ initialization
  tracked_users\_ has to be initialized before registering the
  imageCallback, otherwise it can be NULL inside the callback,
  leading to the following segfault:
  `#0 <https://github.com/ipa320/cob_people_perception/issues/0>`_  0x00007fffec52419a in std::list<nite::UserData, std::allocator<nite::UserData> >::empty (this=0x0) at /usr/include/c++/4.8/bits/stl_list.h:869
  `#1 <https://github.com/ipa320/cob_people_perception/issues/1>`_  0x00007fffec515161 in BodyTracker::imageCallback (this=0x95c090, color_image_msg=...)
  at /home/martin/ros-indigo-ws/src/cob_people_perception/cob_openni2_tracker/ros/src/body_tracker.cpp:196
* Merge pull request `#31 <https://github.com/ipa320/cob_people_perception/issues/31>`_ from ipa-fmw/indigo_dev
  alphabetic order
* add dependency to openni2_camera
* Merge branch 'indigo_dev' of https://github.com/ipa-fmw/cob_people_perception into indigo_dev
  Conflicts:
  cob_leg_detection/CMakeLists.txt
  cob_leg_detection/package.xml
* alphabetic order
* Merge pull request `#30 <https://github.com/ipa320/cob_people_perception/issues/30>`_ from ipa-rmb/indigo_dev
  conversion to package format 2 and little fixes
* cleaning and conversion to package format 2
* added missing install commands
* Merge pull request `#1 <https://github.com/ipa320/cob_people_perception/issues/1>`_ from ipa320/indigo_dev
  updates from ipa320
* Merge pull request `#27 <https://github.com/ipa320/cob_people_perception/issues/27>`_ from ipa-rmb/indigo_dev
  updates from rmb
* hopefully corrected dependencies to message generation in cob_perception_msgs
* fixing dependencies
* fixed some wrong dependencies
* updated package.xml information
* fixed warning
* Merge branch 'hydro_dev' of github.com:rmb-om/cob_people_perception into rmb-om-hydro_dev
* added new readme file
* fixed a bug
* fixed bug on cv functions.Added descriptions.Current state: working
* working on functions for image transform
* added body_tracker nodelet
* added a link creation script to allow NiTE to find its data files
* bugfixing
* fixed an install bug with a config file
* Merge branch 'indigo_dev' into rmb-om-hydro_dev
* merge with rmb-om
* merge with rmb-om
* catkinizing
* finalized catkin portation
* merge with rmb/indigo_dev
* a few updates for people_perception from rmb-om
* Merge branch 'hydro_dev' of github.com:rmb-om/cob_people_perception into rmb-om-hydro_dev
* bugfixing cob_people_perception
* bugfixing cob_people_perception
* fixed dependencies
* merge with rmb-om
* added openni2_tracker
* Contributors: Felix Messmer, Florian Weisshardt, Martin Guenther, Pierre Killy, Rafael, Richard Bormann, ipa-fmw, ipa-fxm, ipa-rmb, rmb-om
