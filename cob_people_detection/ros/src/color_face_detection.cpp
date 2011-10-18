/**********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Caroline Pantofaru
 * Modified for Kinect/Point Cloud support: Richard Bormann */



#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

#include <cob_people_detection/PositionMeasurement.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/CvBridge.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "opencv/cxcore.hpp"
#include "opencv/cv.hpp"
#include "opencv/highgui.h"

#include "cob_people_detection/faces.h"

#include "image_geometry/stereo_camera_model.h"

#include <actionlib/server/simple_action_server.h>
#include <cob_people_detection/FaceDetectorAction.h>


using namespace std;

namespace people {

/** FaceDetector - A wrapper around OpenCV's face detection, plus some usage of depth from stereo to restrict the results based on plausible face size.
 */
class FaceDetectorColor : public nodelet::Nodelet {
public:
  // Constants
  const double BIGDIST_M;// = 1000000.0;

  // Node handle
  ros::NodeHandle nh_;
  std::string node_name_;

  // Images and conversion
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter limage_sub_; /**< Left image msg. */
  message_filters::Subscriber<stereo_msgs::DisparityImage> dimage_sub_; /**< Disparity image msg. */
  message_filters::Subscriber<sensor_msgs::CameraInfo> lcinfo_sub_; /**< Left camera info msg. */
  message_filters::Subscriber<sensor_msgs::CameraInfo> rcinfo_sub_; /**< Right camera info msg. */
  sensor_msgs::CvBridge lbridge_; /**< ROS->OpenCV bridge for the left image. */
  sensor_msgs::CvBridge dbridge_; /**< ROS->OpenCV bridge for the disparity image. */
  message_filters::Subscriber<sensor_msgs::PointCloud2> depth_cloud_sub_;  /**< PointCloud2 input from ROS message */
  

  message_filters::TimeSynchronizer<sensor_msgs::Image, stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_stereo_; /**< Stereo topic synchronizer. */
  //message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::PointCloud2> > sync_pointcloud_; /**< Pointcloud synchronizer. */
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, stereo_msgs::DisparityImage, sensor_msgs::PointCloud2> > sync_pointcloud_showdisp_; /**< Pointcloud synchronizer with disparity display. */
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> > sync_pointcloud_nodisp_; /**< Pointcloud synchronizer without disparity display. */
  

  // Action
  actionlib::SimpleActionServer<cob_people_detection::FaceDetectorAction>* as_;
  cob_people_detection::FaceDetectorFeedback feedback_;
  cob_people_detection::FaceDetectorResult result_;

  // If running the face detector as a component in part of a larger person tracker, this subscribes to the tracker's position measurements and whether it was initialized by some other node. 
  // Todo: resurrect the person tracker.
  ros::Subscriber pos_sub_;
  bool external_init_; 


  // Publishers
  // A point cloud of the face positions, meant for visualization in rviz. 
  // This could be replaced by visualization markers, but they can't be modified 
  // in rviz at runtime (eg the alpha, display time, etc. can't be changed.)
  ros::Publisher cloud_pub_;
  ros::Publisher pos_pub_;

  // Display
  string do_display_; /**< Type of display, none/local */
  cv::Mat cv_image_out_; /**< Display image. */
  bool do_display_disparity_or_depth_; /**< Displays the disparity image (in case of stereo camera sensor) or the depth image (if available besides the point cloud) */

  // Data Source - from stereo camera or depth sensor which exports a point cloud 2
  string data_source_mode_;   /**< "stereo" or "pointcloud" */

  // Stereo
  bool use_depth_; /**< True/false use depth information. */
  image_geometry::StereoCameraModel cam_model_; /**< ROS->OpenCV image_geometry conversion. */

  // Face detector params and output
  Faces *faces_; /**< List of faces and associated fcns. */
  string name_; /**< Name of the detector. Ie frontalface, profileface. These will be the names in the published face location msgs. */
  string haar_filename_; /**< Training file for the haar cascade classifier. */
  double reliability_; /**< Reliability of the predictions. This should depend on the training file used. */

  struct RestampedPositionMeasurement {
    ros::Time restamp;
    cob_people_detection::PositionMeasurement pos;
    double dist;
  };
  map<string, RestampedPositionMeasurement> pos_list_; /**< Queue of updated face positions from the filter. */

  bool quit_;

  tf::TransformListener tf_;

  boost::mutex cv_mutex_, pos_mutex_, limage_mutex_, dimage_mutex_;

  bool do_continuous_; /**< True = run as a normal node, searching for faces continuously, False = run as an action, wait for action call to start detection. */
  
  bool do_publish_unknown_; /**< Publish faces even if they have unknown depth/size. Will just use the image x,y in the pos field of the published position_measurement. */

  void onInit()
  {
    std::cout << "IPA version\n\n";

    nh_ = getNodeHandle();
    node_name_ = ros::this_node::getName();

    it_ = image_transport::ImageTransport(nh_);
    as_ = new actionlib::SimpleActionServer<cob_people_detection::FaceDetectorAction>(nh_, node_name_,false);
    //actionlib::SimpleActionServer<cob_people_detection::FaceDetectorAction> as(nh_, node_name_,false);

    ROS_INFO_STREAM_NAMED("face_detector","Constructing FaceDetector.");
    
    if (do_display_ == "local") {
      // OpenCV: pop up an OpenCV highgui window
      cv::namedWindow("Face detector: Face Detection", CV_WINDOW_AUTOSIZE);
    }
    
    // Action stuff
    as_->registerGoalCallback(boost::bind(&FaceDetectorColor::goalCB, this));
    as_->registerPreemptCallback(boost::bind(&FaceDetectorColor::preemptCB, this));
    
    faces_ = new Faces();
    double face_size_min_m, face_size_max_m, max_face_z_m, face_sep_dist_m;
    
    // Parameters
    ros::NodeHandle local_nh = getPrivateNodeHandle();
    local_nh.param("classifier_name",name_,std::string(""));
    local_nh.param("classifier_filename",haar_filename_,std::string(""));
    local_nh.param("classifier_reliability",reliability_,0.0);
    local_nh.param("do_display",do_display_,std::string("none"));
    local_nh.param("do_continuous",do_continuous_,true);
    local_nh.param("do_publish_faces_of_unknown_size",do_publish_unknown_,false);
    local_nh.param("use_depth",use_depth_,true);
    local_nh.param("use_external_init",external_init_,true);
    local_nh.param("face_size_min_m",face_size_min_m,Faces::FACE_SIZE_MIN_M);
    local_nh.param("face_size_max_m",face_size_max_m,Faces::FACE_SIZE_MAX_M);
    local_nh.param("max_face_z_m",max_face_z_m,Faces::MAX_FACE_Z_M);
    local_nh.param("face_separation_dist_m",face_sep_dist_m,Faces::FACE_SEP_DIST_M);
    local_nh.param("do_display_disparity_or_depth",do_display_disparity_or_depth_,false);
    local_nh.param("data_source_mode",data_source_mode_,std::string("stereo"));
    
    faces_->initFaceDetection(1, haar_filename_, face_size_min_m, face_size_max_m, max_face_z_m, face_sep_dist_m);
    
    // Subscribe to the images and camera parameters
    string stereo_namespace, image_topic;
    stereo_namespace = nh_.resolveName("stereo");
    image_topic = nh_.resolveName("image");
    string left_topic = ros::names::clean(stereo_namespace + "/left/" + image_topic);
    string disparity_topic = ros::names::clean(stereo_namespace + "/disparity");
    string left_camera_info_topic = ros::names::clean(stereo_namespace + "/left/camera_info");
    string right_camera_info_topic = ros::names::clean(stereo_namespace + "/right/camera_info");
    limage_sub_.subscribe(it_,left_topic,3);
    dimage_sub_.subscribe(nh_,disparity_topic,3);
    
    sensor_msgs::PointCloud2::ConstPtr nullPointCloud;
    sensor_msgs::CameraInfo::ConstPtr nullCameraInfo;
    stereo_msgs::DisparityImage::ConstPtr nullDisparity;
    if (data_source_mode_=="stereo")
    {
      lcinfo_sub_.subscribe(nh_,left_camera_info_topic,3);
      rcinfo_sub_.subscribe(nh_,right_camera_info_topic,3);
      sync_stereo_.connectInput(limage_sub_, dimage_sub_, lcinfo_sub_, rcinfo_sub_);
      sync_stereo_.registerCallback(boost::bind(&FaceDetectorColor::imageCBAll, this, _1, _2, _3, _4, nullPointCloud));
    }
    else if (data_source_mode_=="pointcloud")
    {
      depth_cloud_sub_.subscribe(nh_,ros::names::clean("/camera/depth/points"),3);
      // only connect to the disparity topic if this is available
      if (do_display_disparity_or_depth_==true)
      {
        sync_pointcloud_showdisp_.connectInput(limage_sub_, dimage_sub_, depth_cloud_sub_);
        sync_pointcloud_showdisp_.registerCallback(boost::bind(&FaceDetectorColor::imageCBAll, this, _1, _2, nullCameraInfo, nullCameraInfo, _3));
      }
      else
      {
        sync_pointcloud_nodisp_.connectInput(limage_sub_, depth_cloud_sub_);
        sync_pointcloud_nodisp_.registerCallback(boost::bind(&FaceDetectorColor::imageCBAll, this, _1, nullDisparity, nullCameraInfo, nullCameraInfo, _2));
      }
    }
    else
    {
      std::cout << "Error: FaceDetector::FaceDetector: data_source_mode_ must be either 'stereo' or 'pointcloud'.\n";
      return;
    }
    //sensor_msgs::Image::ConstPtr nullImage = 0;
    //sync_.connectInput(limage_sub_, dimage_sub_, lcinfo_sub_, rcinfo_sub_, mdimage_sub_, depth_cloud_sub_);
    //sync_.registerCallback(boost::bind(&FaceDetectorColor::imageCBAll, this, _1, _2, _3, _4, /*_5*/nullImage, _6));
    
    // Advertise a position measure message.
    pos_pub_ = nh_.advertise<cob_people_detection::PositionMeasurement>("face_detector/people_tracker_measurements",1);
    
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("face_detector/faces_cloud",0);
    
    // Subscribe to filter measurements.
    if (external_init_) {
      pos_sub_ = nh_.subscribe("people_tracker_filter",1,&FaceDetectorColor::posCallback,this);
      ROS_INFO_STREAM_NAMED("face_detector","Subscribed to the person filter messages.");
    }
    
    // start the action server
    if (do_continuous_==false) as_->start();
    
    ros::MultiThreadedSpinner s(2);
    ros::spin(s);
  }
  
  FaceDetectorColor() :
    BIGDIST_M(1000000.0),
    it_(nh_),
    sync_stereo_(4),
    sync_pointcloud_showdisp_(3),
    sync_pointcloud_nodisp_(2),
    faces_(0),
    quit_(false)
  {
    // all code in onInit() because we are using a nodelet
  }

  ~FaceDetectorColor()
  {

    cv_image_out_.release();

    if (do_display_ == "local") {
      cv::destroyWindow("Face detector: Face Detection");
    }

    if (faces_) {delete faces_; faces_ = 0;}

    as_->shutdown();
    delete as_;
  }

  void goalCB() {
    ROS_INFO("Face detector action started.");
    as_->acceptNewGoal();
  }

  void preemptCB() {
    ROS_INFO("Face detector action preempted.");
    as_->setPreempted();
  }


  /*!
   * \brief Position message callback. 
   *
   * When hooked into the person tracking filter, this callback will listen to messages 
   * from the filter with a person id and 3D position and adjust the person's face position accordingly.
   */ 
  void posCallback(const cob_people_detection::PositionMeasurementConstPtr& pos_ptr) {

    std::cout << "Tracking callback\n";

    // Put the incoming position into the position queue. It'll be processed in the next image callback.
    boost::mutex::scoped_lock lock(pos_mutex_);
    map<string, RestampedPositionMeasurement>::iterator it;
    it = pos_list_.find(pos_ptr->object_id);
    RestampedPositionMeasurement rpm;
    rpm.pos = *pos_ptr;
    rpm.restamp = pos_ptr->header.stamp;
    rpm.dist = BIGDIST_M;
    if (it == pos_list_.end()) {
      pos_list_.insert(pair<string, RestampedPositionMeasurement>(pos_ptr->object_id, rpm));
    }
    else if ((pos_ptr->header.stamp - (*it).second.pos.header.stamp) > ros::Duration().fromSec(-1.0) ){
      (*it).second = rpm;
    }
    lock.unlock();

  }

  // Workaround to convert a DisparityImage->Image into a shared pointer for cv_bridge in imageCBAll.
  struct NullDeleter
  {
    void operator()(void const *) const {}
  };


  /*! 
   * \brief Image callback for synced messages. 
   *
   * For each new image:
   * convert it to OpenCV format, perform face detection using OpenCV's haar filter cascade classifier, and
   * (if requested) draw rectangles around the found faces.
   * Can also compute which faces are associated (by proximity, currently) with faces it already has in its list of people.
   */
  void imageCBAll(const sensor_msgs::Image::ConstPtr &limage, const stereo_msgs::DisparityImage::ConstPtr& dimage, const sensor_msgs::CameraInfo::ConstPtr& lcinfo, const sensor_msgs::CameraInfo::ConstPtr& rcinfo,
                  const sensor_msgs::PointCloud2::ConstPtr& depth_cloud_msg)
  {

    // Only run the detector if in continuous mode or the detector was turned on through an action invocation.
    if (!do_continuous_ && !as_->isActive())
      return;

    // Clear out the result vector.
    result_.face_positions.clear();

    if (do_display_ == "local") {
      cv_mutex_.lock();
    }
 
    // ROS --> OpenCV
    cv::Mat cv_image_left(lbridge_.imgMsgToCv(limage,"bgr8"));  // (left) camera image
    sensor_msgs::ImageConstPtr boost_dimage;
    if (data_source_mode_=="stereo" || do_display_disparity_or_depth_==true) boost_dimage = sensor_msgs::ImageConstPtr(const_cast<sensor_msgs::Image*>(&dimage->image), NullDeleter());
    cv::Mat cv_image_disp;  // disparity image for stereo mode
    pcl::PointCloud<pcl::PointXYZ> depth_cloud; // point cloud for point cloud mode
    if (data_source_mode_=="stereo")
    {
      cv_image_disp = cv::Mat(dbridge_.imgMsgToCv(boost_dimage));
      cam_model_.fromCameraInfo(lcinfo,rcinfo);
    }
    else if (data_source_mode_=="pointcloud") pcl::fromROSMsg(*depth_cloud_msg, depth_cloud);
    
// check disparity image contents
//       std::cout << "channels: " << cv_image_disp.channels() << "\n";
//       for (int i=100; i < 110; i++)
//       {
//           printf("line %d\n", i);
//           float* p_disp = cv_image_disp.ptr<float>(i);
//           for (int j=100; j < 110; j++)
//               printf("%.4f  ", p_disp[j]);
//           printf("\n");
//       }
    // check point cloud contents which are accessible like via depth image coordinates (u,v)
//     for (int i=100; i < 110; i++)
//     {
//       printf("line %d\n", i);
//       for (int j=100; j < 110; j++)
//       {
//         pcl::PointXYZ point = depth_cloud(i, j);
//         printf("(%d/%d):(%.4f/%.4f/%.4f)  ", i, j, point.x, point.y, point.z);
//       }
//       printf("\n");
//     }

    // display the disparity image if available
    if (do_display_disparity_or_depth_ == true)
    {
      cv::Mat cv_image_disp_copy(dbridge_.imgMsgToCv(boost_dimage, "mono8"));
      cv::imshow("disp", cv_image_disp_copy);
      cv::waitKey(10);
    }

    // For display, keep a copy of the image that we can draw on.
    if (do_display_ == "local") {
      cv_image_out_ = cv_image_left.clone();
    }
 
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    ros::Time starttdetect = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);

    vector<Box2D3D> faces_vector = faces_->detectAllFaces(cv_image_left, 1.0, cv_image_disp, &cam_model_, &depth_cloud);
    gettimeofday(&timeofday,NULL);
    ros::Time endtdetect = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
    ros::Duration diffdetect = endtdetect - starttdetect;
    ROS_DEBUG_STREAM_NAMED("face_detector","Detection duration = " << diffdetect.toSec() << "sec");   
    //ROS_INFO_STREAM("Detection duration = " << diffdetect.toSec() << "sec");   

    bool found_faces = false;

    int ngood = 0;
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = limage->header.stamp;
    cloud.header.frame_id = limage->header.frame_id;

    if (faces_vector.size() > 0 ) {

      // Transform the positions of the known faces and remove anyone who hasn't had an update in a long time.
      boost::mutex::scoped_lock pos_lock(pos_mutex_);
      map<string, RestampedPositionMeasurement>::iterator it;
      for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
        if ((limage->header.stamp - (*it).second.restamp) > ros::Duration().fromSec(5.0)) {
          // Position is too old, kill the person.
          pos_list_.erase(it);
        }
        else {
          // Transform the person to this time. Note that the pos time is updated but not the restamp.
          tf::Point pt;
          tf::pointMsgToTF((*it).second.pos.pos, pt);
          tf::Stamped<tf::Point> loc(pt, (*it).second.pos.header.stamp, (*it).second.pos.header.frame_id);
          try {
                tf_.transformPoint(limage->header.frame_id, limage->header.stamp, loc, "odom_combined", loc);
                (*it).second.pos.header.stamp = limage->header.stamp;
                (*it).second.pos.pos.x = loc[0];
                (*it).second.pos.pos.y = loc[1];
                (*it).second.pos.pos.z = loc[2];
          }
          catch (tf::TransformException& ex) {
          }
        }
      } 
      // End filter face position update

      // Associate the found faces with previously seen faces, and publish all good face centers.
      Box2D3D *one_face;
      cob_people_detection::PositionMeasurement pos;

      for (uint iface = 0; iface < faces_vector.size(); iface++) {
        one_face = &faces_vector[iface];

        if (one_face->status=="good" || (one_face->status=="unknown" && do_publish_unknown_)) {

          std::string id = "";

          // Convert the face format to a PositionMeasurement msg.
          pos.header.stamp = limage->header.stamp;
          pos.name = name_;
          pos.pos.x = one_face->center3d.x;
          pos.pos.y = one_face->center3d.y;
          pos.pos.z = one_face->center3d.z;
          pos.header.frame_id = limage->header.frame_id;//"*_stereo_optical_frame";
          pos.reliability = reliability_;
          pos.initialization = 1;//0;
          pos.covariance[0] = 0.04; pos.covariance[1] = 0.0;  pos.covariance[2] = 0.0;
          pos.covariance[3] = 0.0;  pos.covariance[4] = 0.04; pos.covariance[5] = 0.0;
          pos.covariance[6] = 0.0;  pos.covariance[7] = 0.0;  pos.covariance[8] = 0.04;
          
          // Check if this person's face is close enough to one of the previously known faces and associate it with the closest one.
          // Otherwise publish it with an empty id.
          // Note that multiple face positions can be published with the same id, but ids in the pos_list_ are unique. The position of a face in the list is updated with the closest found face.
          double dist, mindist = BIGDIST_M;
          map<string, RestampedPositionMeasurement>::iterator close_it = pos_list_.end();
          for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
            dist = pow((*it).second.pos.pos.x - pos.pos.x, 2.0) + pow((*it).second.pos.pos.y - pos.pos.y, 2.0) + pow((*it).second.pos.pos.z - pos.pos.z, 2.0);
            if (dist <= faces_->face_sep_dist_m_ && dist < mindist) {
              mindist = dist;
              close_it = it;
            }
          }
          std::cout << "poselist size: " << pos_list_.size() << "\n";
          if (close_it != pos_list_.end()) {
            if (mindist < (*close_it).second.dist) {
              (*close_it).second.restamp = limage->header.stamp;
              (*close_it).second.dist = mindist;
              (*close_it).second.pos = pos;
            }
            pos.object_id = (*close_it).second.pos.object_id;
            std::cout << "Object ID: " << pos.object_id << "\n";
            ROS_INFO_STREAM_NAMED("face_detector","Found face " << pos.object_id);
          }
          else {
            pos.object_id = "";
          }
          result_.face_positions.push_back(pos);
          found_faces = true;
          pos_pub_.publish(pos);

        }
	
      }
      pos_lock.unlock();

      // Clean out all of the distances in the pos_list_
      for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
        (*it).second.dist = BIGDIST_M;
      }
      // Done associating faces.

      /******** Display **************************************************************/

      // Draw an appropriately colored rectangle on the display image and in the visualizer.

      cloud.channels.resize(1);
      cloud.channels[0].name = "intensity";

      for (uint iface = 0; iface < faces_vector.size(); iface++) {
        one_face = &faces_vector[iface];

        // Visualization of good faces as a point cloud
        if (one_face->status == "good") {

          geometry_msgs::Point32 p;
          p.x = one_face->center3d.x;
          p.y = one_face->center3d.y;
          p.z = one_face->center3d.z;
          cloud.points.push_back(p);
          cloud.channels[0].values.push_back(1.0f);

          ngood ++;
        }
        else {
          ROS_DEBUG_STREAM_NAMED("face_detector","The detection didn't have a valid size, so it wasn't visualized.");
        }

        // Visualization by image display.
        if (do_display_ == "local") {
          cv::Scalar color;
          if (one_face->status == "good") {
            color = cv::Scalar(0,255,0);
          }
          else if (one_face->status == "unknown") {
            color = cv::Scalar(255,0,0);
          }
          else {
            color = cv::Scalar(0,0,255);
          }

          if (do_display_ == "local") {
            cv::rectangle(cv_image_out_,
                  cv::Point(one_face->box2d.x,one_face->box2d.y),
                  cv::Point(one_face->box2d.x+one_face->box2d.width, one_face->box2d.y+one_face->box2d.height), color, 4);
          }
        }
      } 

    }

    cloud_pub_.publish(cloud);

    // Display
    if (do_display_ == "local") {

      cv::imshow("Face detector: Face Detection",cv_image_out_);
      cv::waitKey(2);
 
      cv_mutex_.unlock();
    }
    /******** Done display **********************************************************/

    ROS_INFO_STREAM_NAMED("face_detector","Number of faces found: " << faces_vector.size() << ", number with good depth and size: " << ngood);


    // If you don't want continuous processing and you've found at least one face, turn off the detector.
    if (!do_continuous_ && found_faces) {
      as_->setSucceeded(result_);
    }
  }

}; // end class
 
}; // end namespace people

PLUGINLIB_DECLARE_CLASS(people, FaceDetectorColor, people::FaceDetectorColor, nodelet::Nodelet);

// Main
// int main(int argc, char **argv)
// {
//   ros::init(argc,argv,"face_detector");
//
//   node_name_ = ros::this_node::getName();
//   people::FaceDetector fd();
// 
//   return 0;
// }


