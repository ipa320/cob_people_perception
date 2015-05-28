/*********************************************************************
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
#include <ros/ros.h>

// Own includes
//#include <leg_detector/constants.h>
#include <dual_people_leg_tracker/dual_tracker.h>
#include <dual_people_leg_tracker/DualTrackerConfig.h>
#include <leg_detector/laser_processor.h>
#include <leg_detector/calc_leg_features.h>
#include <dual_people_leg_tracker/visualization/visualization_conversions.h>
#include <benchmarking/timer.h>
#include <dual_people_leg_tracker/leg_feature.h>
#include <dual_people_leg_tracker/people_tracker.h>
#include <leg_detector/color_definitions.h>
#include <dual_people_leg_tracker/visualization/color_functions.h>

// OpenCV includes
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

// Messages
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Transforms
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

// People tracking
#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>
#include <dual_people_leg_tracker/models/occlusion_model.h>

// Configuration
#include <dynamic_reconfigure/server.h>

#include <algorithm>

// Namespaces
using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

// Default variables
static string fixed_frame              = "odom_combined";  // The fixed frame in which ? //TODO find out


// Defines
#define DUALTRACKER_DEBUG 1         // Debug the leg detector
#define DUALTRACKER_TIME_DEBUG 1    // Debug the calculation time inside the leg_detector

class MatchedFeature
{
public:
  SampleSet* candidate_;  // The point cluster
  LegFeaturePtr closest_; // The feature/leg tracker
  float distance_;		  // The distance between the
  double probability_;

  MatchedFeature(SampleSet* candidate, LegFeaturePtr closest, float distance, double probability)
    : candidate_(candidate)
    , closest_(closest)
    , distance_(distance)
    , probability_(probability)
  {}

  inline bool operator< (const MatchedFeature& b) const
  {
    return (distance_ <  b.distance_);
  }
};

int g_argc;
char** g_argv;

struct LegDetectionProb {
    Stamped<Point> point;
    SampleSet* cluster;
};

bool isLegFeatureValid(const LegFeaturePtr & o){
  return !o->isValid();
}

// actual legdetector node
class DualTracker
{
public:
  NodeHandle nh_; /**< The node handle */

  TransformListener tfl_; /**< The transform listener */

  ScanMask mask_; /**< A scan mask */

  OcclusionModelPtr occlusionModel_; /**< The occlusion model */

  int mask_count_;

  CvRTrees forest; /**< The forest classificator */

  float connected_thresh_; /**< Parameter for the clustering(Creation of SampleSets) */

  int feat_count_; /**< Number of features evaluated for each SampleSet */

  char save_[100];

  //std::vector<PeopleTrackerPtr> people_tracker_;

  //list<SavedFeature*> saved_features_; /**< List of SavedFeatures that are currently tracked*/

  list<LegFeaturePtr> saved_leg_features; /**< List of SavedFeatures(Legs) that are currently tracked*/

  PeopleTrackerList people_trackers_; /**< Object to handle the people_trackers */

  boost::mutex saved_mutex_; /**< Mutex to handle the access to the Saved Features */

  int feature_id_;

  bool use_seeds_;

  //GlobalConfig* globalConfig;

  bool publish_leg_measurements_;
  bool publish_people_;
  bool publish_leg_markers_;
  bool publish_people_markers_;
  bool publish_clusters_;
  bool publish_particles_;
  bool publish_matches_;
  bool publish_leg_history_;
  bool publish_people_tracker_;
  bool publish_leg_velocity_; /**< True if the estimation of the leg features are visualized as arrows */
  bool publish_static_people_trackers_; /**< Set True if also static People Trackers(Trackers that never moved) should be displayed */
  bool publish_people_history_; /**< Publish the history of the person */
  bool publish_occlusion_model_; /**< Publish the probabilities of the particles (colorcoded) according to the occlusion model */

  int next_p_id_;

  double leg_reliability_limit_;

  double people_probability_limit_; /**< Min Value for people to be considered true  */

  int min_points_per_group_;

  unsigned int cycle_; /**< Cycle counter to count the filter cycles */

  benchmarking::Timer cycleTimer; /**< Timer to measure the cycle time */
  benchmarking::Timer freeTimer; /**< Timer to measure the time left for calculations */

  // The publishers
  ros::Publisher people_measurements_pub_; /**< Publisher for people measurements */
  ros::Publisher leg_measurements_pub_; /**< Publisher for leg measurements */
  ros::Publisher markers_pub_; /**< Publisher for features */
  ros::Publisher clusters_pub_;/**< Publisher for the clusters generated by scan processor */
  ros::Publisher leg_measurements_vis_pub_;/**< Visualization of leg detections */
  ros::Publisher leg_features_vis_pub_;/**< Visualization of leg tracks */
  ros::Publisher leg_features_array_vis_pub_;/**< Visualization of leg estimation using arrows */
  ros::Publisher people_track_vis_pub_;/**< Visualization of people tracks */
  ros::Publisher leg_features_history_vis_pub_;/**< Visualization of leg tracks */
  ros::Publisher people_history_vis_pub_;/**< Visualization of leg tracks */
  ros::Publisher matches_vis_pub_;/**< Visualization of the pairing leg_detection <-> leg_track */
  ros::Publisher particles_pub_;/**< Visualization of particles */
  ros::Publisher people_velocity_pub_;/**< Visualization of the people velocities */
  ros::Publisher people_track_label_pub_; /**< Publishes labels of people tracks */
  ros::Publisher occlusion_model_pub_; /**< Published the occlusion probability */
  ros::Publisher leg_predicted_pub_; /**< Published the occlusion probability */

  dynamic_reconfigure::Server<dual_people_leg_tracker::DualTrackerConfig> server_; /**< The configuration server*/

  message_filters::Subscriber<people_msgs::PositionMeasurement> people_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<people_msgs::PositionMeasurement> people_notifier_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

  DualTracker(ros::NodeHandle nh) :
    nh_(nh),
    mask_count_(0),
    feat_count_(0),
    next_p_id_(0),
    people_sub_(nh_, "dual_tracker", 10),
    laser_sub_(nh_, "scan", 10),
    people_notifier_(people_sub_, tfl_, fixed_frame, 10),
    laser_notifier_(laser_sub_, tfl_, fixed_frame, 10),
    cycle_(0),
    occlusionModel_(new OcclusionModel(tfl_))
  {
    if (g_argc > 1)
    {
      forest.load(g_argv[1]);
      feat_count_ = forest.get_active_var_mask()->cols;
      printf("Loaded forest with %d features: %s\n", feat_count_, g_argv[1]);
    }
    else
    {
      printf("Please provide a trained random forests classifier as an input.\n");
      shutdown();
    }

    nh_.param<bool>("use_seeds", use_seeds_, false); // TODO maybe remove later?

    // advertise topics
    leg_measurements_pub_         = nh_.advertise<people_msgs::PositionMeasurementArray>("leg_tracker_measurements", 0);
    people_measurements_pub_      = nh_.advertise<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 0);
    markers_pub_                  = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    leg_features_history_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("leg_track_history", 0);
    clusters_pub_                 = nh_.advertise<sensor_msgs::PointCloud>("clusters", 0);
    particles_pub_                = nh_.advertise<sensor_msgs::PointCloud>("particles", 0);
    leg_features_array_vis_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("leg_feature_arrow", 0);
    people_velocity_pub_          = nh_.advertise<visualization_msgs::MarkerArray>("people_velocity_arrow", 0);
    people_track_label_pub_       = nh_.advertise<visualization_msgs::MarkerArray>("people_labels", 0);
    occlusion_model_pub_          = nh_.advertise<sensor_msgs::PointCloud>("occlusion_model", 0);

    // Visualization topics
    leg_measurements_vis_pub_     = nh_.advertise<sensor_msgs::PointCloud>("leg_measurements", 0);
    leg_features_vis_pub_         = nh_.advertise<sensor_msgs::PointCloud>("leg_features", 0);
    matches_vis_pub_              = nh_.advertise<visualization_msgs::Marker>("matches", 0);
    people_track_vis_pub_         = nh_.advertise<visualization_msgs::MarkerArray>("peoples", 0);
    people_history_vis_pub_       = nh_.advertise<visualization_msgs::MarkerArray>("people_history", 0);
    leg_predicted_pub_            = nh_.advertise<visualization_msgs::MarkerArray>("leg_predicted_positions_", 0);

    if (use_seeds_)
    {
      //people_notifier_.registerCallback(boost::bind(&DualTracker::peopleCallback, this, _1));
      //people_notifier_.setTolerance(ros::Duration(0.01));
    }

    // Set the laserCallback
    laser_notifier_.registerCallback(boost::bind(&DualTracker::laserCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));

    // Configuration server
    dynamic_reconfigure::Server<dual_people_leg_tracker::DualTrackerConfig>::CallbackType f;
    f = boost::bind(&DualTracker::configure, this, _1, _2);
    server_.setCallback(f);

    feature_id_ = 0;

    ROS_DEBUG("CREATED DUAL_TRACKER");
  }


  ~DualTracker()
  {
  }

  /**
   *  @brief Handles the configuration of this node
   */

  void configure(dual_people_leg_tracker::DualTrackerConfig &config, uint32_t level)
  {
    // Clustering parameters
    connected_thresh_           = config.connection_threshold;    ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - connected_thresh_ %f", __func__, connected_thresh_ );
    min_points_per_group_       = config.min_points_per_group;   ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - min_points_per_group %i", __func__, min_points_per_group_ );

    // Leg Tracker Parameters
    leg_reliability_limit_      = config.leg_reliability_limit;   ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - leg_reliability_limit_ %f", __func__, leg_reliability_limit_ );

    // People Tracker Parameters
    people_probability_limit_   = config.people_probability_limit; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - leg_reliability_limit_ %f", __func__, people_probability_limit_);

    // Publish clustering
    publish_clusters_           = config.publish_clusters;        ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_clusters_ %d", __func__, publish_clusters_ );

    // Publish the leg trackers
    publish_leg_measurements_   = config.publish_leg_measurements;            ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_leg_measurements_ %d", __func__, publish_leg_measurements_ );
    publish_leg_velocity_       = config.publish_leg_velocity; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_leg_velocity_ %d", __func__, publish_leg_velocity_ );
    publish_leg_markers_        = config.publish_leg_markers;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_leg_markers_ %d", __func__, publish_leg_markers_ );
    publish_leg_history_        = config.publish_leg_history;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_leg_history_ %d", __func__, publish_leg_history_ );

    // Publish the people tracker
    publish_people_             = config.publish_people;          ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_ %d", __func__, publish_people_ );
    publish_people_markers_     = config.publish_people_markers;  ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_markers_ %d", __func__, publish_people_markers_ );
    publish_people_tracker_     = config.publish_people_tracker;  ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_tracker_ %d", __func__, publish_people_tracker_ );
    publish_static_people_trackers_ = config.publish_static_people_trackers; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_static_people_trackers_ %d", __func__, publish_static_people_trackers_);
    publish_people_history_     = config.publish_people_history;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_history_ %d", __func__, publish_people_history_ );


    publish_particles_          = config.publish_particles;       ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_particles_ %d", __func__, publish_particles_ );
    publish_matches_            = config.publish_matches;         ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_matches_ %d", __func__, publish_matches_ );

    publish_occlusion_model_    = config.publish_occlusion_model;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_occlusion_model_ %d", __func__, publish_occlusion_model_ );

    no_observation_timeout_s = config.no_observation_timeout;
    max_second_leg_age_s     = config.max_second_leg_age;
    max_track_jump_m         = config.max_track_jump;
    max_meas_jump_m          = config.max_meas_jump;
    leg_pair_separation_m    = config.leg_pair_separation;


    if (fixed_frame.compare(config.fixed_frame) != 0)
    {
      fixed_frame              = config.fixed_frame;
      laser_notifier_.setTargetFrame(fixed_frame);
      people_notifier_.setTargetFrame(fixed_frame);
    }

    ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - Configuration done", __func__);
  }

  /** @brief Called if a new laserscan arrives
   *
   *  If a laserscan arrives the callback function receives it and the following steps are performed://TODO further explanation
   *  1. Processing of the scan using the ScanProcessor Class -> Clusters withing the laserscan are detected
   *  2. Every SavedFeature that has not been detected in within a defined time periode is removed
   *  3. Propagate the features to the current time scan time
   *  4. Predict the probability of the cluster beeing a leg using OpenCV Boosting techniques
   *  5. Find the closest tracker (of type SavedFeature) to the cluster (Euclidian Distance) IDEA_ Use better distance
   *  6. Update the location of the tracker using the assigned Feature
   *  7. Publish visualizations
   *
   *  @param scan ConstPtr to the received laserscan
   */

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    if(cycle_>0){
      freeTimer.stop();
      ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"%sThere where %.2f ms left in the previous cycle(%u)", BOLDCYAN, freeTimer.stopAndGetTimeMs(), cycle_-1);
    }
    ROS_DEBUG_STREAM(BOLDWHITE << "STARTING [Cycle "  << cycle_ << "] Time: " << scan->header.stamp << std::endl);
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Received Laserscan",__func__);

    // Start Cycle Timer
    cycleTimer.start();

    //////////////////////////////////////////////////////////////////////////
    //// Update the occlusion model with the current scan
    //////////////////////////////////////////////////////////////////////////
    occlusionModel_->updateScan(*scan);

    //////////////////////////////////////////////////////////////////////////
    //// Create clusters (takes approx 0.8ms)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sCreating Clusters [Cycle %u]", BOLDWHITE, cycle_);
    // Process the incoming scan
    benchmarking::Timer processTimer; processTimer.start();
    ScanProcessor processor(*scan, mask_);
    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(min_points_per_group_);
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"LegDetector::%s - Process scan(clustering) took %f ms",__func__, processTimer.stopAndGetTimeMs());

    ROS_DEBUG("%sCreating Clusters done! [Cycle %u]", BOLDWHITE, cycle_);
    //////////////////////////////////////////////////////////////////////////
    //// Remove the invalid Trackers (takes approx 0.1ms)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sRemoving old Trackers [Cycle %u]", BOLDWHITE, cycle_);

    // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);

    benchmarking::Timer removeTimer; removeTimer.start();
    // Iterate through the saved features and remove those who havent been observed since (no_observation_timeout_s)

    list<LegFeaturePtr>::iterator sf_iter = saved_leg_features.begin();
    for(list<LegFeaturePtr>::iterator sf_iter = saved_leg_features.begin();
        sf_iter != saved_leg_features.end();
        sf_iter++)
    {
      if ((*sf_iter)->meas_time_ < purge)
      {
        (*sf_iter)->setValidity(false);
      }

      //std::cout << **sf_iter << std::endl;
      //std::cout << "ReferenceCount:" << (*sf_iter).use_count() << std::endl;
    }

    // Remove invalid people tracker
    people_trackers_.removeInvalidTrackers();

    // Remove invalid associations og the leg Features
    for(list<LegFeaturePtr>::iterator leg_it = saved_leg_features.begin();
        leg_it != saved_leg_features.end();
        leg_it++){

        (*leg_it)->removeInvalidAssociations();
    }


    // Remove the invalid legFeatures from saved_leg_features (Remember these are shared pointers!!)
    int size_before = saved_leg_features.size();
    saved_leg_features.erase(std::remove_if(saved_leg_features.begin(), saved_leg_features.end(), isLegFeatureValid), saved_leg_features.end());
    int features_deleted = size_before - saved_leg_features.size();

    removeTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Removed %i features because the havent been detected in the last %f seconds",__func__, features_deleted, no_observation_timeout_s);
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"LegDetector::%s - Removing features took %f ms",__func__, removeTimer.getElapsedTimeMs());

    ROS_DEBUG("%sRemoving old Trackers done! [Cycle %u]", BOLDWHITE, cycle_);

    //////////////////////////////////////////////////////////////////////////
    //// Propagation/Prediction using the motion model
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sPrediction [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer propagationTimer; propagationTimer.start();

    // High level propagation
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > pplTrackers = people_trackers_.getList();
    for(std::vector<PeopleTrackerPtr>::iterator pplTrackerIt = pplTrackers->begin();
        pplTrackerIt != pplTrackers->end();
        pplTrackerIt++)
    {
      (*pplTrackerIt)->propagate(scan->header.stamp);
      // NOT YET IMPLEMENTED
      // MAYBE HERE OUTPUT textfile containing person tracker history
    }

    // System update of trackers, and copy updated ones in propagate list

    list<LegFeaturePtr> propagated;
    for (list<LegFeaturePtr>::iterator legIt = saved_leg_features.begin();
        legIt != saved_leg_features.end();
        legIt++)
    {
      (*legIt)->propagate(scan->header.stamp); // Propagate <-> Predict the filters
      propagated.push_back(*legIt);
    }


    propagationTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Propagated %i SavedFeatures",__func__, (int) propagated.size());
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"LegDetector::%s - Propagating took %f ms",__func__, propagationTimer.getElapsedTimeMs());

    ROS_DEBUG("%sPrediction done! [Cycle %u]", BOLDWHITE, cycle_);
    //////////////////////////////////////////////////////////////////////////
    //// Detection (Search for the existing trackers)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sDetection [Cycle %u]", BOLDWHITE, cycle_);

    std::vector<LegDetectionProb> detections; // vector of leg detections along with their probabilities

    // Matrix for the feature values
    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

    for (list<SampleSet*>::iterator clusterIt = processor.getClusters().begin();
         clusterIt != processor.getClusters().end();
         clusterIt++)
    {
      // Calculate the features of the clusters
      vector<float> f = calcLegFeatures(*clusterIt, *scan); // Calculate the single features -> results in a vector

      // Fill the matrix with the values
      for (int k = 0; k < feat_count_; k++){
        tmp_mat->data.fl[k] = (float)(f[k]);
      }

      // Predict the probability of this cluster in being a leg
      float probability = forest.predict_prob(tmp_mat);  // Predict the probability using the forest
      (*clusterIt)->setProbability(probability); // Assign the probability to the clusters

      Stamped<Point> loc((*clusterIt)->center(), scan->header.stamp, scan->header.frame_id);
      try
      {
        tfl_.transformPoint(fixed_frame, loc, loc); //Transform using odometry information into the fixed frame
        loc.setZ(0); // Ugly //TODO
      }
      catch (...)
      {
        ROS_WARN("TF exception spot 3.");
      }


      if((*clusterIt)->getProbability() > leg_reliability_limit_){
        LegDetectionProb detection;
        detection.point = loc;
        detection.cluster = (*clusterIt);
        detections.push_back(detection);
        ROS_ASSERT(loc.getZ() == 0); //TODO Remove
      }

    }

    ROS_DEBUG("%sDetection done! [Cycle %u]", BOLDWHITE, cycle_);

    //////////////////////////////////////////////////////////////////////////
    //// Matching (Match Leg Detection to Trackers)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sMatching [Cycle %u]", BOLDWHITE, cycle_);

    // Input: The propagated and new trackers, the leg detections

    // Detection step: build up the set of "candidate" clusters
    // For each candidate, find the closest tracker (within threshold) and add to the match list
    // If no tracker is found, start a new one
    // Match = cluster <-> Saved Feature (LEG)

    unsigned int newTrackCounter = 0;
    unsigned int matchesCounter = 0;

    // The found matches
    multiset<MatchedFeature> matches;

    // Iterate through all detections
    for (vector<LegDetectionProb>::iterator detectionIt = detections.begin();
        detectionIt != detections.end();
        detectionIt++)
    {

      Stamped<Point> loc = detectionIt->point;
      SampleSet* cluster = detectionIt->cluster;

      // Find the closest tracker (Note that the tracker has been updated using the filter!)
      // Multiple measurements could be assigned to the same tracker! This problem is solved below. Better methods could be thought of.
      // IDEA_ Do this better! The closest is no necessarily the right one
      list<LegFeaturePtr>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_m;

      // Iterate through the trackers
      for (list<LegFeaturePtr>::iterator legIt = propagated.begin();
          legIt != propagated.end();
          legIt++)
      {
        // find the closest distance between candidate and trackers
        float dist = loc.distance((*legIt)->position_);
        if (dist < closest_dist)
        {
          closest = legIt;
          closest_dist = dist;
        }
      }
      // Nothing close to it, start a new track
      if (closest == propagated.end())
      {
        loc.setZ(0); // TODO ugly fix
        LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(new LegFeature(loc, tfl_));
        newLegFeature->setOcclusionModel(occlusionModel_);
        list<LegFeaturePtr>::iterator new_saved = saved_leg_features.insert(saved_leg_features.end(), newLegFeature);
        ++newTrackCounter;
      }
      // Add the candidate, the tracker and the distance to a match list
      else
      {
        matches.insert(MatchedFeature(cluster, *closest, closest_dist, cluster->getProbability()));
        ++matchesCounter;
      }

    }// end iterate the clusters

    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"DualTracker::%s - Associated tracks to legs - %i matches - %i new tracks",__func__, matchesCounter, newTrackCounter);
    ROS_DEBUG("%sMatching Done! [Cycle %u]", BOLDWHITE, cycle_);

    //////////////////////////////////////////////////////////////////////////
    //// High level Association: Combination of saved features to people tracker (takes approx. 0.8ms)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sHigh Level Association [Cycle %u]", BOLDWHITE, cycle_);
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"DualTracker::%s - Starting to combine %lu leg_tracks to people tracker",__func__, saved_leg_features.size());
    benchmarking::Timer hlAssociationTimer; hlAssociationTimer.start();

    // Do the combinations
    for (list<LegFeaturePtr>::iterator legIt0 = saved_leg_features.begin();
        legIt0 != saved_leg_features.end();
        legIt0++)
    {
      list<LegFeaturePtr>::iterator legIt1 = boost::next(legIt0,1);
      for (;
          legIt1 != saved_leg_features.end();
          legIt1++)
      {

        // Add the tracker to the list if it is new
        if(!this->people_trackers_.exists(*legIt0, *legIt1)){
          // Create the people tracker
          PeopleTrackerPtr temp_people_tracker(new PeopleTracker(*legIt0, *legIt1, scan->header.stamp));
          (*legIt0)->addPeopleTracker(temp_people_tracker);
          (*legIt1)->addPeopleTracker(temp_people_tracker);
          people_trackers_.addPeopleTracker(temp_people_tracker);
        }

        //std::cout << "Investigation of combination " << (*legIt0)->int_id_ << " - " << (*legIt1)->int_id_ << std::endl;
      }
    }

    // Evaluate the combinations (What is the probability for this people tracker
    // assert(false);

    //return;
    hlAssociationTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"High level association took %f ms", hlAssociationTimer.getElapsedTimeMs());
    ROS_DEBUG("%sHigh Level Association done! [Cycle %u]", BOLDWHITE, cycle_);
    //////////////////////////////////////////////////////////////////////////
    //// Update (Update the Trackers using the latest measurements
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sUpdate [Cycle %u]", BOLDWHITE, cycle_);

    // IDEA_ The next step contains one flaw, it is random which closest tracker get choosen and update the tracker, this unacceptable
    // IDEA_ Deploy Linear Association Problem Solver Here
    // Update the matched trackers
    while (matches.size() > 0)
    {
      multiset<MatchedFeature>::iterator matched_iter = matches.begin();  // Matches iterator
      bool found = false;

      // Iterate the propagated SavedFeatures(Legs)
      list<LegFeaturePtr>::iterator pf_iter = propagated.begin(); // Tracker iterator
      while (pf_iter != propagated.end()) // Iterate through all the trackers
      {
        // update the tracker with this candidate
        if (matched_iter->closest_ == *pf_iter) // If this is already the pair
        {
          // Transform candidate to fixed frame
          Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
          try
          {
            tfl_.transformPoint(fixed_frame, loc, loc);
          }
          catch (...)
          {
            ROS_WARN("TF exception spot 4.");
          }

          loc.setZ(0.); // TODO ugly fix that
          // Update the tracker with the candidate location
          matched_iter->closest_->update(loc, matched_iter->probability_);

          //assert(false);

          // remove this match and
          matches.erase(matched_iter);
          propagated.erase(pf_iter++);
          found = true;
          break;
        }
        // still looking for the tracker to update
        else
        {
          pf_iter++;
        }
      }

      // didn't find tracker to update, because it was deleted above
      // try to assign the candidate(leg) to another tracker
      // Explanation: since multiple features could be assigned to the same tracker this can happen. The solution here to this however is not optimal.
      if (!found)
      {

        Stamped<Point> loc(matched_iter->candidate_->center(), scan->header.stamp, scan->header.frame_id);
        try
        {
          tfl_.transformPoint(fixed_frame, loc, loc);
        }
        catch (...)
        {
          ROS_WARN("TF exception spot 5.");
        }

        list<LegFeaturePtr>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;

        for (list<LegFeaturePtr>::iterator remain_iter = propagated.begin();
             remain_iter != propagated.end();
             remain_iter++)
        {
          float dist = loc.distance((*remain_iter)->position_);
          if (dist < closest_dist)
          {
            closest = remain_iter;
            closest_dist = dist;
          }
        }

        // no tracker is within a threshold of this candidate
        // so create a new tracker for this candidate
        if (closest == propagated.end()){
          loc.setZ(0); // TODO
          list<LegFeaturePtr>::iterator new_saved = saved_leg_features.insert(saved_leg_features.end(), boost::shared_ptr<LegFeature>(new LegFeature(loc, tfl_)));

        }
        else{
          matches.insert(MatchedFeature(matched_iter->candidate_, *closest, closest_dist, matched_iter->probability_));

        }
          matches.erase(matched_iter);
      }
    }
    ROS_DEBUG("%sUpdate done! [Cycle %u]", BOLDWHITE, cycle_);

    //////////////////////////////////////////////////////////////////////////
    //// High level Update (Update of the people trackers
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sHigh level update [Cycle %u]", BOLDWHITE, cycle_);

    // Update the probabilites of every people tracker
    people_trackers_.updateAllTrackers(scan->header.stamp);

    ROS_DEBUG("%sHigh level update done [Cycle %u]", BOLDWHITE, cycle_);
    //////////////////////////////////////////////////////////////////////////
    //// Publish data
    //////////////////////////////////////////////////////////////////////////

    ROS_DEBUG("%sPublishing [Cycle %u]", BOLDWHITE, cycle_);

    // Publish the leg measurements
    if(publish_leg_measurements_){
      publishLegMeasurements(processor.getClusters(), scan->header.stamp, scan->header.frame_id);
    }

    // Publish the clustering
    if(publish_clusters_){
      publishClusters(processor.getClusters(), scan->header.stamp, scan->header.frame_id);
    }

    // Publish the detections of legs
    if(publish_leg_velocity_){
      publishLegVelocities(saved_leg_features, scan->header.stamp);
    }

    if(publish_leg_markers_){
      publishLegFeaturesVisualization(saved_leg_features, scan->header.stamp);
    }

    // Publish the history of each leg
    if(publish_leg_history_){
      publishLegHistory(saved_leg_features, scan->header.stamp);
    }

    if(publish_particles_){
      publishParticles(saved_leg_features, scan->header.stamp);
    }

    if(publish_matches_){
      publishMatches(saved_leg_features, scan->header.stamp);
    }

    if(true){
      publishPredictedLegPositions(saved_leg_features, scan->header.stamp);
    }

    if(publish_people_tracker_){
      publishPeopleTracker(scan->header.stamp);
      publishPeopleVelocity(people_trackers_.getList(), scan->header.stamp);
      publishPeopleTrackerLabels(scan->header.stamp);
    }

    // Publish the history of the persons
    if(publish_people_history_){
      publishPeopleHistory(people_trackers_.getList(), scan->header.stamp);
    }

    if(publish_occlusion_model_){
      publishOcclusionModel(saved_leg_features, scan->header.stamp);
    }
    // Publish leg Measurements on
    //if(publish_leg_measurements_){
      //publishLegMeasurementArray(saved_leg_features);
    //}

    ROS_DEBUG("%sPublishing done! [Cycle %u]", BOLDWHITE, cycle_);
    //////////////////////////////////////////////////////////////////////////
    //// Cleaning (Clear data)
    //////////////////////////////////////////////////////////////////////////
    cvReleaseMat(&tmp_mat);
    tmp_mat = 0;

    //////////////////////////////////////////////////////////////////////////
    //// Finalize the Cycle
    //////////////////////////////////////////////////////////////////////////
    cycleTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"%sCycle %u took %.2f ms to complete", BOLDCYAN, cycle_, cycleTimer.stopAndGetTimeMs());
    //assert(cycle_ < 2);

    // Iterate the cycle counter
    cycle_++;

    freeTimer.start();
 }

  /**
   * Publish a list of pointers to legFeatures
   * @param legFeatures List of Leg Feature Pointers
   */
  void publishLegMeasurementArray(std::list<LegFeaturePtr> legFeatures){

    // Abort if List is empty
    if(legFeatures.size() == 0){
      ROS_WARN("Called publishLegMeasurementArray, but the given list is empty. Nothing to publish.");
      return;
    }

    // Iterator variable
    int i = 0;
    vector<people_msgs::PositionMeasurement> legs;

    // Iterate the features
    for (list<LegFeaturePtr>::iterator sf_iter = legFeatures.begin();
         sf_iter != saved_leg_features.end();
         sf_iter++, i++)
    {
      // reliability
      double reliability = (*sf_iter)->getReliability();

      if ((*sf_iter)->getReliability() > leg_reliability_limit_ && publish_leg_measurements_)
      {
        people_msgs::PositionMeasurement pos;
        pos.header.stamp = legFeatures.front()->meas_time_;
        pos.header.frame_id = legFeatures.front()->fixed_frame_;
        pos.name = "leg_detector";
        pos.object_id = (*sf_iter)->id_;
        pos.pos.x = (*sf_iter)->position_[0];
        pos.pos.y = (*sf_iter)->position_[1];
        pos.pos.z = (*sf_iter)->position_[2];
        pos.reliability = reliability;
        pos.covariance[0] = pow(0.3 / reliability, 2.0);
        pos.covariance[1] = 0.0;
        pos.covariance[2] = 0.0;
        pos.covariance[3] = 0.0;
        pos.covariance[4] = pow(0.3 / reliability, 2.0);
        pos.covariance[5] = 0.0;
        pos.covariance[6] = 0.0;
        pos.covariance[7] = 0.0;
        pos.covariance[8] = 10000.0;
        pos.initialization = 0;
        legs.push_back(pos);
      }
  }

  // Create the Position Measurement Array
  people_msgs::PositionMeasurementArray array;
  array.header.stamp =  saved_leg_features.front()->time_;
  array.header.frame_id = saved_leg_features.front()->fixed_frame_;

  // Publish
  array.people = legs;
  leg_measurements_pub_.publish(array);
  ROS_DEBUG("Publishing legs positions on %s", array.header.frame_id.c_str());
  }

  /**
   * Publish the detected Clusters for debugging and illustration purposes
   * @param set
   * @param frame // Frame the clusters are in
   */
  void publishLegMeasurements(std::list<SampleSet*>& clusters, ros::Time time, std::string frame, bool publishAll = false){

    // The pointcloud message
    sensor_msgs::PointCloud clusterPcl;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    clusterPcl.channels.push_back(rgb_channel);
    clusterPcl.header.frame_id = fixed_frame;
    clusterPcl.header.stamp = time;

    for (list<SampleSet*>::iterator clusterIt = clusters.begin();
         clusterIt != clusters.end();
         clusterIt++)
    {
      // Check that either all Clusters should be published or only publish the ones above the reliability limmit
      if(publishAll || (*clusterIt)->getProbability() > leg_reliability_limit_){

        // Create center Point
        Stamped<tf::Point> center((*clusterIt)->center(), time, frame);

        // Transform the Point in the fixed frame
        try
        {
          tfl_.transformPoint(fixed_frame, center, center);
        }
        catch (...)
        {
          ROS_WARN("TF exception in publishClusters!");
        }

        int r,g,b;

        redGreenGradient((*clusterIt)->getProbability(),r,g,b);

        geometry_msgs::Point32 point;
        point.x = center[0];
        point.y = center[1];
        point.z = 0;

        r = 255;

        // Set the color according to the probability
        float color_val = 0;
        int rgb = (r << 16) | (g << 8) | b;
        color_val = *(float*) & (rgb);

        if (clusterPcl.channels[0].name == "rgb")
          clusterPcl.channels[0].values.push_back(color_val);

        clusterPcl.points.push_back(point);
      }
    }

    // Publish the pointcloud
    leg_measurements_vis_pub_.publish(clusterPcl);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

  /**
   * Publish
   * @param legFeatures
   * @param time
   */
  void publishLegVelocities(list<LegFeaturePtr>& legFeatures, ros::Time time){

    // Create the Visualization Message (a marker array)
    visualization_msgs::MarkerArray msgArray;

    int counter = 0;

    for (list<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {
      BFL::StatePosVel est = (*legFeatureIt)->getEstimate();

      //std::cout <<  est << std::endl;

      visualization_msgs::Marker marker;
      marker.header.frame_id = fixed_frame;
      marker.header.stamp = time;
      marker.ns = "leg_feature_arrows";
      marker.id = counter;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;


      geometry_msgs::Point startPoint;
      startPoint.x = est.pos_[0];
      startPoint.y = est.pos_[1];
      startPoint.z = est.pos_[2];

      double factor = 0.5; // Control the arrow length

      geometry_msgs::Point endPoint;
      endPoint.x = est.pos_[0] + est.vel_[0]*factor;
      endPoint.y = est.pos_[1] + est.vel_[1]*factor;
      endPoint.z = est.pos_[2] + est.vel_[2]*factor;

      marker.points.push_back(startPoint);
      marker.points.push_back(endPoint);

      marker.scale.x = 0.05; //shaft diameter
      marker.scale.y = 0.1; //head diameter
      marker.scale.z = 0; // head length (if other than zero)
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      msgArray.markers.push_back(marker);

      counter++;
    }
    leg_features_array_vis_pub_.publish(msgArray);

  }

  void publishPeopleVelocity(boost::shared_ptr<vector<PeopleTrackerPtr> > peopleTracker, ros::Time time){

    // Create the Visualization Message (a marker array)
    visualization_msgs::MarkerArray msgArray;

    int counter = 0;

    for (vector<PeopleTrackerPtr>::iterator peopleIt = peopleTracker->begin();
        peopleIt != peopleTracker->end();
        peopleIt++)
    {
      if((*peopleIt)->getTotalProbability() > people_probability_limit_ ){

        BFL::StatePosVel est = (*peopleIt)->getEstimate();

        //std::cout <<  est << std::endl;

        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame;
        marker.header.stamp = time;
        marker.ns = "people_velocity_arrows";
        marker.id = counter;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point startPoint;
        startPoint.x = est.pos_[0];
        startPoint.y = est.pos_[1];
        startPoint.z = est.pos_[2];

        double factor = 1; // Control the arrow length

        geometry_msgs::Point endPoint;
        endPoint.x = est.pos_[0] + est.vel_[0]*factor;
        endPoint.y = est.pos_[1] + est.vel_[1]*factor;
        endPoint.z = est.pos_[2] + est.vel_[2]*factor;

        marker.points.push_back(startPoint);
        marker.points.push_back(endPoint);

        marker.scale.x = 0.1; //shaft diameter
        marker.scale.y = 0.1; //head diameter
        marker.scale.z = 0; // head length (if other than zero)
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        msgArray.markers.push_back(marker);

        counter++;
      }
    }
    people_velocity_pub_.publish(msgArray);

  }

  /**
   * Publish some clusters
   * @param clusters  The clusters
   * @param time  Time at which the clusters must be published
   * @param frame Frame in which the clusters are published
   */
  void publishClusters(std::list<SampleSet*>& clusters, ros::Time time, std::string frame){

        // Visualize the clusters by creating a pointcloud, each cluster has the same color
        sensor_msgs::PointCloud clusterPCL;
        sensor_msgs::ChannelFloat32 rgb_channel;
        rgb_channel.name="rgb";
        clusterPCL.channels.push_back(rgb_channel);
        clusterPCL.header.frame_id = frame;
        clusterPCL.header.stamp = time;

        int count = 0;
        // Iterate the clusters
        for (list<SampleSet*>::iterator i = clusters.begin();
             i != clusters.end();
             i++)
        {

            int r[3] = { 0, 125, 255};
            int g[3] = { 0, 125, 255};
            int b[3] = { 0, 125, 255};

            int r_ind = count % 3;
            int g_ind = (count/3) % 3;
            int b_ind = (count/9) % 3;
            count++;

            (*i)->appendToCloud(clusterPCL,r[r_ind],g[g_ind],b[b_ind]);
        }

        // Publish the clustering
        clusters_pub_.publish(clusterPCL);
  }

  /**
   * Publish the set Particles
   * @param legFeatures List of Leg Features(Leg Tracker)
   * @param time The current time
   */
  void publishParticles(list<LegFeaturePtr>& legFeatures, ros::Time time){
    // The pointcloud message
    sensor_msgs::PointCloud particlesPCL;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    particlesPCL.channels.push_back(rgb_channel);

    particlesPCL.header.frame_id = fixed_frame;
    particlesPCL.header.stamp = time;

    for (list<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {
        MCPdf<StatePosVel>* mc = (*legFeatureIt)->filter_.getFilter()->PostGet();

        vector<WeightedSample<StatePosVel> > samples = mc->ListOfSamplesGet();

        for(vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
          geometry_msgs::Point32 point;
          point.x = (*sampleIt).ValueGet().pos_[0];
          point.y = (*sampleIt).ValueGet().pos_[1];
          point.z = (*sampleIt).ValueGet().pos_[2];

          //std::cout << (*sampleIt).WeightGet() << std::endl;

          //
          int r,g,b;
          redGreenGradient((*sampleIt).WeightGet()*200,r,g,b);

          // Set the color according to the probability
          float color_val = 0;
          int rgb = (r << 16) | (g << 8) | b;
          color_val = *(float*) & (rgb);

          if (particlesPCL.channels[0].name == "rgb")
            particlesPCL.channels[0].values.push_back(color_val);

          particlesPCL.points.push_back(point);
        }

    }

    // Publish the pointcloud
    particles_pub_.publish(particlesPCL);

    ROS_DEBUG("DualTracker::%s Publishing Particles on %s", __func__, fixed_frame.c_str());
  }

  void publishLegFeaturesVisualization(list<LegFeaturePtr>& legFeatures, ros::Time time){

    // The pointcloud message
    sensor_msgs::PointCloud legPcl;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    legPcl.channels.push_back(rgb_channel);

    legPcl.header.frame_id = fixed_frame;
    legPcl.header.stamp = time;

    for (list<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {

        if(publish_static_people_trackers_ || (*legFeatureIt)->isDynamic()){
          // Create center Point
          Stamped<tf::Point> center = (*legFeatureIt)->position_;

          geometry_msgs::Point32 point;
          point.x = center[0];
          point.y = center[1];
          point.z = 0;

          legPcl.points.push_back(point);

          // Set the color
          int r,g,b;
          //r = 255;
          getColor((*legFeatureIt)->int_id_,r,g,b);

          // Set the color according to the probability
          float color_val = 0;
          int rgb = (r << 16) | (g << 8) | b;
          color_val = *(float*) & (rgb);

          if (legPcl.channels[0].name == "rgb")
            legPcl.channels[0].values.push_back(color_val);
        }
    }

    // Publish the pointcloud
    leg_features_vis_pub_.publish(legPcl);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

  void publishLegHistory(list<LegFeaturePtr>& legFeatures, ros::Time time){

    for (list<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {

      if((*legFeatureIt)->position_history_.size() > 1){


        // The geometry message
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = fixed_frame;
        line_list.header.stamp = time;
        line_list.id = (*legFeatureIt)->int_id_;
        line_list.ns = "history";

        // width
        line_list.scale.x = 0.01;

        // Set the color

        int r,g,b;
        //r = 255;
        getColor((*legFeatureIt)->int_id_,r,g,b);

        line_list.color.r = r/255.0;
        line_list.color.g = g/255.0;
        line_list.color.b = b/255.0;
        line_list.color.a = 1.0;

        std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator prevPointIt;
        std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator nextPointIt;

        prevPointIt = (*legFeatureIt)->position_history_.begin();
        nextPointIt = (*legFeatureIt)->position_history_.begin();
        nextPointIt++;

        //std::cout << "Creating line!" << std::endl;

        int counter = 0;

        while(nextPointIt != (*legFeatureIt)->position_history_.end()){
          geometry_msgs::Point point0, point1;
          point0.x = (*prevPointIt)->getX();
          point0.y = (*prevPointIt)->getY();
          point0.z = 0;

          point1.x = (*nextPointIt)->getX();
          point1.y = (*nextPointIt)->getY();
          point1.z = 0;

          line_list.points.push_back(point0);
          line_list.points.push_back(point1);

          //std::cout << "[" << counter << "]" << point0.x << " " << point0.y << "---->" << point1.x << " " << point1.y << std::endl;

          prevPointIt++;
          nextPointIt++;
          counter++;
        }

        // Publish the pointcloud
        leg_features_history_vis_pub_.publish(line_list);
      }
    }


    ROS_DEBUG("DualTracker::%s Publishing Leg History on %s", __func__, fixed_frame.c_str());
  }

  void publishPredictedLegPositions(list<LegFeaturePtr>& legFeatures, ros::Time time){

    // Marker Array
    visualization_msgs::MarkerArray markerArray;

    int counter = 0;
    for (list<LegFeaturePtr>::iterator legIt = legFeatures.begin();
        legIt != legFeatures.end();
        legIt++)
    {

      // Cylinder
      visualization_msgs::Marker markerMsgCylinder;

      markerMsgCylinder.header.frame_id = fixed_frame;
      markerMsgCylinder.header.stamp = time;
      markerMsgCylinder.ns = "predictions";
      markerMsgCylinder.id = counter;
      markerMsgCylinder.type = visualization_msgs::Marker::CYLINDER;
      markerMsgCylinder.scale.x = 0.02;
      markerMsgCylinder.scale.y = 0.02;
      markerMsgCylinder.scale.z = 0.08; // height
      markerMsgCylinder.color.r = 1.0;
      markerMsgCylinder.color.a = 0.8;

      markerMsgCylinder.pose.position.x = (*legIt)->position_predicted_.getX();
      markerMsgCylinder.pose.position.y = (*legIt)->position_predicted_.getY();
      markerMsgCylinder.pose.position.z = 0.0;

      markerArray.markers.push_back(markerMsgCylinder);
      counter++;


      // Cylinder
      visualization_msgs::Marker markerMsgArrow;

      markerMsgArrow.header.frame_id = fixed_frame;
      markerMsgArrow.header.stamp = time;
      markerMsgArrow.ns = "arrow_pred_corr";
      markerMsgArrow.id = counter;
      markerMsgArrow.type = visualization_msgs::Marker::ARROW;
      markerMsgArrow.scale.x = 0.005;
      markerMsgArrow.scale.y = 0.02;
      markerMsgArrow.color.r = 1.0;
      markerMsgArrow.color.a = 0.8;

      geometry_msgs::Point point0, point1;
      point0.x = (*legIt)->position_predicted_.getX();
      point0.y = (*legIt)->position_predicted_.getY();
      point0.z = 0.0;

      markerMsgArrow.points.push_back(point0);

      point1.x = (*legIt)->position_.getX();
      point1.y = (*legIt)->position_.getY();
      point1.z = 0.0;

      markerMsgArrow.points.push_back(point1);

      markerArray.markers.push_back(markerMsgArrow);
      counter++;

    }

    leg_predicted_pub_.publish(markerArray);

  }


  void publishMatches(list<LegFeaturePtr>& legFeatures, ros::Time time){


    // The pointcloud message
    visualization_msgs::Marker markerMsg;

    markerMsg.header.frame_id = fixed_frame;
    markerMsg.header.stamp = time;
    markerMsg.ns = "matches";
    markerMsg.id = 0;
    markerMsg.type = visualization_msgs::Marker::LINE_LIST;
    markerMsg.scale.x = 0.01;
    markerMsg.color.r = 1.0;
    markerMsg.color.a = 1.0;

    for (list<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {

      if(abs((time-(*legFeatureIt)->meas_loc_last_update_.stamp_).toSec()) < 0.01){
        geometry_msgs::Point p0;
        p0.x = (*legFeatureIt)->position_[0];
        p0.y = (*legFeatureIt)->position_[1];
        p0.z = 0;

        geometry_msgs::Point p1;
        p1.x = (*legFeatureIt)->meas_loc_last_update_[0];
        p1.y = (*legFeatureIt)->meas_loc_last_update_[1];
        p1.z = 0;

        markerMsg.points.push_back(p0);
        markerMsg.points.push_back(p1);
      }
    }

    // Publish the pointcloud
    matches_vis_pub_.publish(markerMsg);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

  void publishPeopleTracker(ros::Time time){

    // Create the Visualization Message (a marker array)
    visualization_msgs::MarkerArray msgArray;

    // The geometry message
    visualization_msgs::Marker line_list;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.header.frame_id = fixed_frame;
    line_list.header.stamp = time;
    line_list.id = 0;
    line_list.ns = "people_tracker";
    line_list.scale.x = 0.05;

    line_list.color.g = 255.0;
    line_list.color.a = 1.0;

    int counter = 0;
    for(vector<PeopleTrackerPtr>::iterator peopleTrackerIt = people_trackers_.getList()->begin();
        peopleTrackerIt != people_trackers_.getList()->end();
        peopleTrackerIt++){


      if((*peopleTrackerIt)->isValid() // Tracker must be valid
         && (*peopleTrackerIt)->getTotalProbability() > 0.5 // Tracker must have certain probability
         && (publish_static_people_trackers_ || (*peopleTrackerIt)->isDynamic()) // Publish static Trackers
         ){

        // The geometry message
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = fixed_frame;
        line_list.header.stamp = time;
        line_list.id = counter;
        line_list.ns = "people_tracker";
        line_list.scale.x = 0.05*(*peopleTrackerIt)->getTotalProbability();

        line_list.color.g = 255.0;
        line_list.color.a = 1.0;

        geometry_msgs::Point pointLeftLeg, pointLegRight, pointHipLeft, pointHipRight, pointCenter;

        // Leg 0
        pointLeftLeg.x = (*peopleTrackerIt)->getLeftLeg()->position_[0];
        pointLeftLeg.y = (*peopleTrackerIt)->getLeftLeg()->position_[1];
        pointLeftLeg.z = 0;

        // Hip 0
        pointHipLeft.x = (*peopleTrackerIt)->hipPosLeft_[0];
        pointHipLeft.y = (*peopleTrackerIt)->hipPosLeft_[1];
        pointHipLeft.z = 0.0;

        // Center of the Person
        //pointCenter.x = (*peopleTrackerIt)->getEstimate().pos_[0];
        //pointCenter.y = (*peopleTrackerIt)->getEstimate().pos_[1];
        //pointCenter.z = 0.0;

        // Hip 1
        pointHipRight.x = (*peopleTrackerIt)->hipPosRight_[0];
        pointHipRight.y = (*peopleTrackerIt)->hipPosRight_[1];
        pointHipRight.z = 0.0;

        // Leg 1
        pointLegRight.x = (*peopleTrackerIt)->getRightLeg()->position_[0];;
        pointLegRight.y = (*peopleTrackerIt)->getRightLeg()->position_[1];;
        pointLegRight.z = 0;

        line_list.points.push_back(pointLeftLeg);

        // Publish intermediate points for the hips only if the person has some speed at least, otherwise only a straight line
        if((*peopleTrackerIt)->getEstimate().vel_.length() > 0.4){
          line_list.points.push_back(pointHipLeft);
          line_list.points.push_back(pointHipLeft);
        //line_list.points.push_back(pointCenter);

        //line_list.points.push_back(pointCenter);
          line_list.points.push_back(pointHipRight);
          line_list.points.push_back(pointHipRight);
        }
        line_list.points.push_back(pointLegRight);

        // Add the pointlist to the msgArray
        msgArray.markers.push_back(line_list);
        counter++;

      }
    }

    // Publish the marker Array
    people_track_vis_pub_.publish(msgArray);


  }

  // Add Labels to the People Trackers
  void publishPeopleTrackerLabels(ros::Time time){

    // The marker Array
    visualization_msgs::MarkerArray labelArray;

    int counter = 0;
    for(vector<PeopleTrackerPtr>::iterator peopleTrackerIt = people_trackers_.getList()->begin();
        peopleTrackerIt != people_trackers_.getList()->end();
        peopleTrackerIt++){

      if((*peopleTrackerIt)->getTotalProbability()>0.1 &&
          (publish_static_people_trackers_ || (*peopleTrackerIt)->isDynamic()))
      {
      visualization_msgs::Marker label;
      label.header.stamp = time;
      label.header.frame_id = fixed_frame;
      label.ns = "PEOPLE_TRACKER_LABEL";
      label.id = counter;
      label.type = label.TEXT_VIEW_FACING;
      label.pose.position.x = (*peopleTrackerIt)->getEstimate().pos_[0];
      label.pose.position.y = (*peopleTrackerIt)->getEstimate().pos_[1];
      label.pose.position.z = 0.3;
      label.scale.z = .17;
      label.color.a = 1;
      label.lifetime = ros::Duration(0.5);

      // Add text
      char buf[100];
      sprintf(buf, "#PT%d-%d-p%.2f", (*peopleTrackerIt)->id_[0], (*peopleTrackerIt)->id_[1], (*peopleTrackerIt)->getTotalProbability());
      label.text = buf;

      labelArray.markers.push_back(label);

      counter++;
      }
    }
    // Publish
    people_track_label_pub_.publish(labelArray);
  }

  // Add Labels to the People Trackers
  void publishPeopleHistory(boost::shared_ptr<vector<PeopleTrackerPtr> > peopleTracker, ros::Time time){

    visualization_msgs::MarkerArray msgArray;

    int counter = 0;
    for (vector<PeopleTrackerPtr>::iterator peopleIt = peopleTracker->begin();
        peopleIt != peopleTracker->end();
        peopleIt++)
    {

      if
      (
          (*peopleIt)->getHistorySize() > 1 &&
          (*peopleIt)->getTotalProbability() > 0.5 &&
          (*peopleIt)->isDynamic()
      )
      {

        // The geometry message
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = fixed_frame;
        line_list.header.stamp = time;
        line_list.id = counter;
        line_list.ns = "people_history";

        // width
        line_list.scale.x = 0.03;

        // Set the color

        int r,g,b;
        //r = 255;
        //getColor((*legFeatureIt)->int_id_,r,g,b);

        line_list.color.r = 255.0;
        line_list.color.g = 0;
        line_list.color.b = 0;
        line_list.color.a = 1.0;

        std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator prevPointIt;
        std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator nextPointIt;

        prevPointIt = (*peopleIt)->position_history_.begin();
        nextPointIt = (*peopleIt)->position_history_.begin();
        nextPointIt++;

        //std::cout << "Creating line!" << std::endl;



        while(nextPointIt != (*peopleIt)->position_history_.end()){
          geometry_msgs::Point point0, point1;
          point0.x = (*prevPointIt)->getX();
          point0.y = (*prevPointIt)->getY();
          point0.z = 0;

          point1.x = (*nextPointIt)->getX();
          point1.y = (*nextPointIt)->getY();
          point1.z = 0;

          line_list.points.push_back(point0);
          line_list.points.push_back(point1);

          //std::cout << "[" << counter << "]" << point0.x << " " << point0.y << "---->" << point1.x << " " << point1.y << std::endl;

          prevPointIt++;
          nextPointIt++;

        }
        counter++;

        // Publish the pointcloud
        msgArray.markers.push_back(line_list);

      }
    }

    people_history_vis_pub_.publish(msgArray);
    //leg_features_history_vis_pub_.publish(line_list);

    ROS_DEBUG("DualTracker::%s Publishing Leg History on %s", __func__, fixed_frame.c_str());
  }

  // Publish the occlusion model for debugging purposes
  void publishOcclusionModel(list<LegFeaturePtr>& legFeatures, ros::Time time){
    // The pointcloud message
    sensor_msgs::PointCloud pcl;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    pcl.channels.push_back(rgb_channel);

    pcl.header.frame_id = fixed_frame;
    pcl.header.stamp = time;

    for (list<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {
        MCPdf<StatePosVel>* mc = (*legFeatureIt)->filter_.getFilter()->PostGet();

        vector<WeightedSample<StatePosVel> > samples = mc->ListOfSamplesGet();

        for(vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
          geometry_msgs::Point32 point;
          point.x = (*sampleIt).ValueGet().pos_[0];
          point.y = (*sampleIt).ValueGet().pos_[1];
          point.z = (*sampleIt).ValueGet().pos_[2];

          tf::Vector3 p((*sampleIt).ValueGet().pos_[0],(*sampleIt).ValueGet().pos_[1],(*sampleIt).ValueGet().pos_[2]);

          // Set the color according to the occlusion model
          tf::Stamped<tf::Vector3>* p_stamp = new tf::Stamped<tf::Vector3>(p, time, fixed_frame);
          double prob = occlusionModel_->getOcclusionProbability(*p_stamp);

          int r,g,b;
          redGreenGradient(prob,r,g,b);

          // Set the color according to the probability
          float color_val = 0;
          int rgb = (r << 16) | (g << 8) | b;
          color_val = *(float*) & (rgb);

          if (pcl.channels[0].name == "rgb")
            pcl.channels[0].values.push_back(color_val);

          pcl.points.push_back(point);
        }

    }

    // Publish the pointcloud
    occlusion_model_pub_.publish(pcl);

    ROS_DEBUG("DualTracker::%s Publishing Particles on %s", __func__, fixed_frame.c_str());
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dual_tracker");
  g_argc = argc;
  g_argv = argv;
  ros::NodeHandle nh;
  DualTracker ld(nh);

  std::cout << "DUAL TRACKER started!!!" << std::endl;
  ROS_INFO("DUAL TRACKER started!");

  ros::spin();

  return 0;
}

