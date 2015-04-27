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
#include <leg_detector/constants.h>
#include <leg_detector/dual_tracker.h>
#include <leg_detector/DualTrackerConfig.h>
#include <leg_detector/laser_processor.h>
#include <leg_detector/calc_leg_features.h>
#include <leg_detector/visualization_conversions.h>
#include <benchmarking/timer.h>
#include <leg_detector/leg_feature.h>
#include <leg_detector/people_tracker.h>
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

// Transforms
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

// People tracking
#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>

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
// static string fixed_frame              = "odom_combined";  // The fixed frame in which ? //TODO find out


// Defines
#define DUALTRACKER_DEBUG 1         // Debug the leg detector
#define DUALTRACKER_TIME_DEBUG 1    // Debug the calculation time inside the leg_detector

class MatchedFeature
{
public:
  SampleSet* candidate_;  // The point cluster
  LegFeature* closest_; // The feature/leg tracker
  float distance_;		  // The distance between the
  double probability_;

  MatchedFeature(SampleSet* candidate, LegFeature* closest, float distance, double probability)
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


// actual legdetector node
class DualTracker
{
public:
  NodeHandle nh_; /**< The node handle */

  TransformListener tfl_; /**< The transform listener */

  ScanMask mask_; /**< A scan mask */

  int mask_count_;

  CvRTrees forest; /**< The forest classificator */

  float connected_thresh_; /**< Parameter for the clustering(Creation of SampleSets) */

  int feat_count_; /**< Number of features evaluated for each SampleSet */

  char save_[100];

  std::vector<PeopleTrackerPtr> people_tracker_;

  //list<SavedFeature*> saved_features_; /**< List of SavedFeatures that are currently tracked*/

  list<LegFeature*> saved_leg_features; /**< List of SavedFeatures(Legs) that are currently tracked*/

  boost::mutex saved_mutex_; /**< Mutex to handle the access to the Saved Features */

  int feature_id_;

  bool use_seeds_;

  bool publish_legs_, publish_people_, publish_leg_markers_, publish_people_markers_, publish_clusters_, publish_particles_, publish_matches_, publish_leg_history_;

  int next_p_id_;

  double leg_reliability_limit_;

  int min_points_per_group_;

  unsigned int cycle_; /**< Cycle counter to count the filter cycles */

  benchmarking::Timer cycleTimer; /**< Timer to measure the cycle time */

  // The publishers
  ros::Publisher people_measurements_pub_; /**< Publisher for people measurements */
  ros::Publisher leg_measurements_pub_; /**< Publisher for leg measurements */
  ros::Publisher markers_pub_; /**< Publisher for features */
  ros::Publisher clusters_pub_;/**< Publisher for the clusters generated by scan processor */
  ros::Publisher leg_measurements_vis_pub_;/**< Visualization of leg detections */
  ros::Publisher leg_features_vis_pub_;/**< Visualization of leg tracks */
  ros::Publisher leg_features_history_vis_pub_;/**< Visualization of leg tracks */
  ros::Publisher matches_vis_pub_;/**< Visualization of the pairing leg_detection <-> leg_track */
  ros::Publisher particles_pub_;/**< Visualization of particles */
  dynamic_reconfigure::Server<leg_detector::DualTrackerConfig> server_; /**< The configuration server*/

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
    cycle_(0)
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
    leg_measurements_pub_ = nh_.advertise<people_msgs::PositionMeasurementArray>("leg_tracker_measurements", 0);
    people_measurements_pub_ = nh_.advertise<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 0);
    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    leg_features_history_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("leg_track_history", 0);
    clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud>("clusters", 0);
    particles_pub_ = nh_.advertise<sensor_msgs::PointCloud>("particles", 0);

    // Visualization topics
    leg_measurements_vis_pub_= nh_.advertise<sensor_msgs::PointCloud>("leg_measurements", 0);
    leg_features_vis_pub_ = nh_.advertise<sensor_msgs::PointCloud>("leg_features", 0);
    //matches_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);




    if (use_seeds_)
    {
      //people_notifier_.registerCallback(boost::bind(&DualTracker::peopleCallback, this, _1));
      //people_notifier_.setTolerance(ros::Duration(0.01));
    }

    // Set the laserCallback
    laser_notifier_.registerCallback(boost::bind(&DualTracker::laserCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));

    dynamic_reconfigure::Server<leg_detector::DualTrackerConfig>::CallbackType f;
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

  void configure(leg_detector::DualTrackerConfig &config, uint32_t level)
  {
    connected_thresh_       = config.connection_threshold;    ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - connected_thresh_ %f", __func__, connected_thresh_ );
    min_points_per_group_    = config.min_points_per_group;   ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - min_points_per_group %i", __func__, min_points_per_group_ );
    leg_reliability_limit_  = config.leg_reliability_limit;   ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - leg_reliability_limit_ %f", __func__, leg_reliability_limit_ );
    publish_legs_           = config.publish_legs;            ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_legs_ %d", __func__, publish_legs_ );
    publish_people_         = config.publish_people;          ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_ %d", __func__, publish_people_ );
    publish_leg_markers_    = config.publish_leg_markers;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_leg_markers_ %d", __func__, publish_leg_markers_ );
    publish_people_markers_ = config.publish_people_markers;  ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_markers_ %d", __func__, publish_people_markers_ );
    publish_clusters_       = config.publish_clusters;        ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_clusters_ %d", __func__, publish_clusters_ );
    publish_particles_      = config.publish_particles;       ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_particles_ %d", __func__, publish_particles_ );
    publish_matches_        = config.publish_matches;         ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_clusters_ %d", __func__, publish_clusters_ );
    publish_leg_history_    = config.publish_leg_history;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_leg_history_ %d", __func__, publish_leg_history_ );


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

    //kal_p                    = config.kalman_p;
    //kal_q                    = config.kalman_q;
    //kal_r                    = config.kalman_r;
    //use_filter               = config.kalman_on == 0;

    ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - Configuration done", __func__);
  }

  /**
   *  @brief The distance between two legs.
   *
   *  Calculates the euclidian distance between to features(legs)
   */
  double distance(list<LegFeature*>::iterator it1,  list<LegFeature*>::iterator it2)
  {
    Stamped<Point> one = (*it1)->position_;
    Stamped<Point> two = (*it2)->position_;

    double distance = (one-two).length();

    return distance;
  }

  /**
   * Generate People Tracker based on the Pairing of legs
   */
  void pairLegsToPeopleTracker(){
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Pairing Legs",__func__);
  }

  /**
   *  @brief Pair the Saved Features
   *
   *  Pairs the features inside the member saved_features_(leg_trackers) with each other based on the distance.
   *  //NEW Determine all possible pairs, neglect pairing based on previous observations
   */
  void pairLegs()
  {
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Pairing Legs",__func__);
    benchmarking::Timer pairLegsTimer; pairLegsTimer.start();

    // Deal With legs that already have ids
    list<LegFeature*>::iterator begin = saved_leg_features.begin();
    list<LegFeature*>::iterator end = saved_leg_features.end();
    list<LegFeature*>::iterator leg1, leg2, best, it;

    for (leg1 = begin; leg1 != end; ++leg1)
    {
      // If this leg has no id, skip
      if ((*leg1)->object_id == "")
        continue;

      leg2 = end;
      best = end;
      double closest_dist = leg_pair_separation_m;
      for (it = begin; it != end; ++it)
      {
        if (it == leg1) continue;

        if ((*it)->object_id == (*leg1)->object_id)
        {
          leg2 = it;
          break;
        }

        if ((*it)->object_id != "")
          continue;

        double d = distance(it, leg1);
        if (((*it)->getLifetime() <= max_second_leg_age_s)
            && (d < closest_dist))
        {
          closest_dist = d;
          best = it;
        }

      }

      if (leg2 != end)
      {
        double dist_between_legs = distance(leg1, leg2);
        if (dist_between_legs > leg_pair_separation_m)
        {
          (*leg1)->object_id = "";
          (*leg1)->other = NULL;
          (*leg2)->object_id = "";
          (*leg2)->other = NULL;
        }
        else
        {
          (*leg1)->other = *leg2;
          (*leg2)->other = *leg1;
        }
      }
      else if (best != end)
      {
        (*best)->object_id = (*leg1)->object_id;
        (*leg1)->other = *best;
        (*best)->other = *leg1;
      }
    }

    // Attempt to pair up legs with no id
    for (;;)
    {
      list<LegFeature*>::iterator best1 = end, best2 = end;
      double closest_dist = leg_pair_separation_m;

      for (leg1 = begin; leg1 != end; ++leg1)
      {
        // If this leg has an id or low reliability, skip
        if ((*leg1)->object_id != ""
            || (*leg1)->getReliability() < leg_reliability_limit_)
          continue;

        for (leg2 = begin; leg2 != end; ++leg2)
        {
          if (((*leg2)->object_id != "") // has no id
              || ((*leg2)->getReliability() < leg_reliability_limit_) // is below leg_reliability_limit_
              || (leg1 == leg2)) continue; // is unequal with the other leg
          double d = distance(leg1, leg2); // Calculate the distance
          if (d < closest_dist)
          {
            best1 = leg1;
            best2 = leg2;
          }
        }
      }

      if (best1 != end)
      {
        char id[100];
        snprintf(id, 100, "Person%d", next_p_id_++);
        (*best1)->object_id = std::string(id);
        (*best2)->object_id = std::string(id);
        (*best1)->other = *best2;
        (*best2)->other = *best1;
      }
      else
      {
        break;
      }
    }
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"LegDetector::%s - Pairing Legs took %f ms",__func__, pairLegsTimer.stopAndGetTimeMs());


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
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Received Laserscan",__func__);

    // Start Cycle Timer
    cycleTimer.start();

    //////////////////////////////////////////////////////////////////////////
    //// Create clusters
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
    //// Remove the Trackers
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sRemoving old Trackers [Cycle %u]", BOLDWHITE, cycle_);

    // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);

    benchmarking::Timer removeTimer; removeTimer.start();
    // Iterate through the saved features and remove those who havent been observed since (no_observation_timeout_s)
    int deletionCounter = 0;
    int numberOfSavedFeaturesBefore = saved_leg_features.size();
    list<LegFeature*>::iterator sf_iter = saved_leg_features.begin();
    while (sf_iter != saved_leg_features.end())
    {
      // If there was no measurement of this feature in the last 'no_observation_timeout_s' seconds-> removed it and clear the link of its partner
      // IDEA_ make this dependent on the observation model
      if ((*sf_iter)->meas_time_ < purge)
      {
        if ((*sf_iter)->other) // If leg is paired
          (*sf_iter)->other->other = NULL; //Remove link to itself from partner
        delete(*sf_iter);
        saved_leg_features.erase(sf_iter++);
        ++deletionCounter;
      }
      else
        ++sf_iter;
    }
    removeTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Removed %i of %i features because the havent been detected in the last %f seconds",__func__, deletionCounter, numberOfSavedFeaturesBefore, no_observation_timeout_s);
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"LegDetector::%s - Removing features took %f ms",__func__, removeTimer.getElapsedTimeMs());

    ROS_DEBUG("%sRemoving old Trackers done! [Cycle %u]", BOLDWHITE, cycle_);
    //////////////////////////////////////////////////////////////////////////
    //// Propagation/Prediction using the motion model-
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sPrediction [Cycle %u]", BOLDWHITE, cycle_);

    // System update of trackers, and copy updated ones in propagate list
    benchmarking::Timer propagationTimer; propagationTimer.start();
    list<LegFeature*> propagated;
    for (list<LegFeature*>::iterator sf_iter = saved_leg_features.begin();
         sf_iter != saved_leg_features.end();
         sf_iter++)
    {
      (*sf_iter)->propagate(scan->header.stamp); // Propagate <-> Predict the filters
      propagated.push_back(*sf_iter);
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

        //std::cout << (*clusterIt)->center().getX() << " " << (*clusterIt)->center().getY() << " " << (*clusterIt)->center().getZ() << std::endl;
      }

    }

    // Create test data



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
    for (vector<LegDetectionProb>::iterator detectionIt = detections.begin();
        detectionIt != detections.end();
        detectionIt++)
    {
      assert(detectionIt->point.getZ() == 0);
      //std::cout << detectionIt->point.getZ() << std::endl;
    }

    unsigned int newTrackCounter = 0;
    unsigned int matchesCounter = 0;

    multiset<MatchedFeature> matches;
//    for (list<SampleSet*>::iterator clusterIt = processor.getClusters().begin();
//         clusterIt != processor.getClusters().end();
//         clusterIt++)
//    {
    for (vector<LegDetectionProb>::iterator detectionIt = detections.begin();
        detectionIt != detections.end();
        detectionIt++)
    {

      Stamped<Point> loc = detectionIt->point;
      SampleSet* cluster = detectionIt->cluster;



      // Find the closest tracker (Note that the tracker has been updated using the kalman filter!)
      // Multiple measurements could be assigned to the same tracker! This problem is solved below. Better methods could be thought of.
      // IDEA_ Do this better! The closest is no necessarily the right one
      list<LegFeature*>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_m;

      // Iterate through the trackers
      for (list<LegFeature*>::iterator pf_iter = propagated.begin();
           pf_iter != propagated.end();
           pf_iter++)
      {
        // find the closest distance between candidate and trackers
        float dist = loc.distance((*pf_iter)->position_);
        if (dist < closest_dist)
        {
          closest = pf_iter;
          closest_dist = dist;
        }
      }
      // Nothing close to it, start a new track
      if (closest == propagated.end())
      {
        loc.setZ(0); // TODO ugly fix
        list<LegFeature*>::iterator new_saved = saved_leg_features.insert(saved_leg_features.end(), new LegFeature(loc, tfl_));
        ++newTrackCounter;
      }
      // Add the candidate, the tracker and the distance to a match list
      else{
        matches.insert(MatchedFeature(cluster, *closest, closest_dist, cluster->getProbability()));
        ++matchesCounter;
      }

    }// end iterate the clusters

    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"DualTracker::%s - Associated tracks to legs - %i matches - %i new tracks",__func__, matchesCounter, newTrackCounter);
    ROS_DEBUG("%sMatching Done! [Cycle %u]", BOLDWHITE, cycle_);

    //////////////////////////////////////////////////////////////////////////
    //// Combination of saved features to people tracker
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sHigh Level Association [Cycle %u]", BOLDWHITE, cycle_);
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"DualTracker::%s - Starting to combine %lu leg_tracks to people tracker",__func__, saved_leg_features.size());

    // Do the combinations
    for (list<LegFeature*>::iterator legIt0 = saved_leg_features.begin();
        legIt0 != saved_leg_features.end();
        legIt0++)
    {
      list<LegFeature*>::iterator legIt1 = boost::next(legIt0,1);
      for (;
          legIt1 != saved_leg_features.end();
          legIt1++)
      {



        //std::cout << "Investigation of combination " << (*legIt0)->int_id_ << " - " << (*legIt1)->int_id_ << std::endl;
      }
    }

    // Evaluate the combinations (What is the probability for this people tracker
    //assert(false);

    //return;
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
      list<LegFeature*>::iterator pf_iter = propagated.begin(); // Tracker iterator
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

        list<LegFeature*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;

        for (list<LegFeature*>::iterator remain_iter = propagated.begin();
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
          list<LegFeature*>::iterator new_saved = saved_leg_features.insert(saved_leg_features.end(), new LegFeature(loc, tfl_));

        }
        else{
          matches.insert(MatchedFeature(matched_iter->candidate_, *closest, closest_dist, matched_iter->probability_));

        }
          matches.erase(matched_iter);
      }
    }
    ROS_DEBUG("%sUpdate done! [Cycle %u]", BOLDWHITE, cycle_);

    //////////////////////////////////////////////////////////////////////////
    //// Publish data
    //////////////////////////////////////////////////////////////////////////

    ROS_DEBUG("%sPublishing [Cycle %u]", BOLDWHITE, cycle_);

    // Publish the detections of legs
    if(publish_legs_){
      publishLegMeasurements(processor.getClusters(), scan->header.stamp, scan->header.frame_id);
    }


    if(publish_clusters_){
      publishClusters(processor.getClusters(), scan->header.stamp, scan->header.frame_id);
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
      publishMatches(matches, scan->header.stamp, scan);
    }

    // Publish leg Measurements on
    if(publish_legs_){
      //publishLegMeasurementArray(saved_leg_features);
    }

    ROS_DEBUG("%sPublishing done! [Cycle %u]", BOLDWHITE, cycle_);
    //////////////////////////////////////////////////////////////////////////
    //// Cleaning (Clear data)
    //////////////////////////////////////////////////////////////////////////
    cvReleaseMat(&tmp_mat);
    tmp_mat = 0;
    //if (use_seeds_)
    //  pairLegs();


    //////////////////////////////////////////////////////////////////////////
    //// Finalize the Cycle
    //////////////////////////////////////////////////////////////////////////
    cycleTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"%sCycle %u took %.2f ms to complete", BOLDCYAN, cycle_, cycleTimer.stopAndGetTimeMs());
    //assert(cycle_ < 2);

    // Iterate the cycle counter
    ++cycle_;
 }

  /**
   * Publish a list of pointers to legFeatures
   * @param legFeatures List of Leg Feature Pointers
   */
  void publishLegMeasurementArray(std::list<LegFeature*> legFeatures){

    // Abort if List is empty
    if(legFeatures.size() == 0){
      ROS_WARN("Called publishLegMeasurementArray, but the given list is empty. Nothing to publish.");
      return;
    }

    // Iterator variable
    int i = 0;
    vector<people_msgs::PositionMeasurement> legs;

    // Iterate the features
    for (list<LegFeature*>::iterator sf_iter = legFeatures.begin();
         sf_iter != saved_leg_features.end();
         sf_iter++, i++)
    {
      // reliability
      double reliability = (*sf_iter)->getReliability();

      if ((*sf_iter)->getReliability() > leg_reliability_limit_ && publish_legs_)
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
  void publishParticles(list<LegFeature*>& legFeatures, ros::Time time){
    // The pointcloud message
    sensor_msgs::PointCloud particlesPCL;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    particlesPCL.channels.push_back(rgb_channel);

    particlesPCL.header.frame_id = fixed_frame;
    particlesPCL.header.stamp = time;

    for (list<LegFeature*>::iterator legFeatureIt = legFeatures.begin();
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

  void publishLegFeaturesVisualization(list<LegFeature*>& legFeatures, ros::Time time){

    // The pointcloud message
    sensor_msgs::PointCloud legPcl;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    legPcl.channels.push_back(rgb_channel);

    legPcl.header.frame_id = fixed_frame;
    legPcl.header.stamp = time;

    for (list<LegFeature*>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {
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

    // Publish the pointcloud
    leg_features_vis_pub_.publish(legPcl);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

  void publishLegHistory(list<LegFeature*>& legFeatures, ros::Time time){


    for (list<LegFeature*>::iterator legFeatureIt = legFeatures.begin();
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



        std::list<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator prevPointIt;
        std::list<boost::shared_ptr<tf::Stamped<tf::Point> > >::iterator nextPointIt;

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

  void publishMatches(multiset<MatchedFeature> matches, ros::Time time, const sensor_msgs::LaserScan::ConstPtr& scan){
    // The pointcloud message
    visualization_msgs::Marker markerMsg;

    markerMsg.header.frame_id = fixed_frame;
    markerMsg.header.stamp = time;
    markerMsg.ns = "matches";
    markerMsg.id = 0;
    markerMsg.type = visualization_msgs::Marker::LINE_LIST;
    markerMsg.scale.x = 0.05;
    markerMsg.color.r = 1.0;
    markerMsg.color.a = 1.0;

    for(multiset<MatchedFeature>::iterator matched_iter = matches.begin(); matched_iter != matches.end(); ++matched_iter){

      geometry_msgs::Point p0;
      p0.x = matched_iter->closest_->position_[0];
      p0.y = matched_iter->closest_->position_[1];
      p0.z = 0;



      Stamped<tf::Point> detection(matched_iter->candidate_->center(), time, scan->header.frame_id);
      // Transform the Point in the fixed frame
      try
      {
        tfl_.transformPoint(fixed_frame, detection, detection);
      }
      catch (...)
      {
        ROS_WARN("TF exception in publishClusters!");
      }

      geometry_msgs::Point p1;
      p1.x = detection[0];
      p1.y = detection[1];
      p1.z = 0;

      markerMsg.points.push_back(p0);
      markerMsg.points.push_back(p1);

    }

    // Publish the pointcloud
    markers_pub_.publish(markerMsg);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dual_tracker");
  g_argc = argc;
  g_argv = argv;
  ros::NodeHandle nh;
  DualTracker ld(nh);
  ros::spin();

  return 0;
}

