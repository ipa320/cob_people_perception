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
#undef NDEBUG
#include <ros/ros.h>
#include <limits>

// Own includes
//#include <leg_detector/constants.h>
#include <dual_people_leg_tracker/dual_tracker.h>
#include <dual_people_leg_tracker/DualTrackerConfig.h>
#include <dual_people_leg_tracker/detection/detection.h>
#include <dual_people_leg_tracker/jpda/association_pair.h>
#include <dual_people_leg_tracker/jpda/unique_association_builder.h>
#include <dual_people_leg_tracker/math/math_functions.h>
#include <dual_people_leg_tracker/jpda/murty.h>
#include <dual_people_leg_tracker/jpda/jpda.h>
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

// DLib include
#include <dlib/optimization/max_cost_assignment.h>

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

static int maxCosts = 10000;

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

// Helper Functions
bool isLegFeatureValid(const LegFeaturePtr & o){
  return !o->isValid();
}

bool sampleWeightCompare(WeightedSample<StatePosVel> i, WeightedSample<StatePosVel> j) { return i.WeightGet()<j.WeightGet(); }


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

  vector<LegFeaturePtr> saved_leg_features; /**< List of SavedFeatures(Legs) that are currently tracked*/

  PeopleTrackerList people_trackers_; /**< Object to handle the people_trackers */

  boost::mutex saved_mutex_; /**< Mutex to handle the access to the Saved Features */

  int feature_id_;

  bool use_seeds_;

  Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic> costMatrixMAP;

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
  bool publish_leg_labels_; /**<Publish the leg labels */
  bool publish_jpda_associations_;/**< Publish the JPDA association probabilities */
  bool publish_measurement_labels_; /**< Publish labels of measurements */
  bool publish_predicted_leg_positions_; /**< Publish the estimated position of the legs due to the prediction of the associated people tracker */
  bool publish_scans_lines_; /**< Publish visualizations of the scan lines */

  int next_p_id_;

  double leg_reliability_limit_;     /** Probability for a leg detection to be considered a leg */
  double new_track_min_probability_; /**< Probability a detection needs to initialize a new leg tracker, this reduces clutter creating false tracks */
  double new_track_creation_likelihood_; /** If a measurement */

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
  ros::Publisher map_pub_; /**< Publishes labels of people tracks */
  ros::Publisher occlusion_model_pub_; /**< Published the occlusion probability */
  ros::Publisher leg_predicted_pub_; /**< Published the occlusion probability */
  ros::Publisher scan_lines_pub_; /**< Publish the laserscan as lines for debugging */
  ros::Publisher leg_label_pub_; /**< Publish leg labels */
  ros::Publisher jpda_association_pub_; /**< Publish the jpda association probability for debugging purposes */
  ros::Publisher measurement_label_pub_; /**< Publish measurements */
  ros::Publisher particles_arrow_pub_; /** < Publish some particles velocity */

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
    occlusionModel_(new OcclusionModel(tfl_)),
    new_track_creation_likelihood_(0.5)
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
    scan_lines_pub_               = nh_.advertise<visualization_msgs::Marker>("scan_lines", 0);
    leg_label_pub_                = nh_.advertise<visualization_msgs::MarkerArray>("leg_labels", 0);
    jpda_association_pub_         = nh_.advertise<visualization_msgs::MarkerArray>("jpda_association", 0);
    measurement_label_pub_        = nh_.advertise<visualization_msgs::MarkerArray>("measurement_label", 0);
    particles_arrow_pub_          = nh_.advertise<visualization_msgs::MarkerArray>("particle_arrows", 0);
    map_pub_ 					  = nh_.advertise<visualization_msgs::MarkerArray>("fake_measurements", 0);

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
    new_track_min_probability_  = config.new_track_min_probability; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - new_track_min_probability %f", __func__, new_track_min_probability_ );

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
    publish_leg_labels_         = config.publish_leg_labels;      ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_leg_labels_ %d", __func__, publish_leg_labels_ );
    publish_measurement_labels_ = config.publish_measurement_labels; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_measurement_labels_ %d", __func__, publish_measurement_labels_);
    publish_predicted_leg_positions_ = config.publish_predicted_leg_positions; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_predicted_leg_positions_ %d", __func__, publish_predicted_leg_positions_);
    publish_scans_lines_ 		= config.publish_scans_lines; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_scans_lines_ %d", __func__, publish_scans_lines_);

    // Publish the people tracker
    publish_people_             = config.publish_people;          ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_ %d", __func__, publish_people_ );
    publish_people_markers_     = config.publish_people_markers;  ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_markers_ %d", __func__, publish_people_markers_ );
    publish_people_tracker_     = config.publish_people_tracker;  ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_tracker_ %d", __func__, publish_people_tracker_ );
    publish_static_people_trackers_ = config.publish_static_people_trackers; ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_static_people_trackers_ %d", __func__, publish_static_people_trackers_);
    publish_people_history_     = config.publish_people_history;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_people_history_ %d", __func__, publish_people_history_ );


    publish_particles_          = config.publish_particles;       ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_particles_ %d", __func__, publish_particles_ );
    publish_matches_            = config.publish_matches;         ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_matches_ %d", __func__, publish_matches_ );

    publish_occlusion_model_    = config.publish_occlusion_model;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_occlusion_model_ %d", __func__, publish_occlusion_model_ );

    // JPDA Publications
    publish_jpda_associations_  = config.publish_jpda_associations;     ROS_DEBUG_COND(DUALTRACKER_DEBUG, "DualTracker::%s - publish_jpda_associations_ %d", __func__, publish_jpda_associations_ );

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

    // Get the zero point of the scan
    tf::Stamped<tf::Point> sensorCoord(tf::Point(0,0,0), scan->header.stamp, scan->header.frame_id);
    tfl_.transformPoint(fixed_frame, sensorCoord, sensorCoord); //Transform using odometry information into the fixed frame
    sensorCoord.setZ(0);

    //////////////////////////////////////////////////////////////////////////
    //// Update the occlusion model(this model is hold by every tracker!) with the current scan
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

    vector<LegFeaturePtr>::iterator sf_iter = saved_leg_features.begin();
    for(vector<LegFeaturePtr>::iterator sf_iter = saved_leg_features.begin();
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

    // Remove invalid associations of the leg Features
    for(vector<LegFeaturePtr>::iterator leg_it = saved_leg_features.begin();
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

    /// People Tracker Propagation
    // High level propagation
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > pplTrackers = people_trackers_.getList();
    for(std::vector<PeopleTrackerPtr>::iterator pplTrackerIt = pplTrackers->begin();
        pplTrackerIt != pplTrackers->end();
        pplTrackerIt++)
    {
      (*pplTrackerIt)->propagate(scan->header.stamp);
      // MAYBE HERE OUTPUT textfile containing person tracker history
    }

    // System update of trackers, and copy updated ones in propagate list
    /// Leg Tracker propagation
    vector<LegFeaturePtr> propagated;
    for (vector<LegFeaturePtr>::iterator legIt = saved_leg_features.begin();
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

    std::vector<DetectionPtr> detections; // vector of leg detections along with their probabilities

    // OpenCV Matrix for the feature values
    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

    std::list<SampleSet *> clusters = processor.getClusters();

    for (list<SampleSet*>::iterator clusterIt = clusters.begin();
         clusterIt != clusters.end();
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


      // Transform the point into the fixed frame
      Stamped<Point> loc((*clusterIt)->center(), scan->header.stamp, scan->header.frame_id);
      try
      {
        tfl_.transformPoint(fixed_frame, loc, loc); //Transform using odometry information into the fixed frame
        loc.setZ(0); // Ugly //TODO
        loc.frame_id_ = fixed_frame;
      }
      catch (...)
      {
        ROS_WARN("TF exception spot 3.");
      }


      if((*clusterIt)->getProbability() > leg_reliability_limit_){
        DetectionPtr detection(new Detection(detections.size(),loc,(*clusterIt)));

        //LegDetectionProb detection;
        //detection->point_ = loc;
        //detection->cluster_ = (*clusterIt);
        //detection->id_ = detections.size();

        // Add to the detections
        detections.push_back(detection);
        ROS_ASSERT(loc.getZ() == 0); //TODO Remove
      }

    }

    // Iterate through all detections
    for (vector<DetectionPtr>::iterator detectionIt = detections.begin();
        detectionIt != detections.end();
        detectionIt++)
    {
      std::cout << "LM " << std::setw(2) << (*detectionIt)->id_ << std::setw(3) << " x:" << (*detectionIt)->point_[0] << " y:" << (*detectionIt)->point_[1] << " prob:" << (*detectionIt)->cluster_->getProbability() << std::endl;
    }

    ROS_DEBUG("%sDetection done! [Cycle %u]", BOLDWHITE, cycle_);


    //////////////////////////////////////////////////////////////////////////
    //// MAP
    //////////////////////////////////////////////////////////////////////////

    int nMeasurementsReal = detections.size();
    int nLegsTracked = propagated.size();


    vector<DetectionPtr> fakeDetections;
    // Calculate the fake measurements
    boost::shared_ptr<std::vector<PeopleTrackerPtr> > ppls = people_trackers_.getList();

    for(std::vector<PeopleTrackerPtr>::iterator pplIt = ppls->begin(); pplIt != ppls->end(); pplIt++){

    	std::cout << "Counting number of measurements" << std::endl;
    	if((*pplIt)->getTotalProbability() > 0.4){

    		int numberOfMeasurementsWithinRange = 0;
    		double rangeThres = 1;
    	    for (vector<DetectionPtr>::iterator detectionIt = detections.begin();
    	        detectionIt != detections.end();
    	        detectionIt++)
    	    {
    	    	Stamped<Point> loc = (*detectionIt)->point_;
    	    	double dist = loc.distance((*pplIt)->pos_vel_estimation_.pos_);

    	    	if(dist < rangeThres){
    	    		numberOfMeasurementsWithinRange++;
    	    	}

    	    }
    		std::cout << **pplIt << std::endl;
    		std::cout << numberOfMeasurementsWithinRange << std::endl;

        	// Partial occlusion
        	if(numberOfMeasurementsWithinRange == 1){
        		// Calculate a fake detection
        		tf::Vector3 pplPos((*pplIt)->pos_vel_estimation_.pos_);
        		tf::Vector3 widthVec = pplPos - sensorCoord;
        		widthVec.normalize();

        		widthVec *= 0.5;

        		tf::Vector3 fakeMeasPos = pplPos + widthVec;

        		tf::Stamped<tf::Vector3> fakeLoc = tf::Stamped<tf::Vector3>(fakeMeasPos, scan->header.stamp, scan->header.frame_id);

        		double fakeLegProb = 0.95;

        		// Create the detection
        		DetectionPtr fakeDetection(new Detection(fakeDetections.size() + detections.size(), fakeLoc, fakeLegProb));

        		fakeDetections.push_back(fakeDetection);
        	}
    	}

    }

    int nMeasurementsFake = fakeDetections.size();
    std::cout << RED << "nMeasurementsFake" << nMeasurementsFake << RESET << std::endl;



    costMatrixMAP = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Zero(nLegsTracked,nMeasurementsReal + nMeasurementsFake);
    costMatrixMAP.resize(nLegsTracked, nMeasurementsReal + nMeasurementsFake);

    int row = 0;
    for (vector<LegFeaturePtr>::iterator legIt = propagated.begin(); legIt != propagated.end(); legIt++)
    {
    	int col = 0;
    	for (vector<DetectionPtr>::iterator detectionIt = detections.begin(); detectionIt != detections.end(); detectionIt++)
    	{
        Stamped<Point> loc = (*detectionIt)->point_;

        double prob = (*legIt)->getMeasurementProbability(loc);
        double negLogLike = -log(prob);


        // find the closest distance between candidate and trackers
        // float dist = loc.distance((*legIt)->position_);
        // TODO: Where exactly is loc to be expected? Should it be calculated based on particles?

        // Calculate assignment probability
        //float assignmentProbability;
        //assignmentProbability = 1.0-sigmoid(dist, 2, max_track_jump_m);//1.0/abs(dist);
        // TODO investigate this parameters
        //std::cout << "row " << row << "col " << col << std::endl;
        if(negLogLike > 1000)
        	negLogLike = 1000;

        if(std::numeric_limits<double>::has_infinity ==negLogLike){
        	negLogLike = 1000;
        }
        costMatrixMAP(row,col) = (int) (negLogLike);
        // costMatrix(i,j) = -assignmentProbability*100;

        //std::cout << BOLDCYAN << "prob: " << prob << " negLogLikelihood: " << negLogLike << " matrixValue " << costMatrixMAP(row,col) << RESET << std::endl;
        col++;
      }

      row++;
    }

    for(size_t col_f = 0; col_f < nMeasurementsFake; col_f++){
        for(size_t row_f = 0; row_f < nLegsTracked; row_f++){
        	 Stamped<Point> loc = fakeDetections[col_f]->point_;

            double fakeProbCorr = 0.95;
            double prob = propagated[row_f]->getMeasurementProbability(loc) * fakeProbCorr;
            double negLogLike = -log(prob);

            std::cout << BOLDCYAN << "LT[" <<  propagated[row_f]->int_id_ << "] prob: " << prob << "negLogLike" << negLogLike << RESET << std::endl;




            // find the closest distance between candidate and trackers
            // float dist = loc.distance((*legIt)->position_);
            // TODO: Where exactly is loc to be expected? Should it be calculated based on particles?

            // Calculate assignment probability
            //float assignmentProbability;
            //assignmentProbability = 1.0-sigmoid(dist, 2, max_track_jump_m);//1.0/abs(dist);
            // TODO investigate this parameters
            if(negLogLike > 1000)
            	negLogLike = 1000;

            if(std::numeric_limits<double>::has_infinity ==negLogLike){
            	negLogLike = 1000;
            }

            costMatrixMAP(row_f,col_f  + nMeasurementsReal) = (int) (negLogLike);

           }
    }



    // Store the object indices, this is needed since the Leg Feature IDs will change with time due to creation and deletion of tracks
    Eigen::VectorXi indicesMAP = Eigen::VectorXi::Zero(nLegsTracked,1);
    int indices_counter_map = 0;
    for (vector<LegFeaturePtr>::iterator legIt = propagated.begin();
        legIt != propagated.end();
        legIt++)
    {
      indicesMAP(indices_counter_map) = (*legIt)->int_id_;
      indices_counter_map++;
    }

    std::cout << costMatrixMAP << std::endl;

    // Print the probability matrix
    std::cout << "costMatrixMAP :____" << std::endl;
    std::cout << "      ";
    for(int j = 0; j < nMeasurementsReal; j++)
      std::cout << BOLDGREEN << "LM" << std::setw(2) << j<< " |";
    std::cout << RESET;

    for(int j = 0; j < nMeasurementsFake; j++)
      std::cout << BOLDYELLOW << "LMF" << std::setw(2) << j<< " |";
    std::cout << RESET << std::endl;

    for(int i = 0; i < nLegsTracked; i++){
      std::cout << BOLDMAGENTA << "LT" << std::setw(3) <<  indicesMAP(i) << RESET <<"|";
      for(int j = 0; j < nMeasurementsReal + nMeasurementsFake; j++)
          std::cout << std::setw(5) << std::fixed << std::setprecision(3) << costMatrixMAP(i,j) << "|";

      std::cout << std::endl;
    }



    //std::cout << std::endl << "Cost Matrix:" << std::endl  << costMatrix << std::endl;
    std::vector<Solution> solutionsMAP;

    // TODO depend this on the number of measurements
    solutionsMAP = murty(costMatrixMAP,1);

    // TODO Filter the solution regarding several parameters using the leg tracker information
    // TODO Calculate the crossing value of the solutions in order to reject obvious unrealistic solutions

    std::cout << "Solutions are:" << std::endl;
    for(std::vector<Solution>::iterator solIt = solutionsMAP.begin(); solIt != solutionsMAP.end(); solIt++){
      color_print_solution(costMatrixMAP,solIt->assignmentMatrix);
      std::cout << "Costs "<< "\033[1m\033[31m" << solIt->cost_total << "\033[0m" << std::endl;
    }

    ROS_ASSERT(solutionsMAP.size() == 1);

    //////////////////////////////////////////////////////////////
    /// Update leg measurements based on the assigned measurements
    //////////////////////////////////////////////////////////////

    std::cout << "Starting Updating the legs" << std::endl;

    // Get the best solution
    Eigen::Matrix<int,-1,-1> assignmentMat;
    assignmentMat = solutionsMAP[0].assignmentMatrix;

    if(nLegsTracked > 0 && nMeasurementsReal + nMeasurementsFake > 0){

      ROS_ASSERT(nLegsTracked == assignmentMat.rows());
      ROS_ASSERT(nMeasurementsReal + nMeasurementsFake == assignmentMat.cols());

      for(int lt = 0; lt < assignmentMat.rows(); lt++){
        for(int lm = 0; lm < assignmentMat.cols(); lm++){

          // If there is an assignment
          ROS_ASSERT(lm < assignmentMat.cols());
          ROS_ASSERT(lt < assignmentMat.rows());
          if(assignmentMat(lt,lm) != 0){

            Stamped<Point> loc;
            // Get the location
            if(lm < nMeasurementsReal){

              ROS_ASSERT(lm < detections.size());
              loc = detections[lm]->point_;
              ROS_ASSERT(loc.frame_id_ == fixed_frame);
            }else{

              ROS_ASSERT(lm-nMeasurementsReal < detections.size());

              loc = detections[lm-nMeasurementsReal]->point_;
              ROS_ASSERT(loc.frame_id_ == fixed_frame);
            }

            double prob = 1.0; // TODO implement
            int idx = (int) indicesMAP(lt);

            ROS_ASSERT(idx < propagated.size());
            propagated[idx]->update(loc,prob);

          }

        }
      }

    }



    /////////////////////////////////////////////////////////////////////////
    /// Objects Creation
    /////////////////////////////////////////////////////////////////////////

    std::cout << "Starting creation of new objects" << std::endl;

    // Iterate the measurements
    for(int lm = 0; lm < nMeasurementsReal; lm++){

      // If there are no trackers
      if(nLegsTracked == 0){

        // Create track for every reliable measurement
        ROS_ASSERT(lm < nMeasurementsReal); // TODO better
        ROS_ASSERT(lm < assignmentMat.cols());
        ROS_ASSERT(lm < detections.size());

        if(detections[lm]->getProbability() > new_track_min_probability_){
          LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(new LegFeature(detections[lm]->point_, tfl_));

          // Set the occlusion model
          newLegFeature->setOcclusionModel(occlusionModel_);

          // Insert the leg feature into the propagated list
          saved_leg_features.push_back(newLegFeature);

          std::cout << BOLDRED << " -> Creating new Tracker LT" << newLegFeature->int_id_ << RESET << std::endl;

        }

      }

      // If tracks exist
      else{

        // Create track for every reliable(real!) measurement
        ROS_ASSERT(lm < nMeasurementsReal); // TODO better
        ROS_ASSERT(lm < assignmentMat.cols());
        ROS_ASSERT(lm < detections.size());
        ROS_ASSERT(assignmentMat.col(lm).sum() == 0 || assignmentMat.col(lm).sum() == 1); // Check that this is hold

        // If there is no measurement assigned to this
        if(assignmentMat.col(lm).sum() == 0  && detections[lm]->getProbability() > new_track_min_probability_){

          LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(new LegFeature(detections[lm]->point_, tfl_));

          // Set the occlusion model
          newLegFeature->setOcclusionModel(occlusionModel_);

          // Insert the leg feature into the propagated list
          saved_leg_features.push_back(newLegFeature);

          std::cout << BOLDRED << " -> Creating new Tracker LT[" << newLegFeature->int_id_ << "]" << RESET << std::endl;

        }


      }
    }


    //ROS_ASSERT(nMeasurementsFake == 0);

    //////////////////////////////////////////////////////////////////////////
    //// Joint Probability Data Association
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sJPDA [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer jpdaTimer; jpdaTimer.start();


    // Precheck for new tracks













    int i=0; // Object indice
    int j=1; // Measurement indice

    int counter = 0;

    int nObjects = propagated.size();
    int nMeasurements = detections.size() + 1; // One extra for the occlusion probability

    // Generate Matrix
    //std::cout << "There are currently " << nObjects << " objects and " << nMeasurements << " measurements(occlusion included)" << std::endl;
    Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic> costMatrix;
    costMatrix = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Zero(nObjects,nMeasurements);

    // Iterate through all detections
    for (vector<DetectionPtr>::iterator detectionIt = detections.begin();
        detectionIt != detections.end();
        detectionIt++)
    {

      i=0;

      // Iterate through the trackers
      for (vector<LegFeaturePtr>::iterator legIt = propagated.begin();
          legIt != propagated.end();
          legIt++)
      {

        Stamped<Point> loc = (*detectionIt)->point_;
        SampleSet* cluster = (*detectionIt)->cluster_;

        // find the closest distance between candidate and trackers
        float dist = loc.distance((*legIt)->position_);
        // TODO: Where exactly is loc to be expected? Should it be calculated based on particles?

        // Calculate assignment probability
        float assignmentProbability;
        assignmentProbability = 1.0-sigmoid(dist, 2, max_track_jump_m);//1.0/abs(dist);
        // TODO investigate this parameters
        //costMatrix(i,j) = min((int)-log(assignmentProbability),1000);
        costMatrix(i,j) = -assignmentProbability*100;

        i++;
      }
      //std::cout << std::endl;
      j++;
    }

    // Fill the first column of the cost matrix with dummy data for occluded objects
    int occlusionCostValue = -60; // TODO, make the dependend on something else
    for(int i = 0; i<nObjects;i++){
      costMatrix(i,0) = occlusionCostValue;
    }

    // Store the object indices, this is needed since the Leg Feature IDs will change with time due to creation and deletion of tracks
    Eigen::VectorXi indices = Eigen::VectorXi::Zero(nObjects,1);
    int indices_counter = 0;
    for (vector<LegFeaturePtr>::iterator legIt = propagated.begin();
        legIt != propagated.end();
        legIt++)
    {
      indices(indices_counter) = (*legIt)->int_id_;
      indices_counter++;
    }

    // Calculate the k-best assignments using murtys algorithm
    ////////////////////////////////////////////////////////////////////////////////////

    //std::cout << std::endl << "Cost Matrix:" << std::endl  << costMatrix << std::endl;
    std::vector<Solution> solutions;

    // TODO depend this on the number of measurements
    int k = nObjects;
    solutions = murty(costMatrix,k);

    // TODO Filter the solution regarding several parameters using the leg tracker information
    // TODO Calculate the crossing value of the solutions in order to reject obvious unrealistic solutions

    std::cout << "Solutions are:" << std::endl;
    for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
      color_print_solution(costMatrix,solIt->assignmentMatrix);
      std::cout << "Costs "<< "\033[1m\033[31m" << solIt->cost_total << "\033[0m" << std::endl;
    }

    // DEBUG OUTPUT
    std::cout << std::endl << "Cost Matrix:" << std::endl  << costMatrix << std::endl;

    // Precaculate which measurements will be required
    Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic> neededUpdateMat = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Zero(nObjects,nMeasurements);
    for(std::vector<Solution>::iterator solIt = solutions.begin(); solIt != solutions.end(); solIt++){
      neededUpdateMat += solIt->assignmentMatrix;
    }

    // Calculate the required measurement probabilities
    // Each entry represents 1/N \sum_{n=1}^N p(z_j(k)|x_{i,n}^n)
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic> probabilities = Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic>::Constant(nObjects,nMeasurements,-1);

    //std::cout << "propagated.size() " << propagated.size() << std::endl;
    //std::cout << "probabilities.rows() " << probabilities.rows() << std::endl;



    for(int j = 0; j < probabilities.cols(); j++){
      for(int i = 0; i < probabilities.rows(); i++){

        // If this calculation is needed
        if(neededUpdateMat(i,j) != 0){

          double prob = 0.0;

          // Calculate Occlusion Probability
          if(j == 0){
            // TODO Implement the calculation of the occlusion probability

            prob = propagated[i]->getOcclusionProbability(occlusionModel_); // TODO
          }

          // Calculate the measurement probability
          else
          {
            prob = propagated[i]->getMeasurementProbability(detections[j-1]->point_);
          }

          probabilities(i,j) = prob;

        }

      }
    }
    std::cout << "measurement probabilities" << std::endl << probabilities << std::endl;


    // Fill the assignment probability matrix (contains \beta_ij)
    Eigen::Matrix<double, -1,-1> assignmentProbabilityMatrix = Eigen::Matrix< double, -1, -1>::Zero(nObjects,nMeasurements);

    for(int j = 0; j < nMeasurements; j++){
      for(int i = 0; i < nObjects; i++){

        double prob = 0.0;

        // Iterate the possible assignments
        for(int l = 0; l < solutions.size(); l++){

          // Check if this assignment associates measurement j with object i
          if(solutions[l].assignmentMatrix(i,j) == 1){

            double falseMeasurementProbability = 0.01;
            prob += jpda::calculateAssociationProbability(solutions[l].assignmentMatrix,falseMeasurementProbability,probabilities);

            assignmentProbabilityMatrix(i,j) = prob;
            //TODO Remember the occlusion probabilities
          }
        }
      }
    }

    // Normalize the Assignment Probability Matrix
    Eigen::Matrix<double, -1,-1> assignmentProbabilityMatrixNormalized = Eigen::Matrix< double, -1, -1>::Zero(nObjects,nMeasurements);
    for(int i = 0; i < assignmentProbabilityMatrixNormalized.rows(); i++){

    	double rowSum = assignmentProbabilityMatrix.row(i).sum();
    	if(rowSum > 0.0){
    		assignmentProbabilityMatrixNormalized.row(i) = assignmentProbabilityMatrix.row(i)/rowSum;
    	}
    }

    // Print the probability matrix
    std::cout << "Probability Matrix:____" << std::endl;
    std::cout << "     |Occ  |";
    for(int j = 0; j < nMeasurements-1; j++)
      std::cout << BOLDGREEN << "LM" << std::setw(2) << j<< " |";
    std::cout << RESET << std::endl;

    for(int i = 0; i < nObjects; i++){
      std::cout << BOLDMAGENTA << "LT" << std::setw(3) <<  indices(i) << RESET <<"|";
      for(int j = 0; j < nMeasurements; j++)
        if(probabilities(i,j) > 0.0)
          std::cout << std::setw(5) << std::fixed << std::setprecision(3) << probabilities(i,j) << "|";
        else
          std::cout << "     |";
      std::cout << std::endl;
    }


    // Print the assignment probability matrix
    std::cout << "Assignment Probability Matrix:____" << std::endl;
    std::cout << "     |Occ  |";
    for(int j = 0; j < nMeasurements-1; j++)
    	std::cout << BOLDGREEN << "LM" << std::setw(2) << j<< " |";
    std::cout << RESET << std::endl;

    for(int i = 0; i < nObjects; i++){
    	std::cout << BOLDMAGENTA << "LT" << std::setw(3) <<  indices(i) << RESET <<"|";
    	for(int j = 0; j < nMeasurements; j++)
    		if(assignmentProbabilityMatrixNormalized(i,j) > 0.0)
    			std::cout << std::setw(5) << std::fixed << std::setprecision(3) << assignmentProbabilityMatrixNormalized(i,j) << "|";
    		else
    			std::cout << "     |";
    	std::cout << std::endl;
    }

    Eigen::Matrix<double, -1,-1> combinedMat = Eigen::Matrix< double, -1, -1>::Zero(nObjects,nMeasurements);
    combinedMat = (probabilities.array() * assignmentProbabilityMatrixNormalized.array()).matrix();


    // Print the assignment probability matrix
    std::cout << "Combined Probability Matrix:____" << std::endl;
    std::cout << "     |Occ  |";
    for(int j = 0; j < nMeasurements-1; j++)
      std::cout << BOLDGREEN << "LM" << std::setw(2) << j<< " |";
    std::cout << RESET << std::endl;

    for(int i = 0; i < nObjects; i++){
      std::cout << BOLDMAGENTA << "LT" << std::setw(3) <<  indices(i) << RESET <<"|";
      for(int j = 0; j < nMeasurements; j++)
        if(combinedMat(i,j) > 0.0)
          std::cout << std::setw(5) << std::fixed << std::setprecision(3) << combinedMat(i,j) << "|";
        else
          std::cout << "     |";
      std::cout << std::endl;
    }


    // The assignment probability matrix indicates which measurements influences which tracker to what degree.
    //std::cout << "Assignment Probabilities Matrix" << std::endl << assignmentProbabilityMatrix << std::endl;


    //std::cout << "Normalized version: " << std::endl << assignmentProbabilityMatrixNormalized << std::endl;

//    /// Invalidate Leg Trackers in necessary
//    for(int i = 0; i < nObjects; i++){
//      if(combinedMat.row(i).sum())
//      propagated[i]->setValidity(false);
//    }


    jpdaTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"DualTracker::%s - JPDA",__func__);
    ROS_DEBUG_COND(DUALTRACKER_TIME_DEBUG,"DualTracker::%s - JPDA took %f ms",__func__, jpdaTimer.getElapsedTimeMs());
    ROS_DEBUG("%sJPDA Done! [Cycle %u]", BOLDWHITE, cycle_);

    //////////////////////////////////////////////////////////////////////////
    //// JPDA Update (Use the results of the JPDA to update the leg trackers
    //////////////////////////////////////////////////////////////////////////

    bool useJPDA = false;
    bool usegreedy = false;

    if(useJPDA){

    /// Avoid updates of trackers with low probability
    // Iterate through the trackers
    std::cout << BOLDYELLOW << "Doing the JPDA Update: " << RESET << std::endl;
    for (unsigned int i = 0; i < propagated.size(); i++)
    {
      // Get the probabilities concerning this tracker
      Eigen::VectorXd probs;
      probs = assignmentProbabilityMatrixNormalized.row(i);

      if(probabilities.row(i).maxCoeff() > 0.03 && probabilities(i,0) < 0.5){
        std::cout << "LT" << propagated[i]->int_id_ << " is updated" << std::endl;
        propagated[i]->JPDAUpdate(detections, probs, occlusionModel_, scan->header.stamp);
      }
      // Avoid updates of bad leg trackers
      else
      {
        std::cout << "LT" << i << " is NOT updated because no reliable association is found" << std::endl;
      }
    }



    /// Create new trackers if needed
    // Iterate through the assignment probabilities of each measurement, create a new LT for each LM not assigned to any LT
    std::cout << "";

    unsigned int newTrackCounter = 0;

    for (unsigned int j = 1; j < nMeasurements; j++){

      // There should be at least some objects, this is not the case on the first run
      if(nObjects > 0){
        // Get the maximum likelihood
        double maxLikelihood = probabilities.col(j).maxCoeff();
        double minCreationLikelihood = 0.05;

        if(maxLikelihood < 0.01){

          std::cout << "max. Likelihood for LM" << j-1 << " is " << maxLikelihood;

          LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(new LegFeature(detections[j-1]->point_, tfl_));

          // Set the occlusion model
          newLegFeature->setOcclusionModel(occlusionModel_);

          // Insert the leg feature into the propagated list
          saved_leg_features.push_back(newLegFeature);

          // Increase the new track counter
          ++newTrackCounter;

          std::cout << BOLDRED << " -> Creating new Tracker LT" << newLegFeature->int_id_ << RESET << std::endl;

        }


      }

      // No objects are yet tracked at all
      else
      {

        //double newTrackProbability = (1 - combinedMat.row())

        std::cout << "No object found for measurement LM" << j-1 << " ... Creating new LT" << std::endl;
        if(detections[j-1]->cluster_->getProbability( ) > new_track_min_probability_){

          std::cout << "New object is to be created for measurement " << j-1 << "(prob "<< detections[j-1]->cluster_->getProbability( ) << " at " << detections[j-1]->point_[0] << "   " << detections[j-1]->point_[1] << std::endl;

          LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(new LegFeature(detections[j-1]->point_, tfl_));

          // Set the occlusion model
          newLegFeature->setOcclusionModel(occlusionModel_);

          // Insert the leg feature into the propagated list
          saved_leg_features.push_back(newLegFeature);

          // Increase the new track counter
          ++newTrackCounter;
        }
      }

    }

    std::cout << "Created " << newTrackCounter << " LTs" << std::endl;
    }
    //std::cout << "Second" << possibleAssignments(1,1) << std::endl;
    //////////////////////////////////////////////////////////////////////////
    //// Matching (Match Leg Detection to Trackers)
    //////////////////////////////////////////////////////////////////////////


    // The found matches
    multiset<MatchedFeature> matches;
    if(usegreedy){



    ROS_DEBUG("%sMatching [Cycle %u]", BOLDWHITE, cycle_);

    // Input: The propagated and new trackers, the leg detections

    // Detection step: build up the set of "candidate" clusters
    // For each candidate, find the closest tracker (within threshold) and add to the match list
    // If no tracker is found, start a new one
    // Match = cluster <-> Saved Feature (LEG)

    unsigned int newTrackCounter = 0;
    unsigned int matchesCounter = 0;



    //assert(false);

    // Iterate through all detections
    for (vector<DetectionPtr>::iterator detectionIt = detections.begin();
        detectionIt != detections.end();
        detectionIt++)
    {

      Stamped<Point> loc = (*detectionIt)->point_;
      SampleSet* cluster = (*detectionIt)->cluster_;

      // Find the closest tracker (Note that the tracker has been updated using the filter!)
      // Multiple measurements could be assigned to the same tracker! This problem is solved below. Better methods could be thought of.
      // IDEA_ Do this better! The closest is no necessarily the right one
      vector<LegFeaturePtr>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_m;

      // Iterate through the trackers
      for (vector<LegFeaturePtr>::iterator legIt = propagated.begin();
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
      // Nothing close to it, start a new track // TODO only create if the probability is high
      if (closest == propagated.end())
      {
        std::cout << "Meas. Prob." << (*detectionIt)->cluster_->probability_ << std::endl;
        std::cout << "#########################################################################################################" << std::endl;


        if(cluster->getProbability( )> new_track_min_probability_){
          loc.setZ(0); // TODO ugly fix
          LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(new LegFeature(loc, tfl_));
          newLegFeature->setOcclusionModel(occlusionModel_);
          vector<LegFeaturePtr>::iterator new_saved = saved_leg_features.insert(saved_leg_features.end(), newLegFeature);
          ++newTrackCounter;
        }

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

    }


    //////////////////////////////////////////////////////////////////////////
    //// High level Association: Combination of saved features to people tracker (takes approx. 0.8ms)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sHigh Level Association [Cycle %u]", BOLDWHITE, cycle_);
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"DualTracker::%s - Starting to combine %lu leg_tracks to people tracker",__func__, saved_leg_features.size());
    benchmarking::Timer hlAssociationTimer; hlAssociationTimer.start();

    // Do the combinations
    for (vector<LegFeaturePtr>::iterator legIt0 = saved_leg_features.begin();
        legIt0 != saved_leg_features.end();
        legIt0++)
    {
      vector<LegFeaturePtr>::iterator legIt1 = boost::next(legIt0,1);
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
    //// Update (Update the Trackers using the latest measurements (GNN)
    //////////////////////////////////////////////////////////////////////////

    if(!useJPDA){

    ROS_DEBUG("%sUpdate [Cycle %u]", BOLDWHITE, cycle_);

    // IDEA_ The next step contains one flaw, it is random which closest tracker get choosen and update the tracker, this unacceptable
    // IDEA_ Deploy Linear Association Problem Solver Here
    // Update the matched trackers
    while (matches.size() > 0)
    {
      multiset<MatchedFeature>::iterator matched_iter = matches.begin();  // Matches iterator
      bool found = false;

      // Iterate the propagated SavedFeatures(Legs)
      vector<LegFeaturePtr>::iterator pf_iter = propagated.begin(); // Tracker iterator
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

        vector<LegFeaturePtr>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;

        for (vector<LegFeaturePtr>::iterator remain_iter = propagated.begin();
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
          vector<LegFeaturePtr>::iterator new_saved = saved_leg_features.insert(saved_leg_features.end(), boost::shared_ptr<LegFeature>(new LegFeature(loc, tfl_)));

        }
        else{
          matches.insert(MatchedFeature(matched_iter->candidate_, *closest, closest_dist, matched_iter->probability_));

        }
          matches.erase(matched_iter);
      }
    }
    ROS_DEBUG("%sUpdate done! [Cycle %u]", BOLDWHITE, cycle_);
    }


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

    if(publish_leg_labels_){
      publishLegLabels(saved_leg_features, scan->header.stamp);
    }

    if(publish_predicted_leg_positions_){
      publishPredictedLegPositions(saved_leg_features, scan->header.stamp);
    }

    if(publish_scans_lines_){
      publishScanLines(*scan);
    }

    if(publish_jpda_associations_){
      publishJPDAAssociations(saved_leg_features, detections, indices, assignmentProbabilityMatrix, scan->header.stamp);
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

    publishParticlesArrows(saved_leg_features, scan->header.stamp);
    // Publish leg Measurements on
    //if(publish_leg_measurements_){
      //publishLegMeasurementArray(saved_leg_features);
    //}

    if(publish_measurement_labels_){
      publishMeasurementsLabels(detections, scan->header.stamp);
    }

    publishFakeMeasPos(people_trackers_.getList(), scan->header.stamp, sensorCoord);

    //////////////////////////////////////////////////////////////////////////
    //// Print debug information (Should happen after visual publications)
    //////////////////////////////////////////////////////////////////////////

    // Print Occlusion Probabilities
    std::cout << "Occlusion Probabilities" << std::endl;
    for(int i = 0; i < probabilities.rows(); i++)
      std::cout << "[" << indices[i] << "] " <<  probabilities(i,0) << std::endl;


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
  void publishLegMeasurementArray(std::vector<LegFeaturePtr> legFeatures){

    // Abort if List is empty
    if(legFeatures.size() == 0){
      ROS_WARN("Called publishLegMeasurementArray, but the given list is empty. Nothing to publish.");
      return;
    }

    // Iterator variable
    int i = 0;
    vector<people_msgs::PositionMeasurement> legs;

    // Iterate the features
    for (vector<LegFeaturePtr>::iterator sf_iter = legFeatures.begin();
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
   * Publish the measurements for debugging and illustration purposes
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
  void publishLegVelocities(vector<LegFeaturePtr>& legFeatures, ros::Time time){

    // Create the Visualization Message (a marker array)
    visualization_msgs::MarkerArray msgArray;

    int counter = 0;

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
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
      if((*peopleIt)->getTotalProbability() > new_track_min_probability_ ){

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
 * Publish the laserscan as lines for debugging purposes (e.g. strange clustering), or to visualize shadows
 * @param scan
 * @param time
 * @param frame
 */
void publishScanLines(const sensor_msgs::LaserScan & scan){

  visualization_msgs::Marker linesMsg;
  linesMsg.header.frame_id = scan.header.frame_id;
  linesMsg.header.stamp = scan.header.stamp;
  linesMsg.ns = "lines_ns";
  linesMsg.id = 0;
  linesMsg.type = visualization_msgs::Marker::LINE_LIST;
  linesMsg.scale.x = 0.003;
  linesMsg.scale.y = 0.01;
  linesMsg.scale.z = 0.01;
  linesMsg.color.a = 0.7; // Don't forget to set the alpha!
  linesMsg.color.r = 0.0;
  linesMsg.color.g = 1.0;
  linesMsg.color.b = 0.0;

  // Iterate the scan
  for (uint32_t i = 0; i < scan.ranges.size(); i++)
  {
    laser_processor::Sample* s = laser_processor::Sample::Extract(i, scan);
    if (s != NULL)
    {


      geometry_msgs::Point startPoint;
      startPoint.x = 0.0;
      startPoint.y = 0.0;
      startPoint.z = 0.0;

      geometry_msgs::Point endPoint;
      startPoint.x = s->x;
      startPoint.y = s->y;
      startPoint.z = 0.0;

      linesMsg.points.push_back(startPoint);
      linesMsg.points.push_back(endPoint);
    }
  }

  // Publish the clustering
  scan_lines_pub_.publish(linesMsg);
}

  /**
   * Publish the posterior particles
   * @param legFeatures List of Leg Features(Leg Tracker)
   * @param time The current time
   */
  void publishParticles(vector<LegFeaturePtr>& legFeatures, ros::Time time){
    // The pointcloud message
    sensor_msgs::PointCloud particlesPCL;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    particlesPCL.channels.push_back(rgb_channel);

    particlesPCL.header.frame_id = fixed_frame;
    particlesPCL.header.stamp = time;

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {
        MCPdf<StatePosVel>* mc = (*legFeatureIt)->filter_.getFilter()->PostGet();

        vector<WeightedSample<StatePosVel> > samples = mc->ListOfSamplesGet();


        WeightedSample<StatePosVel> maxSample = *std::max_element(samples.begin(), samples.end(), sampleWeightCompare);
        double maxSampleWeight = maxSample.WeightGet();


        for(vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
          geometry_msgs::Point32 point;
          point.x = (*sampleIt).ValueGet().pos_[0];
          point.y = (*sampleIt).ValueGet().pos_[1];
          point.z = (*sampleIt).WeightGet();//(*sampleIt).ValueGet().pos_[2];

          //
          int r,g,b;

          // If there is no sample with weight at all, make the particles blue
          if(maxSampleWeight == 0.0){
            r=0;
            g=0;
            b=255;
          }

          else
          {
            double weight = (*sampleIt).WeightGet();
            double normalizeWeight = weight / maxSampleWeight;
            redGreenGradient(normalizeWeight,r,g,b);
            //std::cout << normalizeWeight << " r:" << r << " g:" << g << " b:" << b << std::endl;
          }


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

  void publishLegFeaturesVisualization(vector<LegFeaturePtr>& legFeatures, ros::Time time){

    // The pointcloud message
    sensor_msgs::PointCloud legPcl;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    legPcl.channels.push_back(rgb_channel);

    legPcl.header.frame_id = fixed_frame;
    legPcl.header.stamp = time;

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
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

  void publishLegHistory(vector<LegFeaturePtr>& legFeatures, ros::Time time){

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
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

  void publishPredictedLegPositions(vector<LegFeaturePtr>& legFeatures, ros::Time time){

    // Marker Array
    visualization_msgs::MarkerArray markerArray;

    int counter = 0;
    for (vector<LegFeaturePtr>::iterator legIt = legFeatures.begin();
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
      markerMsgCylinder.scale.x = 0.05; // diameter x
      markerMsgCylinder.scale.y = 0.05; // diameter y
      markerMsgCylinder.scale.z = 0.3;  // height
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


  void publishMatches(vector<LegFeaturePtr>& legFeatures, ros::Time time){


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

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
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

  // Add Labels to the People Trackers
  void publishLegLabels(vector<LegFeaturePtr>& legFeatures, ros::Time time){

    // The marker Array
    visualization_msgs::MarkerArray labelArray;

    int counter = 0;
    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {

      visualization_msgs::Marker label;
      label.header.stamp = time;
      label.header.frame_id = fixed_frame;
      label.ns = "PEOPLE_TRACKER_LABEL";
      label.id = counter;
      label.type = label.TEXT_VIEW_FACING;
      label.pose.position.x = (*legFeatureIt)->getEstimate().pos_[0];
      label.pose.position.y = (*legFeatureIt)->getEstimate().pos_[1];
      label.pose.position.z = 0.4;
      label.scale.z = .1;
      label.color.a = 1;
      //label.lifetime = ros::Duration(0.5);

      // Add text
      char buf[100];
      sprintf(buf, "L%d", (*legFeatureIt)->int_id_);
      label.text = buf;

      labelArray.markers.push_back(label);

      counter++;

    }
    // Publish
    leg_label_pub_.publish(labelArray);
  }

  void publishPeopleTracker(ros::Time time){

    // Create the Visualization Message (a marker array)
    visualization_msgs::MarkerArray msgArray;

    int counter = 0;
    for(vector<PeopleTrackerPtr>::iterator peopleTrackerIt = people_trackers_.getList()->begin();
        peopleTrackerIt != people_trackers_.getList()->end();
        peopleTrackerIt++){


      if((*peopleTrackerIt)->isValid() // Tracker must be valid
         && (*peopleTrackerIt)->getTotalProbability() > 0.5 // Tracker must have certain probability
         && (publish_static_people_trackers_ || (*peopleTrackerIt)->isDynamic()) // Publish static Trackers
         ){

        std::cout << BOLDRED << "Publishing people tracker" << (**peopleTrackerIt) << RESET << std::endl;

        // The geometry message
        visualization_msgs::Marker line_list0;
        visualization_msgs::Marker line_list1;
        line_list0.type = visualization_msgs::Marker::LINE_LIST;
        line_list1.type = visualization_msgs::Marker::LINE_LIST;
        line_list0.header.frame_id = fixed_frame;
        line_list1.header.frame_id = fixed_frame;
        line_list0.header.stamp = time;
        line_list1.header.stamp = time;
        line_list0.id = counter++;
        line_list1.id = counter++;
        line_list0.ns = "people_tracker";
        line_list1.ns = "people_tracker";

        line_list0.scale.x = 0.05*(*peopleTrackerIt)->getTotalProbability();
        line_list1.scale.x = 0.05*(*peopleTrackerIt)->getTotalProbability();

        line_list0.color.r = 0.0;
        line_list0.color.g = 1.0;
        line_list0.color.b = 0.0;

        line_list1.color.r = 1.0;
        line_list1.color.g = 0.0;
        line_list1.color.b = 0.0;

        line_list0.color.a = 1.0;
        line_list1.color.a = 1.0;

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
        pointCenter.x = (*peopleTrackerIt)->getEstimate().pos_[0];
        pointCenter.y = (*peopleTrackerIt)->getEstimate().pos_[1];
        pointCenter.z = 0.0;

        // Hip 1
        pointHipRight.x = (*peopleTrackerIt)->hipPosRight_[0];
        pointHipRight.y = (*peopleTrackerIt)->hipPosRight_[1];
        pointHipRight.z = 0.0;

        // Leg 1
        pointLegRight.x = (*peopleTrackerIt)->getRightLeg()->position_[0];
        pointLegRight.y = (*peopleTrackerIt)->getRightLeg()->position_[1];
        pointLegRight.z = 0;

        if((*peopleTrackerIt)->getEstimate().vel_.length() > 0.2){

          //Line
          line_list0.points.push_back(pointLeftLeg);
          line_list0.points.push_back(pointHipLeft);

          // Line between these
          line_list0.points.push_back(pointHipLeft);
          line_list0.points.push_back(pointCenter);

          // Line between these
          line_list1.points.push_back(pointCenter);
          line_list1.points.push_back(pointHipRight);

          // Line
          line_list1.points.push_back(pointHipRight);
          line_list1.points.push_back(pointLegRight);

          // Add Visualizations for the leg estimations
          visualization_msgs::Marker leg_mov_marker;
          visualization_msgs::Marker leg_stat_marker;
          leg_mov_marker.type  = visualization_msgs::Marker::SPHERE;
          leg_stat_marker.type = visualization_msgs::Marker::SPHERE;
          leg_mov_marker.header.frame_id  = fixed_frame;
          leg_stat_marker.header.frame_id = fixed_frame;
          leg_mov_marker.header.stamp = time;
          leg_stat_marker.header.stamp = time;
          leg_mov_marker.id = 0;
          leg_stat_marker.id = 1;
          leg_mov_marker.ns = "people_leg_estimation";
          leg_stat_marker.ns = "people_leg_estimation";

          leg_mov_marker.color.r = 0.0;
          leg_mov_marker.color.g = 1.0;
          leg_mov_marker.color.b = 1.0;
          leg_mov_marker.color.a = 1.0;

          leg_stat_marker.color.r = 0.0;
          leg_stat_marker.color.g = 1.0;
          leg_stat_marker.color.b = 1.0;
          leg_stat_marker.color.a = 1.0;

          double sphereSize = 0.05;

          leg_mov_marker.scale.x = sphereSize;
          leg_mov_marker.scale.y = sphereSize;
          leg_mov_marker.scale.z = sphereSize;
          leg_stat_marker.scale.x = sphereSize;
          leg_stat_marker.scale.y = sphereSize;
          leg_stat_marker.scale.z = sphereSize;

          leg_mov_marker.pose.position.x = (*peopleTrackerIt)->leg0Prediction_.pos_[0];
          leg_mov_marker.pose.position.y = (*peopleTrackerIt)->leg0Prediction_.pos_[1];
          leg_mov_marker.pose.position.z = 0.0;

          leg_stat_marker.pose.position.x = (*peopleTrackerIt)->leg1Prediction_.pos_[0];
          leg_stat_marker.pose.position.y = (*peopleTrackerIt)->leg1Prediction_.pos_[1];
          leg_stat_marker.pose.position.z = 0.0;

          msgArray.markers.push_back(leg_mov_marker);
          msgArray.markers.push_back(leg_stat_marker);

          std::cout << "Leg0 Prediction" << (*peopleTrackerIt)->leg0Prediction_.pos_[0] << " " << (*peopleTrackerIt)->leg0Prediction_.pos_[1] << std::endl;
          std::cout << "Leg1 Prediction" << (*peopleTrackerIt)->leg1Prediction_.pos_[0] << " " << (*peopleTrackerIt)->leg1Prediction_.pos_[1] << std::endl;

        }else{
          // End line
          line_list0.color.r = 0.0;
          line_list0.color.g = 0.0;
          line_list0.color.b = 0.0;

          line_list0.points.push_back(pointLeftLeg);
          line_list0.points.push_back(pointLegRight);
        }

        // Add the lines to the msgArray
        msgArray.markers.push_back(line_list0);
        msgArray.markers.push_back(line_list1);

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

      if((*peopleTrackerIt)->getTotalProbability() > 0.5 &&
          (publish_static_people_trackers_ || (*peopleTrackerIt)->isDynamic()))
      {
      visualization_msgs::Marker label;
      label.header.stamp = time;
      label.header.frame_id = fixed_frame;
      if((*peopleTrackerIt)->isDynamic()){
        label.ns = "dynamic";
      }
      else{
        label.ns = "static";
      }

      label.id = counter;
      label.type = label.TEXT_VIEW_FACING;
      label.pose.position.x = (*peopleTrackerIt)->getEstimate().pos_[0];
      label.pose.position.y = (*peopleTrackerIt)->getEstimate().pos_[1];
      label.pose.position.z = 0.5;
      label.scale.z = .1;
      label.color.a = 1;
      label.lifetime = ros::Duration(0.5);

      // Add text
      string state;

      if((*peopleTrackerIt)->isDynamic()){
        state = "dynamic";
      }
      else{
        state = "static";
      }
      char buf[100];
      sprintf(buf, "#PT%d-%d-p%.2f(%s)", (*peopleTrackerIt)->id_[0], (*peopleTrackerIt)->id_[1], (*peopleTrackerIt)->getTotalProbability(), state.c_str());
      label.text = buf;

      labelArray.markers.push_back(label);

      counter++;
      }
    }
    // Publish
    people_track_label_pub_.publish(labelArray);
  }

  // Add Labels to the People Trackers
  void publishFakeMeasPos(boost::shared_ptr<vector<PeopleTrackerPtr> > peopleTracker, ros::Time time, tf::Stamped<tf::Point> sensorCoord){

    visualization_msgs::MarkerArray msgArray;

    int counter = 0;
    for (vector<PeopleTrackerPtr>::iterator peopleIt = peopleTracker->begin();
        peopleIt != peopleTracker->end();
        peopleIt++)
    {

      if
      (
          (*peopleIt)->getTotalProbability() > 0.25 &&
          (*peopleIt)->isDynamic()
      )
      {

        // The geometry message
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = fixed_frame;
        line_list.header.stamp = time;
        line_list.id = counter;
        line_list.ns = "FakeMeasurements";

        // width
        line_list.scale.x = 0.1;

        // Set the color

        int r,g,b;
        //r = 255;
        //getColor((*legFeatureIt)->int_id_,r,g,b);



        // Calculate the precise position
        tf::Vector3 vec;

        tf::Vector3 fake((*peopleIt)->getEstimate().pos_[0],(*peopleIt)->getEstimate().pos_[1],0);
        vec = fake - sensorCoord;

        line_list.color.r = 255.0;
        line_list.color.g = 0;
        line_list.color.b = 100;
        line_list.color.a = 1.0;

        geometry_msgs::Point point0, point1;
        point0.x = sensorCoord.getX();
        point0.y = sensorCoord.getY();
        point0.z = 0;

        point1.x = (*peopleIt)->getEstimate().pos_[0] + vec[0];
        point1.y = (*peopleIt)->getEstimate().pos_[1] + vec[1];
        point1.z = 0;

        line_list.points.push_back(point0);
        line_list.points.push_back(point1);


        counter++;

        // Publish the pointcloud
        msgArray.markers.push_back(line_list);

      }
    }

    map_pub_.publish(msgArray);
    //leg_features_history_vis_pub_.publish(line_list);


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
  void publishOcclusionModel(vector<LegFeaturePtr>& legFeatures, ros::Time time){
    // The pointcloud message
    sensor_msgs::PointCloud pcl;

    sensor_msgs::ChannelFloat32 rgb_channel;
    rgb_channel.name="rgb";
    pcl.channels.push_back(rgb_channel);

    pcl.header.frame_id = fixed_frame;
    pcl.header.stamp = time;

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
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

  /**
   * Publish the data associations using lines with variable width indicating the association probability
   * @param legFeatures  The currently tracked leg features
   * @param detections   The detections at this time
   * @param indices      The indices relating the rows of the assignmentProbabilityMatrix to the legfeatures
   * @param assignmentProbabilityMatrix The assignment probability matrix
   * @param time  The current time
   */
  void publishJPDAAssociations(vector<LegFeaturePtr>& legFeatures, vector<DetectionPtr>& detections, Eigen::VectorXi indices, Eigen::Matrix<double, -1,-1> assignmentProbabilityMatrix,  ros::Time time){


    // The marker array
    visualization_msgs::MarkerArray markerArray;



    // Get the number of rows and columns
    unsigned int nRows = assignmentProbabilityMatrix.rows();
    unsigned int nCols = assignmentProbabilityMatrix.cols();

    int counter = 0;

    // Iterate the objects(rows)
    for(int row = 0; row < nRows; row++){
      for(int col = 1; col < nCols; col++){ // Start at 1 because 0 is the occlusion prob

        double assigmentProbability = assignmentProbabilityMatrix(row,col);

        // If there is an assignment probability
        if(assigmentProbability > 0){

          //std::cout << "leg[" << indices[row] << "] measurement [" << col << "] prob: " << assigmentProbability << std::endl;

          LegFeaturePtr leg = legFeatures[row];
          DetectionPtr  detection = detections[col-1];

          // Add the points to create a line
          visualization_msgs::Marker markerMsg;

          markerMsg.header.frame_id = fixed_frame;
          markerMsg.header.stamp = time;
          markerMsg.ns = "jpda_associations";
          markerMsg.id = counter;
          markerMsg.type = visualization_msgs::Marker::LINE_LIST;
          markerMsg.scale.x = assigmentProbability/(100.0*10);
          markerMsg.color.b = 1.0;
          markerMsg.color.r = 1.0;
          markerMsg.color.a = 1.0;

          geometry_msgs::Point p0;
          p0.x = leg->position_predicted_.getX();
          p0.y = leg->position_predicted_.getY();
          p0.z = 0;

          geometry_msgs::Point p1;
          p1.x = detection->point_[0];
          p1.y = detection->point_[1];
          p1.z = 0;

          markerMsg.points.push_back(p0);
          markerMsg.points.push_back(p1);

          //std::cout << "drawing ling from " << p0.x << "   " << p0.y << "   " << p0.z << "    -->    " << p1.x << "   "  << p1.y << "   " << p1.z << std::endl;

          // Add to the marker array
          markerArray.markers.push_back(markerMsg);
          counter++;

        }

      }
    }

    jpda_association_pub_.publish(markerArray);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

  void publishMeasurementsLabels(vector<DetectionPtr>& detections, ros::Time time){

    // The marker Array
    visualization_msgs::MarkerArray labelArray;

    int counter = 0;
    for (vector<DetectionPtr>::iterator detectionsIt = detections.begin();
        detectionsIt != detections.end();
        detectionsIt++)
    {

      visualization_msgs::Marker label;
      label.header.stamp = time;
      label.header.frame_id = fixed_frame;
      label.ns = "meas_label";
      label.id = counter;
      label.type = label.TEXT_VIEW_FACING;
      label.pose.position.x = (*detectionsIt)->point_[0];
      label.pose.position.y = (*detectionsIt)->point_[1];
      label.pose.position.z = 0.3;
      label.scale.z = .1;
      label.color.b = 1;
      label.color.a = 1.0;
      //label.lifetime = ros::Duration(0.5);

      // Add text
      char buf[100];
      sprintf(buf, "#%i-%g", counter, (*detectionsIt)->cluster_->probability_);
      label.text = buf;

      labelArray.markers.push_back(label);

      counter++;

    }
    // Publish
    measurement_label_pub_.publish(labelArray);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

  void publishParticlesArrows(vector<LegFeaturePtr>& legFeatures, ros::Time time){
    // Marker Array
    visualization_msgs::MarkerArray markerArray;

    int id_counter = 0;

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {
        MCPdf<StatePosVel>* mc = (*legFeatureIt)->filter_.getFilter()->PostGet();

        vector<WeightedSample<StatePosVel> > samples = mc->ListOfSamplesGet();


        WeightedSample<StatePosVel> maxSample = *std::max_element(samples.begin(), samples.end(), sampleWeightCompare);
        double maxSampleWeight = maxSample.WeightGet();

        int counter=0;

        for(vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){

          // Not a arrow for every particle
          counter++;

          if(counter % 5 != 0) continue;

          geometry_msgs::Point point_start;
          point_start.x = (*sampleIt).ValueGet().pos_[0];
          point_start.y = (*sampleIt).ValueGet().pos_[1];
          point_start.z = (*sampleIt).WeightGet();//(*sampleIt).ValueGet().pos_[2];

          geometry_msgs::Point point_end;
          point_end.x = (*sampleIt).ValueGet().pos_[0] + (*sampleIt).ValueGet().vel_[0] * 1.0/12;
          point_end.y = (*sampleIt).ValueGet().pos_[1] + (*sampleIt).ValueGet().vel_[1] * 1.0/12;
          point_end.z = (*sampleIt).WeightGet();//(*sampleIt).ValueGet().pos_[2];


          // Arrow
          visualization_msgs::Marker markerMsgArrow;

          markerMsgArrow.header.frame_id = fixed_frame;
          markerMsgArrow.header.stamp = time;
          markerMsgArrow.ns = "arrow_pred_corr";
          markerMsgArrow.id = id_counter;
          markerMsgArrow.type = visualization_msgs::Marker::ARROW;
          markerMsgArrow.scale.x = 0.005;
          markerMsgArrow.scale.y = 0.02;
          markerMsgArrow.color.r = 1.0;
          markerMsgArrow.color.a = 0.8;

          markerMsgArrow.points.push_back(point_start);

          markerMsgArrow.points.push_back(point_end);

          markerArray.markers.push_back(markerMsgArrow);
          id_counter++;
        }

    }

    // Publish the pointcloud
    particles_arrow_pub_.publish(markerArray);

    ROS_DEBUG("DualTracker::%s Publishing Particles Arrows on %s", __func__, fixed_frame.c_str());
  }


};

int main(int argc, char **argv)
{
  // Boost generator
  typedef boost::mt19937 RNGType;
  RNGType rng;
  boost::uniform_int<> dist( 0, 9999);
  boost::variate_generator< RNGType, boost::uniform_int<> >rand_gen(rng, dist);



  // Let's imagine you need to assign N people to N jobs.  Additionally, each person will make
  // your company a certain amount of money at each job, but each person has different skills
  // so they are better at some jobs and worse at others.  You would like to find the best way
  // to assign people to these jobs.  In particular, you would like to maximize the amount of
  // money the group makes as a whole.  This is an example of an assignment problem and is
  // what is solved by the max_cost_assignment() routine.
  //
  // So in this example, let's imagine we have 3 people and 3 jobs.  We represent the amount of
  // money each person will produce at each job with a cost matrix.  Each row corresponds to a
  // person and each column corresponds to a job.  So for example, below we are saying that
  // person 0 will make $1 at job 0, $2 at job 1, and $6 at job 2.
  unsigned int N = 20;
  dlib::matrix<unsigned long> cost(N,N);



  for (unsigned int i = 0; i < N*N; i++){
    unsigned long random_value = rand_gen();
    cost(i) = random_value;


  }

  benchmarking::Timer assignmentProblemSolverTimer; assignmentProblemSolverTimer.start();

  // To find out the best assignment of people to jobs we just need to call this function.
  std::vector<long> assignment = dlib::max_cost_assignment(cost);

  assignmentProblemSolverTimer.stop();

  // Print the cost matrix
  std::cout << "COST MATRIX" << std::endl;
  for (unsigned int i = 0; i < N*N; i++){
    int row = i/N;
    int col = i % N;

    // Newline
    if(i % N == 0) std::cout << std::endl;

    // Highlight the match
    if(assignment[row] == col) std::cout << BOLDYELLOW;

    // Print the value
    std::cout << std::setw(4) << cost(i) << " " << RESET;


  }
  std::cout << std::endl;

  std::cout << "optimal cost: " << dlib::assignment_cost(cost, assignment) << std::endl;
  std::cout << "This took " << assignmentProblemSolverTimer.getElapsedTimeMs() << " ms" << std::endl;

  // EIGEN Tests
  Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic> possibleAssignments;
  possibleAssignments.resize(5,2);
  possibleAssignments = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Zero(5,2);

  possibleAssignments.col(0).setZero();


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

