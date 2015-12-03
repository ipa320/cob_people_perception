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
#include <algorithm>

// Leg Detector includes
#include <leg_detector/laser_processor.h>
#include <leg_detector/calc_leg_features.h>

// Own includes
#include <dual_people_leg_tracker/DualTrackerConfig.h>
#include <dual_people_leg_tracker/dual_tracker.h>
#include <dual_people_leg_tracker/detection/detection.h>
#include <dual_people_leg_tracker/math/math_functions.h>
#include <dual_people_leg_tracker/jpda/murty.h>
#include <dual_people_leg_tracker/jpda/jpda.h>
#include <dual_people_leg_tracker/config_struct.h>
#include <dual_people_leg_tracker/benchmarking/timer.h>
#include <dual_people_leg_tracker/leg_feature.h>
#include <dual_people_leg_tracker/people_tracker.h>
#include <dual_people_leg_tracker/models/occlusion_model.h>
#include <dual_people_leg_tracker/association/association.h>

#include <dual_people_leg_tracker/visualization/color_functions.h>
#include <dual_people_leg_tracker/visualization/color_definitions.h>
#include <dual_people_leg_tracker/visualization/visualization_conversions.h>
#include <dual_people_leg_tracker/visualization/matrix_cout_helper.h>

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

// People tracking filter
#include <people_tracking_filter/tracker_kalman.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>

// Configuration
#include <dynamic_reconfigure/server.h>

// Namespaces
using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

// Default variables
static string fixed_frame              = "odom_combined";  // The fixed frame

// Debug defines
#define DUALTRACKER_DEBUG 1         // Debug the leg detector
#define DUALTRACKER_TIME_DEBUG 1    // Debug the calculation time inside the leg_detector

// Param output
#define ROS_PARAM_OUT(param) \
    ROS_DEBUG_STREAM_COND(DUALTRACKER_DEBUG, "\t" << #param << " " << param);
    //ROS_DEBUG_COND(DUALTRACKER_DEBUG, "\t%s %d", #param, param);

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

  tf::TransformBroadcaster br_; /**< A transform broadcaster */

  ScanMask mask_; /**< A scan mask */

  OcclusionModelPtr occlusionModel_; /**< The occlusion model */

  int mask_count_;

  CvRTrees forest; /**< The forest classificator */

  float connected_thresh_; /**< Parameter for the clustering(Creation of SampleSets) */

  int feat_count_; /**< Number of features evaluated for each SampleSet */

  char save_[100];

  config_struct filter_config;

  //std::vector<PeopleTrackerPtr> people_tracker_;

  //list<SavedFeature*> saved_features_; /**< List of SavedFeatures that are currently tracked*/

  vector<LegFeaturePtr> saved_leg_features; /**< List of SavedFeatures(Legs) that are currently tracked*/

  PeopleTrackerList people_trackers_; /**< Object to handle the people_trackers */

  boost::mutex saved_mutex_; /**< Mutex to handle the access to the Saved Features */

  int feature_id_;

  // Needed for old marker removal
  int n_detections_last_cycle_;
  int n_leg_trackers_last_cycle_;
  int n_people_markers_last_published_;
  int n_people_label_markers_last_published_;
  int n_leg_tracker_last_published_;
  int n_associations_last_published_;


  //bool use_seeds_;

  // RMSE, Error stuff
  std::vector<double> error_buf;
  std::vector<double> error_buf_rmse;
  tf::StampedTransform errorTransform;
  tf::StampedTransform positionTransform;
  std::vector<ros::Time> measurePointTime;


  Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic> costMatrixMAP;
  Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic> probMAPMat;

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
  bool publish_people_velocity_kalman_; /**< Publish the kalman estimation of the people velocity (used as smoothing filter) */
  bool publish_people_lines_; /**< Publish the track history of the people track as a line */
  bool publish_people_3d_;/**< Publish 3d representations of people */
  bool publish_leg_velocity_; /**< True if the estimation of the leg features are visualized as arrows */
  bool publish_static_people_trackers_; /**< Set True if also static People Trackers(Trackers that never moved) should be displayed */
  bool publish_people_history_; /**< Publish the history of the person */
  bool publish_occlusion_model_; /**< Publish the probabilities of the particles (colorcoded) according to the occlusion model */
  bool publish_particle_arrows_; /**< Publish the particles as arrows representing the velocity */
  bool publish_leg_labels_; /**<Publish the leg labels */
  bool publish_jpda_associations_;/**< Publish the JPDA association probabilities */
  bool publish_measurement_labels_; /**< Publish labels of measurements */
  bool publish_fake_measurements_; /**< Publish a visualization of the fake measurements */
  bool publish_predicted_leg_positions_; /**< Publish the estimated position of the legs due to the prediction of the associated people tracker */
  bool publish_scans_lines_; /**< Publish laserscan as lines */

  bool publish_measurements_visualizations_; /**< Publish leg measurements visualizations */
  bool publish_measurements_visualizations_debug_; /**< Publish leg measurements visualizations (debug) */

  bool publish_leg_visualizations_; /**< Publish leg visualizations */
  bool publish_leg_visualizations_debug_; /**< Publish leg visualizations (debug) */

  bool publish_people_visualizations_; /**< Publish people visualizations */
  bool publish_people_visualizations_debug_; /**< Publish people visualizations (debug) */

  bool publish_particles_visualizations_debug_; /**< Publish particles visualization (debug) */

  int next_p_id_;

  double leg_reliability_limit_;     /** Probability for a leg detection to be considered a leg */
  double new_track_min_probability_; /**< Probability a detection needs to initialize a new leg tracker, this reduces clutter creating false tracks */
  double new_track_creation_likelihood_; /** If a measurement */

  bool use_fake_measurements_; /** True if fake leg measurements should be used */

  double people_probability_limit_; /**< Min Value for people to be considered true  */

  int min_points_per_group_;

  unsigned int cycle_; /**< Cycle counter to count the filter cycles */

  double leg_feature_predict_pos_cov_;
  double leg_feature_predict_vel_cov_;
  double leg_feature_update_cov_;
  double leg_feature_measurement_cov_;
  double initial_leg_feature_predict_pos_cov_;
  double initial_leg_feature_predict_vel_cov_;
  double min_people_probability_for_hl_prediction_;
  double static_threshold_distance_;

  double v_max_;
  double position_factor_;
  double velocity_factor_;

  benchmarking::Timer cycleTimer; /**< Timer to measure the cycle time */
  benchmarking::Timer freeTimer; /**< Timer to measure the time left for calculations */

  // The publishers
  ros::Publisher people_measurements_pub_; /**< Publisher for people measurements */
  ros::Publisher leg_measurements_pub_; /**< Publisher for leg measurements */

  ros::Publisher clusters_pub_;/**< Publisher for the clusters generated by scan processor */
  ros::Publisher particles_pub_;/**< Visualization of particles */
  ros::Publisher data_association_pub_; /**< Publishes labels of people tracks */
  ros::Publisher occlusion_model_pub_; /**< Published the occlusion probability */
  ros::Publisher scan_lines_pub_; /**< Publish the laserscan as lines for debugging */
  ros::Publisher particles_pred_pub_; /** <Publish the predicted particles */
  ros::Publisher particles_arrow_pub_; /** < Publish some particles velocity */
  ros::Publisher particles_pred_arrow_pub_; /**< Publish the predicted particles as arrows */

  // Marker Visualization Publisher
  ros::Publisher measurement_visualization_pub_; /**< Publish leg visualizations */
  ros::Publisher people_visualization_pub_;/**< Visualization of people tracks */
  ros::Publisher leg_visualization_pub_; /**< Publish measurements */
  ros::Publisher association_visualization_pub_; /**< Publish association */



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
    n_detections_last_cycle_(0),
    n_leg_trackers_last_cycle_(0),
    n_people_markers_last_published_(0),
    n_people_label_markers_last_published_(0),
    n_leg_tracker_last_published_(0),
    n_associations_last_published_(0)
    //occlusionModel_(new OcclusionModel(tfl_)),
    //new_track_creation_likelihood_(0.5)
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

    //nh_.param<bool>("use_seeds", use_seeds_, false); // TODO maybe remove later?

    // advertise topics
    leg_measurements_pub_         = nh_.advertise<people_msgs::PositionMeasurementArray>("leg_tracker_measurements", 0);
    people_measurements_pub_      = nh_.advertise<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 0);

    clusters_pub_                 = nh_.advertise<sensor_msgs::PointCloud>("clusters", 0);
    particles_pub_                = nh_.advertise<sensor_msgs::PointCloud>("particles", 0);
    occlusion_model_pub_          = nh_.advertise<sensor_msgs::PointCloud>("occlusion_model", 0);

    measurement_visualization_pub_= nh_.advertise<visualization_msgs::MarkerArray>("measurement_visualization", 0);
    leg_visualization_pub_        = nh_.advertise<visualization_msgs::MarkerArray>("leg_visualization", 0);
    people_visualization_pub_     = nh_.advertise<visualization_msgs::MarkerArray>("people_visualization", 0);
    association_visualization_pub_= nh_.advertise<visualization_msgs::MarkerArray>("association_visualization", 0);
    scan_lines_pub_               = nh_.advertise<visualization_msgs::Marker>("scan_lines", 0);
    particles_arrow_pub_          = nh_.advertise<visualization_msgs::MarkerArray>("particle_arrows", 0);
    particles_pred_pub_           = nh_.advertise<sensor_msgs::PointCloud>("particles_pred", 0);
    particles_pred_arrow_pub_     = nh_.advertise<visualization_msgs::MarkerArray>("particle_arrows_pred", 0);

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
    connected_thresh_           = config.connection_threshold;
    ROS_PARAM_OUT(connected_thresh_);

    min_points_per_group_       = config.min_points_per_group;
    ROS_PARAM_OUT(min_points_per_group_);

    new_track_min_probability_  = config.new_track_min_probability;
    ROS_PARAM_OUT(new_track_min_probability_);


    // Leg Tracker Parameters
    leg_reliability_limit_      = config.leg_reliability_limit;
    ROS_PARAM_OUT(leg_reliability_limit_);


    // People Tracker Parameters
    people_probability_limit_   = config.people_probability_limit;
    ROS_PARAM_OUT(people_probability_limit_);


    // Visualizations (Measurements)
    publish_measurements_visualizations_ = config.publish_measurements_visualizations;
    ROS_PARAM_OUT(publish_measurements_visualizations_);

    publish_measurements_visualizations_debug_ = config.publish_measurements_visualizations_debug;
    ROS_PARAM_OUT(publish_measurements_visualizations_debug_);

    // Visualizations (Legs)
    publish_leg_visualizations_ = config.publish_leg_visualizations;
    ROS_PARAM_OUT(publish_leg_visualizations_);

    publish_leg_visualizations_debug_ = config.publish_leg_visualizations_debug;
    ROS_PARAM_OUT(publish_leg_visualizations_debug_);

    // Visualizations (People)
    publish_people_visualizations_ = config.publish_people_visualizations;
    ROS_PARAM_OUT(publish_people_visualizations_);

    publish_people_visualizations_debug_ = config.publish_people_visualizations_debug;
    ROS_PARAM_OUT(publish_people_visualizations_debug_);

    // Visualizations (Particles)
    publish_particles_visualizations_debug_ = config.publish_particles_visualizations_debug;
    ROS_PARAM_OUT(publish_particles_visualizations_debug_);


    no_observation_timeout_s = config.no_observation_timeout;
    ROS_PARAM_OUT(no_observation_timeout_s);

    max_meas_jump_          = config.max_meas_jump;
    ROS_PARAM_OUT(max_meas_jump_);

    // Set probabilties of the filter
    use_fake_measurements_                            = config.use_fake_measurements;
    ROS_PARAM_OUT(use_fake_measurements_);

    /*filter_config.fakeLegProb                         = config.fake_leg_probability;

    filter_config.minFakeLegPersonProbability         = config.min_fake_leg_person_probability;
    filter_config.fakeLegRealLegDistance              = config.fake_leg_real_leg_distance;
    filter_config.fakeLegRangeThres                   = config.fake_leg_range_thres;
    filter_config.fakeLegMeasurementProbabiltyFactor  = config.fake_leg_measurement_probabilty_factor;

    filter_config.minUpdateProbability                = config.min_update_probability; // TODO make cfg editable
    */

    // Leg Feature properties
    leg_feature_update_cov_                           = config.leg_feature_update_cov; // default 0.05;
    ROS_PARAM_OUT(leg_feature_update_cov_);

    leg_feature_predict_pos_cov_                      = config.leg_feature_predict_pos_cov; // default 0.2;
    ROS_PARAM_OUT(leg_feature_predict_pos_cov_);

    leg_feature_predict_vel_cov_                      = config.leg_feature_predict_vel_cov; // default 1.2;
    ROS_PARAM_OUT(leg_feature_predict_vel_cov_);

    leg_feature_measurement_cov_                      = config.leg_feature_measurement_cov; // default 0.004;
    ROS_PARAM_OUT(leg_feature_measurement_cov_);

    initial_leg_feature_predict_pos_cov_              = config.initial_leg_feature_predict_pos_cov; // default 0.2;
    ROS_PARAM_OUT(initial_leg_feature_predict_pos_cov_);

    initial_leg_feature_predict_vel_cov_              = config.initial_leg_feature_predict_vel_cov;       // default 1.5;   // TODO make configable
    ROS_PARAM_OUT(initial_leg_feature_predict_vel_cov_);

    min_people_probability_for_hl_prediction_         = config.min_people_probability_for_hl_prediction;  // default 0.6; // TODO make configable
    ROS_PARAM_OUT(min_people_probability_for_hl_prediction_);

    static_threshold_distance_                        = config.static_threshold_distance;                 // default 0.4; // TODO make configable
    ROS_PARAM_OUT(static_threshold_distance_);

    // Filter properties
    v_max_                                            = config.v_max;                                     // default 4.0;
    ROS_PARAM_OUT(v_max_);

    position_factor_                                  = config.position_factor;                           // default 0.8;
    ROS_PARAM_OUT(position_factor_);

    velocity_factor_                                  = config.velocity_factor;                           // default 1.6;
    ROS_PARAM_OUT(velocity_factor_);

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
    //occlusionModel_->updateScan(*scan);

    //////////////////////////////////////////////////////////////////////////
    //// Create clusters (takes approx 0.8ms)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sCreating Clusters [Cycle %u]", BOLDWHITE, cycle_);

    // Start the timer
    benchmarking::Timer processTimer; processTimer.start();

    // Create the scan processor
    ScanProcessor processor(*scan, mask_);
    processor.splitConnected(connected_thresh_);
    //processor.splitConnectedRangeAware(connected_thresh_);
    processor.removeLessThan(min_points_per_group_);
    std::list<SampleSet*> clusters_temp = processor.getClusters();

    // Set ids
    int cluster_id_counter = 0;
    for (list<SampleSet*>::iterator i = clusters_temp.begin();
         i != clusters_temp.end();
         i++)
    {
      (*i)->id_ = cluster_id_counter;
      cluster_id_counter++;
    }


    ROS_DEBUG("%sCreating Clusters done! [Cycle %u] - %f ms", BOLDWHITE, cycle_, processTimer.stopAndGetTimeMs());

    //////////////////////////////////////////////////////////////////////////
    //// Remove the invalid Trackers (takes approx 0.1ms)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sRemoving old Trackers [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer removeTimer; removeTimer.start();

    // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);


    // Iterate through the saved features and remove those who havent been observed since (no_observation_timeout_s)

    //vector<LegFeaturePtr>::iterator sf_iter = saved_leg_features.begin();
    for(vector<LegFeaturePtr>::iterator sf_iter = saved_leg_features.begin();
        sf_iter != saved_leg_features.end();
        sf_iter++)
    {
      if ((*sf_iter)->getLastUpdateTime() < purge)
      {
        (*sf_iter)->setValidity(false);
      }
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
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Removed %i features not detected in the last %f seconds",__func__, features_deleted, no_observation_timeout_s);
    ROS_DEBUG("%sRemoving old Trackers done! [Cycle %u] - %f ms", BOLDWHITE, cycle_, removeTimer.getElapsedTimeMs());

    boost::shared_ptr<std::vector<PeopleTrackerPtr> > pplTrackers = people_trackers_.getList();

    //////////////////////////////////////////////////////////////////////////
    //// Update the tracker configuration/parameters for existing trackers
    //////////////////////////////////////////////////////////////////////////

    /*
    for(std::vector<PeopleTrackerPtr>::iterator pplTrackerIt = pplTrackers->begin();
        pplTrackerIt != pplTrackers->end();
        pplTrackerIt++)
    {
      (*pplTrackerIt)->configure(this->filter_config);
    }

    for (vector<LegFeaturePtr>::iterator legIt = saved_leg_features.begin();
        legIt != saved_leg_features.end();
        legIt++)
    {
      (*legIt)->configure(this->filter_config);
    }
    */


    //////////////////////////////////////////////////////////////////////////
    //// Propagation/Prediction using the motion model
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sPrediction [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer propagationTimer; propagationTimer.start();

    /// People Tracker Propagation
    benchmarking::Timer propagationPeopleTrackerTimer; propagationPeopleTrackerTimer.start();
    // High level propagation
    for(std::vector<PeopleTrackerPtr>::iterator pplTrackerIt = pplTrackers->begin();
        pplTrackerIt != pplTrackers->end();
        pplTrackerIt++)
    {
      (*pplTrackerIt)->propagate(scan->header.stamp);
    }
    propagationPeopleTrackerTimer.stop();


    // System update of trackers, and copy updated ones in propagate list
    /// Leg Tracker propagation
    benchmarking::Timer propagationLegTrackerTimer; propagationLegTrackerTimer.start();

    // Prepare propagation
    for (vector<LegFeaturePtr>::iterator legIt = saved_leg_features.begin(); legIt != saved_leg_features.end(); legIt++)
    {
      (*legIt)->preparePropagation(scan->header.stamp); // Propagate <-> Predict the filters
    }


    int nLegs = saved_leg_features.size();
    //for (vector<LegFeaturePtr>::iterator legIt = saved_leg_features.begin(); legIt != saved_leg_features.end(); legIt++)

#pragma omp parallel
{
    #pragma omp for
    for(int i = 0; i < nLegs; ++i)
    {
      saved_leg_features[i]->getId(); // works
      //saved_leg_features[i]->propagate(scan->header.stamp); // Propagate <-> Predict the filters

      //printf("Propagation of loop %i",i);
    }
}
    for(int i = 0; i < nLegs; ++i)
    {
      saved_leg_features[i]->propagate(scan->header.stamp); // Propagate <-> Predict the filters
      //printf("Propagation of loop %i",i);
    }

    vector<LegFeaturePtr> propagated;
    for (vector<LegFeaturePtr>::iterator legIt = saved_leg_features.begin(); legIt != saved_leg_features.end(); legIt++)
    {
      propagated.push_back(*legIt);
    }




    propagationLegTrackerTimer.stop();

    propagationTimer.stop();
    ROS_DEBUG_COND(DUALTRACKER_DEBUG,"LegDetector::%s - Propagated %i SavedFeatures",__func__, (int) propagated.size());

    publishParticlesPrediction(propagated, scan->header.stamp);
    //publishParticlesPredArrows(propagated, scan->header.stamp);

    ROS_DEBUG("%sPrediction done! [Cycle %u] - %f ms (%f People, %f Legs)", BOLDWHITE, cycle_, propagationTimer.getElapsedTimeMs(), propagationPeopleTrackerTimer.getElapsedTimeMs(), propagationLegTrackerTimer.getElapsedTimeMs());

    //////////////////////////////////////////////////////////////////////////
    //// Detection (Search for the existing trackers)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sDetection [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer detectionTimer; detectionTimer.start();

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
        loc.setZ(0);
        loc.frame_id_ = fixed_frame;
      }
      catch (...)
      {
        ROS_WARN("TF exception spot 3.");
      }

      if((*clusterIt)->getProbability() > leg_reliability_limit_){
        DetectionPtr detection(new Detection(detections.size(),loc,(*clusterIt)));

        // Add to the detections
        detections.push_back(detection);
        ROS_ASSERT(loc.getZ() == 0); //TODO Remove
      }

    }

    // Print the detections for debugging
    for(size_t i = 0; i < detections.size(); ++i)
      std::cout << "LM[" << detections[i]->getId() << "] - " << detections[i]->getProbability() << std::endl;


    detectionTimer.stop();
    ROS_DEBUG("%sDetection done! [Cycle %u] - %f ms", BOLDWHITE, cycle_, detectionTimer.getElapsedTimeMs());

    //////////////////////////////////////////////////////////////////////////
    //// Global Nearest Neighbour (GNN)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sGNN [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer gnnTimer; gnnTimer.start();



    int nMeasurementsReal = detections.size();
    int nLegsTracked = propagated.size();

    /// Fake measurement calculation
    vector<DetectionPtr> fakeDetections;

    if(use_fake_measurements_){
      boost::shared_ptr<std::vector<PeopleTrackerPtr> > ppls = people_trackers_.getList();

      // Iterate the people tracker
      for(std::vector<PeopleTrackerPtr>::iterator pplIt = ppls->begin(); pplIt != ppls->end(); pplIt++){

        if((*pplIt)->getTotalProbability() > filter_config.minFakeLegPersonProbability){ //TODO make probability variable

          // Number of possible measurements in range of this person
          size_t numberOfMeasurementsWithinRange = 0;

          // For every detection check if a fake measurement should be created...
          for (vector<DetectionPtr>::iterator detectionIt = detections.begin();
            detectionIt != detections.end();
            detectionIt++)
          {
            // Euclidean distance between propagated position and detection
            double dist = (*detectionIt)->getLocation().distance((*pplIt)->getEstimate().pos_);

            if(dist < filter_config.fakeLegRangeThres){
              numberOfMeasurementsWithinRange++;
            }

          }

          // If there is only one measurement within range
          if(numberOfMeasurementsWithinRange == 1){

            // Calculate a normalized vector from the sensor center to the estimated position
            tf::Vector3 pplPos((*pplIt)->getEstimate().pos_);
            tf::Vector3 widthVec = pplPos - sensorCoord;
            widthVec.normalize();

            // Increase with the distance the real and fake leg should have
            widthVec *= filter_config.fakeLegRealLegDistance;

            // Append the vector to the person position
            tf::Stamped<tf::Vector3> fakeLoc = tf::Stamped<tf::Vector3>(pplPos + widthVec, scan->header.stamp, scan->header.frame_id);

            // Create the detection
            DetectionPtr fakeDetection(new Detection(fakeDetections.size() + detections.size(), fakeLoc, filter_config.fakeLegProb));
            fakeDetections.push_back(fakeDetection);

          }
        }

      }
    }

    int nMeasurementsFake = fakeDetections.size();
    ROS_DEBUG("Fake measurements took %f ms", gnnTimer.getElapsedTimeMs());


    // costMatrixMAP is the actual costmatrix which is solved
    costMatrixMAP = Eigen::Matrix< int, Eigen::Dynamic, Eigen::Dynamic>::Zero(nLegsTracked,nMeasurementsReal + nMeasurementsFake);

    // probMAPMat represents the probability and is used for visualization
    probMAPMat = Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic>::Zero(nLegsTracked,nMeasurementsReal + nMeasurementsFake);

    // Iterate the legs
    int row = 0;
    for (vector<LegFeaturePtr>::iterator legIt = propagated.begin(); legIt != propagated.end(); legIt++)
    {

      // Iterate the detections
      int col = 0;
      for (vector<DetectionPtr>::iterator detectionIt = detections.begin(); detectionIt != detections.end(); detectionIt++)
      {

        // Get the measurement probability
        double prob = (*legIt)->getMeasurementProbability((*detectionIt)->getLocation());

        // Set the negloglikelihood (limit it to avoid infinity)
        double negLogLike = -log( max(0.000001, prob) );

        costMatrixMAP(row,col) = (int) ((negLogLike) * 100);
        probMAPMat(row,col) = prob;

        //std::cout << BOLDCYAN << "prob: " << prob << " negLogLikelihood: " << negLogLike << " matrixValue " << costMatrixMAP(row,col) << RESET << std::endl;
        col++;
      }

      row++;
    }

    //// FAKE MEASUREMENTS
    // Iterate the fake measurements
    for(size_t col_f = 0; col_f < nMeasurementsFake; col_f++){

        // Iterate the tracked legs
        for(size_t row_f = 0; row_f < nLegsTracked; row_f++){

            // Calculate the measurement probability
            double prob = propagated[row_f]->getMeasurementProbability( fakeDetections[col_f]->getLocation() ) * filter_config.fakeLegMeasurementProbabiltyFactor;

            // Set the negloglikelihood (limit it to avoid infinity)
            double negLogLike = -log( max(0.000001, prob) );

            costMatrixMAP(row_f,col_f  + nMeasurementsReal) = (int) ((negLogLike) * 100);
            probMAPMat(row_f,col_f  + nMeasurementsReal) = prob;

           }
    }

    // Store the object indices, this is needed since the Leg Feature IDs will change with time due to creation and deletion of tracks
    Eigen::VectorXi indicesVec = Eigen::VectorXi::Zero(nLegsTracked,1);
    for(size_t i = 0; i < propagated.size(); i++){ indicesVec(i) = propagated[i]->getId();}

    // Currently a vector of solutions is allowed but only the first is used
    // however using murty is possible to determine multiple associations and
    // use multiple of these for procedures such as JPDA
    std::vector<Solution> associationSets;

    // Obtain multiple solutions using murty algorithm
    //solutionsMAP = murty(costMatrixMAP,1);

    ROS_DEBUG("Preparing matrix took %f ms", gnnTimer.getElapsedTimeMs());
    associationSets.push_back(solvehungarian(costMatrixMAP));

    ROS_ASSERT(associationSets.size() == 1);

    /*CoutMatrixHelper::cout_cost_matrix("CostMatrix",
                                       costMatrixMAP,
                                       associationSets[0].assignmentMatrix,
                                       indicesVec,
                                       nLegsTracked,
                                       nMeasurementsReal,
                                       nMeasurementsFake);
    */
    CoutMatrixHelper::cout_probability_matrix("Probabilities",
                                              probMAPMat,
                                              associationSets[0].assignmentMatrix,
                                              indicesVec,
                                              nLegsTracked,
                                              nMeasurementsReal,
                                              nMeasurementsFake);


    gnnTimer.stop();
    ROS_DEBUG("%sGNN [Cycle %u] done  - %f ms", BOLDWHITE, cycle_, gnnTimer.getElapsedTimeMs());

    //////////////////////////////////////////////////////////////
    /// Update leg measurements based on the assigned measurements
    //////////////////////////////////////////////////////////////
    ROS_DEBUG("%sUpdate Trackers [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer updateTimer; updateTimer.start();

    // Get the best solution
    Eigen::Matrix<int,-1,-1> assignmentMat;

    ROS_ASSERT(associationSets.size() == 1);

    assignmentMat = associationSets[0].assignmentMatrix;

    // Reformulate into Association set
    std::vector<Association*> associationSet;
    for(int r = 0; r < assignmentMat.rows(); r++){
      // Extract the leg
      LegFeaturePtr lt = propagated[r];
      for(int c = 0; c < assignmentMat.cols(); c++){
        if(assignmentMat(r,c) ==  1){
          DetectionPtr dt = detections[c];

          associationSet.push_back(new Association(lt,dt,probMAPMat(r,c)));
        }
      }
    }

    // Print the Associations
    for(int i = 0; i < associationSet.size(); ++i){

      Association* association = associationSet[i];

      if(association->getAssociationProbability() > filter_config.minUpdateProbability && association->getDistance() < max_meas_jump_){ // TODO make variable
        std::cout << association->toString() << " doing update! (prob: " << association->getAssociationProbability() << ", dist: " << association->getDistance() << ")" << std::endl;
        //propagated[lt]->update(loc,1.0);

        association->getLeg()->update(association->getDetection()->getLocation(), 1.0);
        association->getDetection()->setUsedForUpdate(true);

      }else{
        if(association->getDistance() >= max_meas_jump_){
          std::cout << YELLOW << " NOT updating " << association->getLeg()->getIdStr() << " with " << association->getDetection()->getIdStr() << " because the distance:" << association->getDistance() << " is greate than the max_meas_jump: " << max_meas_jump_ << RESET << std::endl;
        }

        if(association->getAssociationProbability() <= filter_config.minUpdateProbability){
          std::cout << YELLOW << " NOT updating " << association->getLeg()->getIdStr() << " with " << association->getDetection()->getIdStr() << " because the assignment probability:" << association->getAssociationProbability() << " is too low(must be at least " << filter_config.minUpdateProbability << "): " << RESET << std::endl;
        }
      }
    }




    updateTimer.stop();
    ROS_DEBUG("%sUpdate Trackers [Cycle %u] done - %f ms", BOLDWHITE, cycle_, updateTimer.getElapsedTimeMs());

    /////////////////////////////////////////////////////////////////////////
    /// Tracker Creation - Create new trackers if no valid leg was found
    /////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sCreating Trackers [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer creationTimer; creationTimer.start();

    // Iterate the real measurements (no trackers for fake measurements!)
    for(int lm = 0; lm < nMeasurementsReal; lm++){

      std::stringstream status_stream;

      status_stream << "LM["<< detections[lm]->getId() << "] ";

      // If there are no trackers at all create a new tracker for every measurement
      if(nLegsTracked == 0){

        // Create track for every reliable measurement
        ROS_ASSERT(lm < nMeasurementsReal); // TODO better
        ROS_ASSERT(lm < assignmentMat.cols());
        ROS_ASSERT(lm < detections.size());


        if(detections[lm]->getProbability() > new_track_min_probability_){

          LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(
              new LegFeature(detections[lm]->getLocation(),
                             tfl_,
                             leg_feature_predict_pos_cov_,
                             leg_feature_predict_vel_cov_,
                             leg_feature_update_cov_,
                             leg_feature_measurement_cov_,
                             initial_leg_feature_predict_pos_cov_,
                             initial_leg_feature_predict_vel_cov_,
                             min_people_probability_for_hl_prediction_,
                             static_threshold_distance_,
                             v_max_,
                             position_factor_,
                             velocity_factor_
                             )
          );

          std::cout << "Created new leg feature id: " << newLegFeature->getIdStr() << std::endl;

          // Set the occlusion model // Set the occlusion model (Currently no occlusion model is used!)
          // newLegFeature->setOcclusionModel(occlusionModel_);

          // Insert the leg feature into the propagated list
          saved_leg_features.push_back(newLegFeature);

          status_stream << YELLOW << " -> Creating new Tracker LT" << newLegFeature->getId() << RESET;

        }
        // if the detection is below the new_track_min_probability
        else{
          status_stream << YELLOW << " -> Detection Probability (" << detections[lm]->getProbability() << ") to low, must be at least " << new_track_min_probability_ << RESET << std::endl;

        }

      }

      // If there are tracks create new tracks for detections with a low probability
      else
        {

        // Create track for every reliable(real!) measurement
        ROS_ASSERT(lm < nMeasurementsReal); // TODO better
        ROS_ASSERT(lm < assignmentMat.cols());
        ROS_ASSERT(lm < detections.size());
        ROS_ASSERT(assignmentMat.col(lm).sum() == 0 || assignmentMat.col(lm).sum() == 1); // Check that this is hold

        double colSum = assignmentMat.col(lm).sum();
        double detectionProb = detections[lm]->getProbability();

        // Extract the probability of the assignment, if to low a new tracker will be created
        double assignmentProb = 0;
        for(size_t i = 0; i < probMAPMat.rows(); i++){
          if(assignmentMat(i,lm) == 1)
            assignmentProb = probMAPMat(i,lm);
        }
        //double probSum = probMAPMat.col(lm).sum()/ probMAPMat.rows();

        double maxAssignmentProbForNewTracker = 0.003;


        bool usedBeforeCondition = detections[lm]->usedForUpdate();
        bool assignmentProbCondition = assignmentProb < maxAssignmentProbForNewTracker; // TODO make variable
        bool newTrackMinProbCondition = detectionProb > new_track_min_probability_;


        // If no track is assigned to this measurement (or only a unreliable one)
        if(!usedBeforeCondition && newTrackMinProbCondition && assignmentProbCondition){

          // Check the distance to the next measurement
          double dist_min = 1000;
          for(size_t i = 0; i < nMeasurementsReal; i++){
            if(i != lm){
              double dist = (detections[i]->getLocation() - detections[lm]->getLocation()).length();
              //status_stream << "Dist LM[" << i << "] <-> LM[" << lm << "]" << dist << std::endl;

              if(dist < dist_min){
                dist_min = dist;
              }
            }
          }

          bool distCondition = dist_min > 0.2;

          // Create only if the distance to between two detections is below a certain threshold
          if(distCondition)
          { // TODO make this variable
            LegFeaturePtr newLegFeature = boost::shared_ptr<LegFeature>(
                new LegFeature(detections[lm]->getLocation(),
                               tfl_,
                               leg_feature_predict_pos_cov_,
                               leg_feature_predict_vel_cov_,
                               leg_feature_update_cov_,
                               leg_feature_measurement_cov_,
                               initial_leg_feature_predict_pos_cov_,
                               initial_leg_feature_predict_vel_cov_,
                               min_people_probability_for_hl_prediction_,
                               static_threshold_distance_,
                               v_max_,
                               position_factor_,
                               velocity_factor_)
            );


            // Set the occlusion model (Currently no occlusion model is used!)
            // newLegFeature->setOcclusionModel(occlusionModel_);

            // Insert the leg feature into the propagated list
            saved_leg_features.push_back(newLegFeature);

            status_stream << YELLOW << " -> Creating new Tracker LT[" << newLegFeature->getId() << "]" << RESET;
          }
          //if(dist_min > 0.2)
          else
          {
            status_stream << YELLOW << " -> no Tracker because distance criteria dist_min: (" << dist_min << ")" << RESET;
          }

        }

        // If no tracker was created for a measurement notify at least why
        else
        {
          if(!assignmentProbCondition)
            status_stream << " -> assignment probability was to high (" << assignmentProb << ", max: " << maxAssignmentProbForNewTracker << ")";

          if(!newTrackMinProbCondition)
            status_stream << " -> no tracker its detection probability " << detectionProb << " is to low( must be at least " << new_track_min_probability_;
        }


      }

      std::cout << status_stream.str() << std::endl;
    }



    creationTimer.stop();
    ROS_DEBUG("%sCreating Trackers [Cycle %u] done - %f ms", BOLDWHITE, cycle_, creationTimer.getElapsedTimeMs());

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
      for (;legIt1 != saved_leg_features.end();legIt1++)
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
    ROS_DEBUG("%sHigh Level Association [Cycle %u] done -  %f ms", BOLDWHITE, cycle_, hlAssociationTimer.getElapsedTimeMs());

    //////////////////////////////////////////////////////////////////////////
    //// High level Update (Update of the people trackers)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sHigh level update [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer hlUpdateTimer;
    hlUpdateTimer.start();

    // Update the probabilites of every people tracker
    people_trackers_.updateAllTrackers(scan->header.stamp);

    hlUpdateTimer.stop();
    ROS_DEBUG("%sHigh level update [Cycle %u] done - %f ms", BOLDWHITE, cycle_, hlUpdateTimer.getElapsedTimeMs());

    //////////////////////////////////////////////////////////////////////////
    //// Publish data
    //////////////////////////////////////////////////////////////////////////

    ROS_DEBUG("%sPublishing [Cycle %u]", BOLDWHITE, cycle_);
    benchmarking::Timer publishTimer; publishTimer.start();

    // Publish the leg measurements
    if(publish_measurements_visualizations_){
      //publishLegMeasurements(processor.getClusters(), scan->header.stamp, scan->header.frame_id);
    }

    //// Measurement related publication
    //publishScanLines(*scan); (Not used anymore)


    // Publish the clustering
    if(publish_measurements_visualizations_){
      publishClusters(processor.getClusters(), scan->header.stamp, scan->header.frame_id);
    }

    if(publish_measurements_visualizations_){
      publishMeasurementsVisualization(detections, scan->header.stamp);
    }

    //// Leg related publication
    // Publish the detections of legs
    if(publish_leg_visualizations_debug_){
      publishLegVelocities(people_trackers_.getList(), scan->header.stamp);
    }

    // Publish the history of each leg
    if(publish_leg_visualizations_debug_){
      publishLegHistory(saved_leg_features, scan->header.stamp);
    }

   // if(publish_matches_){
   //   publishMatches(saved_leg_features, scan->header.stamp);
   // }

    if(publish_leg_visualizations_){
      publishLegLabels(saved_leg_features, scan->header.stamp);
    }

    if(publish_leg_visualizations_){
      publishLegTracker(saved_leg_features, scan->header.stamp);
    }

    //// People related publication
    if(publish_people_tracker_){
      publishPeopleTracker(scan->header.stamp);
      publishPeopleVelocity(people_trackers_.getList(), scan->header.stamp);
      publishPeopleLabels(scan->header.stamp);
    }

    if(publish_people_visualizations_){
      publishPeopleVelocityKalman(people_trackers_.getList(), scan->header.stamp);
    }

    if(publish_people_visualizations_){
      publishPeople3d(scan->header.stamp);
    }

    if(publish_people_visualizations_){
      publishPeopleHistory(people_trackers_.getList(), scan->header.stamp);
    }

    //// Association related publication
    publishDataAssociationVisualization(associationSet, scan->header.stamp);

    //// Particle related publication
    if(publish_particles_visualizations_debug_){
      publishParticlesArrows(saved_leg_features, scan->header.stamp);
    }

    if(publish_particles_visualizations_debug_){
      publishParticles(saved_leg_features, scan->header.stamp);
    }

    if(publish_fake_measurements_){
      publishFakeMeasPos(fakeDetections, scan->header.stamp, sensorCoord);
    }

    // Print all the people trackers
    people_trackers_.printTrackerList();




    publishTimer.stop();
    ROS_DEBUG("%sPublishing [Cycle %u] done - %f ms", BOLDWHITE, cycle_, publishTimer.getElapsedTimeMs());

    //////////////////////////////////////////////////////////////////////////
    //// RMSE (for evaluation purposes)
    //////////////////////////////////////////////////////////////////////////
    /*
    tf::Transform correctionTransform;

    std::string optiTrackName = "optitrack_robot_station1";
    //std::string optiTrackNameCorrection = "optitrack_robot_station1_corr";

    tf::Quaternion q;
    q.setRPY(0,0,0);
    correctionTransform.setOrigin( tf::Vector3(-0.15, 0.0, 0.0) );
    correctionTransform.setRotation(q);
    //br.sendTransform(tf::StampedTransform(transform, scan->header.stamp, optiTrackName, optiTrackNameCorrection));

    try {
      tfl_.lookupTransform("odom_combined", optiTrackName, ros::Time(0), errorTransform);
      tfl_.lookupTransform("odom_combined", "ppl1_3", ros::Time(0), positionTransform);

      if(abs((errorTransform.stamp_ - positionTransform.stamp_).toSec()) < 0.1){

        std::cout << "errorTransform Time:" << errorTransform.stamp_ << std::endl;
        std::cout << "positionTransform Time:" << positionTransform.stamp_ << std::endl;
        std::cout << "Delta: " << (errorTransform.stamp_ - positionTransform.stamp_).toSec() << std::endl;

        std::cout << "positionTransform " << positionTransform.getOrigin().getX() << " " << positionTransform.getOrigin().getY() << " time:" << positionTransform.stamp_ << std::endl;
        std::cout << "errorTransform " << errorTransform.getOrigin().getX() << " " << errorTransform.getOrigin().getY() << " time:" << errorTransform.stamp_ << std::endl;

        std::cout << BOLDBLUE << "distance " << ((errorTransform*correctionTransform).getOrigin() - positionTransform.getOrigin()).length() << std::endl;
        double distance = ((errorTransform*correctionTransform).getOrigin() - positionTransform.getOrigin()).length();


        error_buf.push_back(distance);
        measurePointTime.push_back(scan->header.stamp);



        // Calculate the rmse
        double sum = 0;
        for(size_t i = 0; i < error_buf.size(); i++){
          sum += pow(error_buf[i],2);
        }

        double rmse =  sqrt(sum/error_buf.size());

        error_buf_rmse.push_back(rmse);

        // Python output the rmse
        std::cout << "error = [";
        for(size_t i = 0; i < error_buf.size(); i++){
          std::cout << error_buf[i] << ",";
        }
        std::cout << ']' << RESET << std::endl;

        // Python output the rmse
        std::cout << "rmse = [";
        for(size_t i = 0; i < error_buf_rmse.size(); i++){
          std::cout << error_buf_rmse[i] << ",";
        }
        std::cout << ']' << RESET << std::endl;

        // Python output the time
        std::cout << "time = [";
        for(size_t i = 0; i < measurePointTime.size(); i++){
          std::cout << measurePointTime[i] << ",";
        }
        std::cout << ']' << RESET << std::endl;

      }

    }
    catch (tf::TransformException ex){
      ROS_WARN("NO TRANSFORMATION FOUND");
    }*/


    //////////////////////////////////////////////////////////////////////////
    //// Social interaction (Early alpha!!!)
    //////////////////////////////////////////////////////////////////////////
    ROS_DEBUG("%sSocial interaction [Cycle %u]", BOLDWHITE, cycle_);

    if(false){
      double predictionTimeInterval = 0.2; // TODO make this variable
      double predictionSteps = 15;  // TODO make this variable
      people_trackers_.calculateTheNextDesiredVelocities(predictionTimeInterval, predictionSteps);

      publishEstimateNextVelocity(people_trackers_.getList(), scan->header.stamp, predictionTimeInterval);
    }
    ROS_DEBUG("%sSocial interaction [Cycle %u] done", BOLDWHITE, cycle_);
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

      if ((*sf_iter)->getReliability() > leg_reliability_limit_ && publish_measurements_visualizations_)
      {
        people_msgs::PositionMeasurement pos;
        pos.header.stamp = legFeatures.front()->getLastUpdateTime();
        pos.header.frame_id = legFeatures.front()->getFixedFrame();
        pos.name = "leg_detector";
        pos.object_id = (*sf_iter)->getIdStr();
        pos.pos.x = (*sf_iter)->getPosition()[0];
        pos.pos.y = (*sf_iter)->getPosition()[1];
        pos.pos.z = (*sf_iter)->getPosition()[2];
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
  array.header.stamp =  saved_leg_features.front()->getLastScanTime();
  array.header.frame_id = saved_leg_features.front()->getFixedFrame();

  // Publish
  array.people = legs;
  leg_measurements_pub_.publish(array);
  }



  /**
   * Publish the measurements for debugging and illustration purposes
   * @param set
   * @param frame // Frame the clusters are in
   */
  /*
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
*/


  /**
   * Publish visualization of the detections/measurements
   * @param detections
   * @param time
   */
  void publishMeasurementsVisualization(vector<DetectionPtr>& detections, ros::Time time){

    // The marker Array, collecting Label and Marker
    visualization_msgs::MarkerArray markerArray;

    // Parameters
    double height = 0.3; // User for Cyclinder height and Label position
    double label_z_offset = 0.05; // Distance label to cylinder

    int counter = 0;
    // Iterate the detections
    for (vector<DetectionPtr>::iterator detectionsIt = detections.begin();
        detectionsIt != detections.end();
        detectionsIt++)
    {

      visualization_msgs::Marker leg_measurement_label_marker;
      leg_measurement_label_marker.header.stamp = time;
      leg_measurement_label_marker.header.frame_id = fixed_frame;
      leg_measurement_label_marker.ns = "meas_labels";
      leg_measurement_label_marker.id = counter;
      leg_measurement_label_marker.type = leg_measurement_label_marker.TEXT_VIEW_FACING;
      leg_measurement_label_marker.pose.position.x = (*detectionsIt)->getLocation()[0];
      leg_measurement_label_marker.pose.position.y = (*detectionsIt)->getLocation()[1];
      leg_measurement_label_marker.pose.position.z = height + label_z_offset;

      // Choose the color
      int r,g,b;
      getCycledColor((*detectionsIt)->getCluster()->id_,r,g,b);

      leg_measurement_label_marker.scale.x = .1;
      leg_measurement_label_marker.scale.y = .1;
      leg_measurement_label_marker.scale.z = .1;
      leg_measurement_label_marker.color.r = r / 255.0;
      leg_measurement_label_marker.color.g = g / 255.0;
      leg_measurement_label_marker.color.b = b / 255.0;
      leg_measurement_label_marker.color.a = 1.0;

      // Add text
      char buf[100];
      sprintf(buf, "#%i-%g", counter, (*detectionsIt)->getProbability());
      leg_measurement_label_marker.text = buf;

      markerArray.markers.push_back(leg_measurement_label_marker);

      // Add a marker
      visualization_msgs::Marker leg_measurement_maker;

      leg_measurement_maker.header.stamp = time;
      leg_measurement_maker.header.frame_id = fixed_frame;
      leg_measurement_maker.ns = "meas_position";
      leg_measurement_maker.id = counter;
      leg_measurement_maker.type = leg_measurement_label_marker.CYLINDER;
      leg_measurement_maker.pose.position.x = (*detectionsIt)->getLocation()[0];
      leg_measurement_maker.pose.position.y = (*detectionsIt)->getLocation()[1];
      leg_measurement_maker.pose.position.z = height/2;
      leg_measurement_maker.scale.x = .1;
      leg_measurement_maker.scale.y = .1;
      leg_measurement_maker.scale.z = height;
      leg_measurement_maker.color.r = r / 255.0;
      leg_measurement_maker.color.g = g / 255.0;
      leg_measurement_maker.color.b = b / 255.0;
      leg_measurement_maker.color.a = 1.0;

      markerArray.markers.push_back(leg_measurement_maker);

      counter++;

    }

    // Delete old labels and markers
    for(int i = 0; i < n_detections_last_cycle_ - ((int)detections.size()); ++i){
      visualization_msgs::Marker delete_label;
      delete_label.header.stamp = time;
      delete_label.header.frame_id = fixed_frame;
      delete_label.id = counter;
      delete_label.scale.x = 1;
      delete_label.scale.y = 1;
      delete_label.scale.z = 1;
      delete_label.ns = "meas_labels";
      delete_label.type = delete_label.DELETE;

      markerArray.markers.push_back(delete_label);

      visualization_msgs::Marker delete_marker;
      delete_marker.header.stamp = time;
      delete_marker.header.frame_id = fixed_frame;
      delete_marker.scale.x = 1;
      delete_marker.scale.y = 1;
      delete_marker.scale.z = 1;
      delete_marker.id = counter;
      delete_marker.ns = "meas_position";
      delete_marker.type = delete_label.DELETE;

      markerArray.markers.push_back(delete_marker);


      counter++;
    }

    n_detections_last_cycle_ = detections.size();

    // Publish
    measurement_visualization_pub_.publish(markerArray);

    ROS_DEBUG("DualTracker::%s Publishing Clusters on %s", __func__, fixed_frame.c_str());
  }

  /**
   * Publish
   * @param legFeatures
   * @param time
   */
  void publishLegVelocities(boost::shared_ptr<vector<PeopleTrackerPtr> > peopleTracker, ros::Time time){

	if(peopleTracker->size() == 0)
		return;

    // Create the Visualization Message (a marker array)
    visualization_msgs::MarkerArray msgArray;

    int counter = 0;

    for (vector<PeopleTrackerPtr>::iterator peopleIt = peopleTracker->begin();
        peopleIt != peopleTracker->end();
        peopleIt++)
    {


      if((*peopleIt)->getTotalProbability() > 0.6 && (*peopleIt)->isDynamic()){


      LegFeaturePtr movingLeg = (*peopleIt)->getMovingLeg();
      LegFeaturePtr standingLeg = (*peopleIt)->getStandingLeg();



      BFL::StatePosVel estMov = movingLeg->getEstimate();
      BFL::StatePosVel estStat = standingLeg->getEstimate();

      //std::cout << "estMov last Step width: " << movingLeg->getLastStepWidth() << std::endl;
      //std::cout << "estStat last Step width: " << standingLeg->getLastStepWidth() << std::endl;
      //std::cout << "estMov: " << estMov << std::endl;
      //std::cout << "estStat: " << estStat << std::endl;

      ROS_ASSERT((movingLeg->getId() != standingLeg->getId()));

      visualization_msgs::Marker markerMoving;
      markerMoving.header.frame_id = fixed_frame;
      markerMoving.header.stamp = time;
      markerMoving.ns = "leg_feature_arrows";
      markerMoving.id = counter;
      markerMoving.type = visualization_msgs::Marker::ARROW;
      double factor = 0.7; // Control the arrow length

      geometry_msgs::Point startPoint;
      startPoint.x = estMov.pos_[0];
      startPoint.y = estMov.pos_[1];
      startPoint.z = estMov.pos_[2];

      geometry_msgs::Point endPoint;
      endPoint.x = estMov.pos_[0] + estMov.vel_[0]*factor;
      endPoint.y = estMov.pos_[1] + estMov.vel_[1]*factor;
      endPoint.z = estMov.pos_[2] + estMov.vel_[2]*factor;

      markerMoving.points.push_back(startPoint);
      markerMoving.points.push_back(endPoint);


      markerMoving.scale.x = 0.05; //shaft diameter
      markerMoving.scale.y = 0.1; //head diameter
      markerMoving.scale.z = 0; // head length (if other than zero)
      markerMoving.color.a = 1.0; // Don't forget to set the alpha!
      markerMoving.color.r = 1.0;
      markerMoving.color.g = 0.0;
      markerMoving.color.b = 0.0;



      counter++;



      visualization_msgs::Marker markerStanding;
      markerStanding.header.frame_id = fixed_frame;
      markerStanding.header.stamp = time;
      markerStanding.ns = "leg_feature_arrows";
      markerStanding.id = counter;
      markerStanding.type = visualization_msgs::Marker::ARROW;


      geometry_msgs::Point startPointStat;
      startPoint.x = estStat.pos_[0];
      startPoint.y = estStat.pos_[1];
      startPoint.z = 0;





      geometry_msgs::Point endPointStat;
      endPoint.x = estStat.pos_[0] + estStat.vel_[0];
      endPoint.y = estStat.pos_[1] + estStat.vel_[1];
      endPoint.z = 0;

      //std::cout << "Arrow from " <<  std::endl << startPoint << " to " << std::endl << endPoint << std::endl;

      markerStanding.points.push_back(startPointStat);
      markerStanding.points.push_back(endPointStat);

      markerStanding.scale.x = 3; //shaft diameter
      markerStanding.scale.y = 3; //head diameter
      markerStanding.scale.z = 5; // head length (if other than zero)
      markerStanding.color.a = 1.0; // Don't forget to set the alpha!
      markerStanding.color.r = 0.0;
      markerStanding.color.g = 1.0;
      markerStanding.color.b = 0.0;

      msgArray.markers.push_back(markerMoving);
      //msgArray.markers.push_back(markerStanding);

      counter++;

      }
    }
    leg_visualization_pub_.publish(msgArray);

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
        //marker.lifetime = ros::Duration(0.1);

        double factor = 2; // Control the arrow length

        geometry_msgs::Point startPoint;
        startPoint.x = est.pos_[0] + est.vel_[0] * 0.5;
        startPoint.y = est.pos_[1] + est.vel_[1] * 0.5;
        startPoint.z = est.pos_[2] + est.vel_[2] * 0.5;



        geometry_msgs::Point endPoint;
        endPoint.x = est.pos_[0] + est.vel_[0]*factor;
        endPoint.y = est.pos_[1] + est.vel_[1]*factor;
        endPoint.z = est.pos_[2] + est.vel_[2]*factor;

        marker.points.push_back(startPoint);
        marker.points.push_back(endPoint);

        marker.scale.x = 0.03; //shaft diameter
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
    people_visualization_pub_.publish(msgArray);

  }

  void publishPeopleVelocityKalman(boost::shared_ptr<vector<PeopleTrackerPtr> > peopleTracker, ros::Time time){

    // Create the Visualization Message (a marker array)
    visualization_msgs::MarkerArray msgArray;

    int counter = 0;

    for (vector<PeopleTrackerPtr>::iterator peopleIt = peopleTracker->begin();
        peopleIt != peopleTracker->end();
        peopleIt++)
    {
      if((*peopleIt)->getTotalProbability() > 0.6 ){

        BFL::StatePosVel est = (*peopleIt)->getEstimateKalman();

        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame;
        marker.header.stamp = time;
        marker.ns = "kalman_estimation";
        marker.id = counter;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.1);

        double factor = 0.5; // Control the arrow length

        geometry_msgs::Point startPoint;
        startPoint.x = est.pos_[0];
        startPoint.y = est.pos_[1];
        startPoint.z = 0.0;



        geometry_msgs::Point endPoint;
        endPoint.x = est.pos_[0] + est.vel_[0]*factor;
        endPoint.y = est.pos_[1] + est.vel_[1]*factor;
        endPoint.z = 0.0;

        marker.points.push_back(startPoint);
        marker.points.push_back(endPoint);

        marker.scale.x = 0.1; //shaft diameter
        marker.scale.y = 0.1; //head diameter
        marker.scale.z = 0; // head length (if other than zero)
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        msgArray.markers.push_back(marker);

        counter++;
      }
    }
    people_visualization_pub_.publish(msgArray);

  }

  void publishEstimateNextVelocity(boost::shared_ptr<vector<PeopleTrackerPtr> > peopleTracker, ros::Time time, double predictionTimeInterval){

      // Create the Visualization Message (a marker array)
      visualization_msgs::MarkerArray msgArray;

      int counter = 0;
      int counter_goal = 0;

      for (vector<PeopleTrackerPtr>::iterator peopleIt = peopleTracker->begin();
          peopleIt != peopleTracker->end();
          peopleIt++)
      {

        if((*peopleIt)->getTotalProbability() > 0.8 && (*peopleIt)->getEstimate().vel_.length() > 0.3){

          //std::cout << YELLOW << "######### " << (*peopleIt)->getName() << RESET << std::endl;

          // Plot the goal
          visualization_msgs::Marker markerGoal;
          markerGoal.header.frame_id = fixed_frame;
          markerGoal.header.stamp = time;
          markerGoal.ns = "goals";
          markerGoal.id = ++counter_goal;
          markerGoal.type = visualization_msgs::Marker::CYLINDER;
          markerGoal.action = visualization_msgs::Marker::ADD;
          markerGoal.pose.position.x = (*peopleIt)->getGoal()[0];
          markerGoal.pose.position.y = (*peopleIt)->getGoal()[1];
          markerGoal.pose.position.z = 0.25;
          markerGoal.scale.x = 0.02; //shaft diameter
          markerGoal.scale.y = 0.02; //head diameter
          markerGoal.scale.z = 0.5; // head length (if other than zero)
          markerGoal.color.a = 1.0;
          markerGoal.color.r = 0.0;
          markerGoal.color.g = 1.0;
          markerGoal.color.b = 0.0;

          msgArray.markers.push_back(markerGoal);

          // Add goal label
          visualization_msgs::Marker label;
          label.header.stamp = time;
          label.header.frame_id = fixed_frame;
          label.ns = "goals_labels";
          label.id = counter;
          label.type = label.TEXT_VIEW_FACING;
          label.pose.position.x = (*peopleIt)->getGoal()[0];
          label.pose.position.y = (*peopleIt)->getGoal()[1];
          label.pose.position.z = 0.55;
          label.scale.z = .1;
          label.color.a = 1;
          label.color.r = 0;
          label.color.g = 1.0;
          label.color.b = 0;
          // Add text
          char buf[100];
          sprintf(buf, "Goal");
          label.text = buf;

          msgArray.markers.push_back(label);

          // Plot line to goal
          visualization_msgs::Marker arrowMarker;
          arrowMarker.header.frame_id = fixed_frame;
          arrowMarker.header.stamp = time;
          arrowMarker.ns = "goal_arrow";
          arrowMarker.id = counter;
          arrowMarker.type = visualization_msgs::Marker::ARROW;
          arrowMarker.action = visualization_msgs::Marker::ADD;
          //marker.lifetime = ros::Duration(0.1);

          geometry_msgs::Point startPoint;
          startPoint.x = (*peopleIt)->getEstimate().pos_[0];
          startPoint.y = (*peopleIt)->getEstimate().pos_[1];
          startPoint.z = 0.0;

          geometry_msgs::Point endPoint;
          endPoint.x = (*peopleIt)->getGoal()[0];
          endPoint.y = (*peopleIt)->getGoal()[1];
          endPoint.z = 0.0;

          arrowMarker.points.push_back(startPoint);
          arrowMarker.points.push_back(endPoint);

          arrowMarker.scale.x = 0.03; //shaft diameter
          arrowMarker.scale.y = 0.03; //head diameter
          arrowMarker.scale.z = 0; // head length (if other than zero)
          arrowMarker.color.a = 0.6; // Don't forget to set the alpha!
          arrowMarker.color.r = 0.0;
          arrowMarker.color.g = 1.0;
          arrowMarker.color.b = 0.0;

          msgArray.markers.push_back(arrowMarker);


          // Iterate the number of predictions and draw Arrows
          for(size_t predN = 0; predN < (*peopleIt)->getNumberOfPredictions()-1; predN++){

            // Get the prediction
            BFL::StatePosVel est = (*peopleIt)->getNextDesiredPosVel(predN);
            BFL::StatePosVel estNext = (*peopleIt)->getNextDesiredPosVel(predN+1);
            std::cout << "[step " << predN << "] " << est << std::endl;

            visualization_msgs::Marker marker;
            marker.header.frame_id = fixed_frame;
            marker.header.stamp = time;
            marker.ns = "next_desired_velocity";
            marker.id = counter;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            //marker.lifetime = ros::Duration(0.1);

            double factor = 1; // Control the arrow length

            geometry_msgs::Point startPoint;
            startPoint.x = est.pos_[0];
            startPoint.y = est.pos_[1];
            startPoint.z = 0.0;

            geometry_msgs::Point endPoint;
            endPoint.x = estNext.pos_[0];// + est.vel_[0]*factor;
            endPoint.y = estNext.pos_[1];// + est.vel_[1]*factor;
            endPoint.z = 0.0;

            marker.points.push_back(startPoint);
            marker.points.push_back(endPoint);

            // Calculate the alpha value
            double alpha_min = 0.2;
            double range = 1 - alpha_min;


            marker.scale.x = 0.03; //shaft diameter
            marker.scale.y = 0.1; //head diameter
            marker.scale.z = 0; // head length (if other than zero)
            marker.color.a = alpha_min + ((double)predN)/(*peopleIt)->getNumberOfPredictions() * range; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            //std::cout << "alpha" << marker.color.a << std::endl;

            msgArray.markers.push_back(marker);

            counter++;
          }

        }
      }
      people_visualization_pub_.publish(msgArray);

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
            int r,g,b;

            getCycledColor((*i)->id_, r, g, b);
            count++;

            (*i)->appendToCloud(clusterPCL,r,g,b);
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
        //std::cout << "Particles of LT[" << (*legFeatureIt)->int_id_ << "]" << std::endl;

        MCPdf<StatePosVel>* mc = (*legFeatureIt)->postGet();

        vector<WeightedSample<StatePosVel> > samples = mc->ListOfSamplesGet();


        WeightedSample<StatePosVel> maxSample = *std::max_element(samples.begin(), samples.end(), sampleWeightCompare);
        double maxSampleWeight = maxSample.WeightGet();

        //std::cout << "NSamples:" << samples.size() << " maxSampleWeight:" << maxSampleWeight << "------" << std::endl;

        for(vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
          geometry_msgs::Point32 point;
          point.x = (*sampleIt).ValueGet().pos_[0];
          point.y = (*sampleIt).ValueGet().pos_[1];
          point.z = 0;//(*sampleIt).WeightGet();//(*sampleIt).ValueGet().pos_[2];

          //
          int r,g,b;
          double weight;
          // If there is no sample with weight at all, make the particles blue
          if(maxSampleWeight == 0.0){
            r=0;
            g=0;
            b=255;
          }

          else
          {
            weight = (*sampleIt).WeightGet();
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


  /**
   * Publish the posterior particles
   * @param legFeatures List of Leg Features(Leg Tracker)
   * @param time The current time
   */
  void publishParticlesPrediction(vector<LegFeaturePtr>& legFeatures, ros::Time time){
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
        //std::cout << "Particles of LT[" << (*legFeatureIt)->int_id_ << "]" << std::endl;

        MCPdf<StatePosVel>* mc = (*legFeatureIt)->postGet();

        vector<WeightedSample<StatePosVel> > samples = mc->ListOfSamplesGet();


        WeightedSample<StatePosVel> maxSample = *std::max_element(samples.begin(), samples.end(), sampleWeightCompare);
        double maxSampleWeight = maxSample.WeightGet();

        //std::cout << "NSamples:" << samples.size() << " maxSampleWeight:" << maxSampleWeight << "------" << std::endl;
        int printFirstN = 200; int n = 0;
        for(vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
          geometry_msgs::Point32 point;
          point.x = (*sampleIt).ValueGet().pos_[0];
          point.y = (*sampleIt).ValueGet().pos_[1];
          point.z = 0;//(*sampleIt).WeightGet();//(*sampleIt).ValueGet().pos_[2];

          //
          int r,g,b;
          double weight;
          // If there is no sample with weight at all, make the particles blue
          if(maxSampleWeight == 0.0){
            r=0;
            g=0;
            b=255;
          }

          else
          {
            weight = (*sampleIt).WeightGet();
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


          if(n < printFirstN){
            //std::cout << "w: " << weight;
          }

          n++;
        }
        //std::cout << std::endl;




    }

    // Publish the pointcloud
    particles_pred_pub_.publish(particlesPCL);

    ROS_DEBUG("DualTracker::%s Publishing Particles on %s", __func__, fixed_frame.c_str());
  }

/*
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
          Stamped<tf::Point> center = (*legFeatureIt)->getPosition();

          geometry_msgs::Point32 point;
          point.x = center[0];
          point.y = center[1];
          point.z = 0;

          legPcl.points.push_back(point);

          // Set the color
          int r,g,b;
          //r = 255;
          getColor((*legFeatureIt)->getId(),r,g,b);

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
*/
  void publishLegHistory(vector<LegFeaturePtr>& legFeatures, ros::Time time){

    // Marker Array
    visualization_msgs::MarkerArray markerArray;

    // Iterate each leg
    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {

      // Iteration has to be at least of size 2 for a line drawing
      if((*legFeatureIt)->getHistory().size() > 1){

        // The geometry message
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = fixed_frame;
        line_list.header.stamp = time;
        line_list.id = (*legFeatureIt)->getId();
        line_list.ns = "history";

        // width
        line_list.scale.x = 0.01;

        // Set the color
        int r,g,b;
        getColor((*legFeatureIt)->getId(),r,g,b);

        line_list.color.r = r/255.0;
        line_list.color.g = g/255.0;
        line_list.color.b = b/255.0;
        line_list.color.a = 1.0;

        std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::const_iterator prevPointIt;
        std::vector<boost::shared_ptr<tf::Stamped<tf::Point> > >::const_iterator nextPointIt;

        // Use to pointers to pairwise iteration
        prevPointIt = (*legFeatureIt)->getHistory().begin();
        nextPointIt = (*legFeatureIt)->getHistory().begin();
        nextPointIt++;

        while(nextPointIt != (*legFeatureIt)->getHistory().end()){

          geometry_msgs::Point point0, point1;
          point0.x = (*prevPointIt)->getX();
          point0.y = (*prevPointIt)->getY();
          point0.z = 0;

          point1.x = (*nextPointIt)->getX();
          point1.y = (*nextPointIt)->getY();
          point1.z = 0;

          line_list.points.push_back(point0);
          line_list.points.push_back(point1);

          prevPointIt++;
          nextPointIt++;
        }

        // Publish the pointcloud
        markerArray.markers.push_back(line_list);
      }
    }

    leg_visualization_pub_.publish(markerArray);

    ROS_DEBUG("DualTracker::%s Publishing Leg History on %s", __func__, fixed_frame.c_str());
  }

  void publishLegTracker(vector<LegFeaturePtr>& legFeatures, ros::Time time){

    // Parameters
    double heightCreationLabel = 1.7;
    double cylinderHeight = 0.7;

    // Marker Array
    visualization_msgs::MarkerArray markerArray;

    int counter = 0;
    for (vector<LegFeaturePtr>::iterator legIt = legFeatures.begin();
        legIt != legFeatures.end();
        legIt++)
    {

      // Cylinder (Predicted Position)
      visualization_msgs::Marker markerMsgCylinder;

      markerMsgCylinder.header.frame_id = fixed_frame;
      markerMsgCylinder.header.stamp = time;
      markerMsgCylinder.ns = "predictions";
      markerMsgCylinder.id = counter;
      markerMsgCylinder.type = visualization_msgs::Marker::CYLINDER;
      markerMsgCylinder.scale.x = 0.03; // diameter x
      markerMsgCylinder.scale.y = 0.03; // diameter y
      markerMsgCylinder.scale.z = cylinderHeight;  // height
      markerMsgCylinder.color.a = 0.7;

      markerMsgCylinder.pose.position.x = (*legIt)->getPredictedPosition().getX();
      markerMsgCylinder.pose.position.y = (*legIt)->getPredictedPosition().getY();
      markerMsgCylinder.pose.position.z = cylinderHeight / 2;

      markerArray.markers.push_back(markerMsgCylinder);

      // Cylinder (Estimated Position)
      visualization_msgs::Marker markerEstimation;

      markerEstimation.header.frame_id = fixed_frame;
      markerEstimation.header.stamp = time;
      markerEstimation.ns = "estimation";
      markerEstimation.id = counter;
      markerEstimation.type = visualization_msgs::Marker::CYLINDER;
      markerEstimation.scale.x = 0.07; // diameter x
      markerEstimation.scale.y = 0.07; // diameter y
      markerEstimation.scale.z = 0.3;  // height
      markerEstimation.color.a = 0.9;

      markerEstimation.pose.position.x = (*legIt)->getEstimate().pos_.getX();
      markerEstimation.pose.position.y = (*legIt)->getEstimate().pos_.getY();
      markerEstimation.pose.position.z = 0.0;

      markerArray.markers.push_back(markerEstimation);


      // Cylinder
      visualization_msgs::Marker markerMsgArrow;

      markerMsgArrow.header.frame_id = fixed_frame;
      markerMsgArrow.header.stamp = time;
      markerMsgArrow.ns = "arrow_pred_corr";
      markerMsgArrow.id = counter;
      markerMsgArrow.type = visualization_msgs::Marker::ARROW;
      markerMsgArrow.scale.x = 0.02;
      markerMsgArrow.scale.y = 0.03;
      markerMsgArrow.scale.z = 0.03;
      markerMsgArrow.color.b = 1.0;
      markerMsgArrow.color.a = 0.8;

      geometry_msgs::Point point0, point1;
      point0.x = (*legIt)->getPredictedPosition().getX();
      point0.y = (*legIt)->getPredictedPosition().getY();
      point0.z = 0.0;

      markerMsgArrow.points.push_back(point0);

      point1.x = (*legIt)->getEstimate().pos_.getX();
      point1.y = (*legIt)->getEstimate().pos_.getY();
      point1.z = 0.0;

      markerMsgArrow.points.push_back(point1);

      markerArray.markers.push_back(markerMsgArrow);

      //// Initial position (Arrow)
      visualization_msgs::Marker creationMarkerArrow;

      creationMarkerArrow.header.frame_id = fixed_frame;
      creationMarkerArrow.header.stamp = time;
      creationMarkerArrow.ns = "arrow_creation";
      creationMarkerArrow.id = counter;
      creationMarkerArrow.type = visualization_msgs::Marker::ARROW;
      creationMarkerArrow.scale.x = 0.02;
      creationMarkerArrow.scale.y = 0.02;
      creationMarkerArrow.scale.z = 0.02;
      creationMarkerArrow.color.a = 0.8;

      geometry_msgs::Point point0_created, point1_created;
      point0_created.x = (*legIt)->getInitialPosition().getX();
      point0_created.y = (*legIt)->getInitialPosition().getY();
      point0_created.z = (*legIt)->getInitialPosition().getZ();

      point1_created = point0_created;
      point1_created.z = heightCreationLabel;

      creationMarkerArrow.points.push_back(point0_created);
      creationMarkerArrow.points.push_back(point1_created);

      markerArray.markers.push_back(creationMarkerArrow);

      //// Initial position (Label)
      visualization_msgs::Marker initial_label;
      initial_label.header.stamp = time;
      initial_label.header.frame_id = fixed_frame;
      initial_label.ns = "initial_creation_label";
      initial_label.id = counter;
      initial_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      initial_label.pose.position.x = (*legIt)->getInitialPosition().getX();
      initial_label.pose.position.y = (*legIt)->getInitialPosition().getY();
      initial_label.pose.position.z = 1.1 * heightCreationLabel;
      initial_label.scale.x = 0.1;
      initial_label.scale.y = 0.1;
      initial_label.scale.z = 0.1;
      initial_label.color.a = 1;
      //label.lifetime = ros::Duration(0.5);

      // Add text
      char buf[100];
      sprintf(buf, "L%d Creation", (*legIt)->getId());
      initial_label.text = buf;

      markerArray.markers.push_back(initial_label);


      counter++;

    }

    // Publish deletion markers
    for(int i = 0; i < n_leg_tracker_last_published_ - counter; i++){
      visualization_msgs::Marker deletionMarker0;
      deletionMarker0.header.stamp = time;
      deletionMarker0.header.frame_id = fixed_frame;
      deletionMarker0.id = counter + i;
      deletionMarker0.ns = "predictions";
      deletionMarker0.type = visualization_msgs::Marker::DELETE;
      deletionMarker0.scale.x = 0.1;
      deletionMarker0.scale.y = 0.1;
      deletionMarker0.scale.z = 0.1;

      markerArray.markers.push_back(deletionMarker0);

      visualization_msgs::Marker deletionMarker1;
      deletionMarker1.header.stamp = time;
      deletionMarker1.header.frame_id = fixed_frame;
      deletionMarker1.id = counter + i;
      deletionMarker1.ns = "estimation";
      deletionMarker1.type = visualization_msgs::Marker::DELETE;
      deletionMarker1.scale.x = 0.1;
      deletionMarker1.scale.y = 0.1;
      deletionMarker1.scale.z = 0.1;

      markerArray.markers.push_back(deletionMarker1);

      visualization_msgs::Marker deletionMarker2;
      deletionMarker2.header.stamp = time;
      deletionMarker2.header.frame_id = fixed_frame;
      deletionMarker2.id = counter + i;
      deletionMarker2.ns = "arrow_pred_corr";
      deletionMarker2.type = visualization_msgs::Marker::DELETE;
      deletionMarker2.scale.x = 0.1;
      deletionMarker2.scale.y = 0.1;
      deletionMarker2.scale.z = 0.1;

      markerArray.markers.push_back(deletionMarker2);
    }

    n_leg_tracker_last_published_ = counter;


    leg_visualization_pub_.publish(markerArray);

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
      label.ns = "leg_label";
      label.id = counter;
      label.type = label.TEXT_VIEW_FACING;
      label.pose.position.x = (*legFeatureIt)->getEstimate().pos_[0];
      label.pose.position.y = (*legFeatureIt)->getEstimate().pos_[1];
      label.pose.position.z = 0.8;
      label.scale.x = .1;
      label.scale.y = .1;
      label.scale.z = .2;
      label.color.a = 1;
      //label.lifetime = ros::Duration(0.5);

      // Add text
      char buf[100];
      sprintf(buf, "L%d", (*legFeatureIt)->getId());
      label.text = buf;

      labelArray.markers.push_back(label);

      counter++;

    }

    // Delete old labels
    for(int i = 0; i < n_leg_trackers_last_cycle_ - counter; ++i){
      visualization_msgs::Marker delete_label;
      delete_label.header.stamp = time;
      delete_label.header.frame_id = fixed_frame;
      delete_label.id = counter;
      delete_label.ns = "leg_label";
      delete_label.type = delete_label.DELETE;
      delete_label.scale.x = 1;
      delete_label.scale.y = 1;
      delete_label.scale.z = 1;

      labelArray.markers.push_back(delete_label);

    }
    n_leg_trackers_last_cycle_ = counter;

    // Publish
    leg_visualization_pub_.publish(labelArray);
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
        pointLeftLeg.x = (*peopleTrackerIt)->getLeftLeg()->getEstimate().pos_[0];
        pointLeftLeg.y = (*peopleTrackerIt)->getLeftLeg()->getEstimate().pos_[1];
        pointLeftLeg.z = 0;

        // Hip 0
        pointHipLeft.x = (*peopleTrackerIt)->getHipPosLeft()[0];
        pointHipLeft.y = (*peopleTrackerIt)->getHipPosLeft()[1];
        pointHipLeft.z = 0.0;

        // Center of the Person
        pointCenter.x = (*peopleTrackerIt)->getEstimate().pos_[0];
        pointCenter.y = (*peopleTrackerIt)->getEstimate().pos_[1];
        pointCenter.z = 0.0;

        // Hip 1
        pointHipRight.x = (*peopleTrackerIt)->getHipPosRight()[0];
        pointHipRight.y = (*peopleTrackerIt)->getHipPosRight()[1];
        pointHipRight.z = 0.0;

        // Leg 1
        pointLegRight.x = (*peopleTrackerIt)->getRightLeg()->getEstimate().pos_[0];
        pointLegRight.y = (*peopleTrackerIt)->getRightLeg()->getEstimate().pos_[1];
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

          leg_mov_marker.pose.position.x = (*peopleTrackerIt)->getLeg0Prediction().pos_[0];
          leg_mov_marker.pose.position.y = (*peopleTrackerIt)->getLeg0Prediction().pos_[1];
          leg_mov_marker.pose.position.z = 0.0;

          leg_stat_marker.pose.position.x = (*peopleTrackerIt)->getLeg1Prediction().pos_[0];
          leg_stat_marker.pose.position.y = (*peopleTrackerIt)->getLeg1Prediction().pos_[1];
          leg_stat_marker.pose.position.z = 0.0;

          msgArray.markers.push_back(leg_mov_marker);
          msgArray.markers.push_back(leg_stat_marker);

          //std::cout << "Leg0 Prediction" << (*peopleTrackerIt)->leg0Prediction_.pos_[0] << " " << (*peopleTrackerIt)->leg0Prediction_.pos_[1] << std::endl;
          //std::cout << "Leg1 Prediction" << (*peopleTrackerIt)->leg1Prediction_.pos_[0] << " " << (*peopleTrackerIt)->leg1Prediction_.pos_[1] << std::endl;

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
    people_visualization_pub_.publish(msgArray);


  }

  // Add Labels to the People Trackers
  void publishPeopleLabels(ros::Time time){

    // The marker Array
    visualization_msgs::MarkerArray markerArray;

    int counter = 0;
    for(vector<PeopleTrackerPtr>::iterator peopleTrackerIt = people_trackers_.getList()->begin();
        peopleTrackerIt != people_trackers_.getList()->end();
        peopleTrackerIt++){

      if((*peopleTrackerIt)->getTotalProbability() > 0.75 &&
          (publish_static_people_trackers_ || (*peopleTrackerIt)->isDynamic()))
      {
      visualization_msgs::Marker label;
      label.header.stamp = time;
      label.header.frame_id = fixed_frame;
      label.ns = "people_label";
      label.id = counter;
      label.type = label.TEXT_VIEW_FACING;
      label.pose.position.x = (*peopleTrackerIt)->getEstimate().pos_[0];
      label.pose.position.y = (*peopleTrackerIt)->getEstimate().pos_[1];
      label.pose.position.z = 0.5;
      label.scale.z = .1;
      label.color.a = 1;

      // Add text
      string state;

      if((*peopleTrackerIt)->isDynamic()){
        state = "dynamic";
      }
      else{
        state = "static";
      }
      char buf[100];
      sprintf(buf, "#PT%d-%d-p%.2f(%s)", (*peopleTrackerIt)->getId()[0], (*peopleTrackerIt)->getId()[1], (*peopleTrackerIt)->getTotalProbability(), state.c_str());
      label.text = buf;

      markerArray.markers.push_back(label);

      counter++;
      }
    }

    // Publish deletion markers
    for(int i = 0; i < n_people_label_markers_last_published_ - counter; i++){
      visualization_msgs::Marker deletionMarker0;
      deletionMarker0.header.stamp = time;
      deletionMarker0.header.frame_id = fixed_frame;
      deletionMarker0.id = counter + i;
      deletionMarker0.ns = "people_label";
      deletionMarker0.type = visualization_msgs::Marker::DELETE;
      deletionMarker0.scale.x = 1;
      deletionMarker0.scale.y = 1;
      deletionMarker0.scale.z = 1;

      markerArray.markers.push_back(deletionMarker0);

    }

    n_people_label_markers_last_published_ = counter;

    // Publish
    people_visualization_pub_.publish(markerArray);


  }

  // Add Labels to the People Trackers
  void publishPeople3d(ros::Time time){

    // The marker Array
    visualization_msgs::MarkerArray personsArray;

    int counter = 0;

    for(vector<PeopleTrackerPtr>::iterator peopleTrackerIt = people_trackers_.getList()->begin();
        peopleTrackerIt != people_trackers_.getList()->end();
        peopleTrackerIt++){

      if( (*peopleTrackerIt)->isValid() &&
          (*peopleTrackerIt)->getTotalProbability() > 0.75 &&
          (publish_static_people_trackers_ || (*peopleTrackerIt)->isDynamic()))
      {
      visualization_msgs::Marker person3d;
      person3d.header.stamp = time;
      person3d.header.frame_id = fixed_frame;

      int r0,g0,b0,r1,g1,b1;
      //r = 255;
      getColor((*peopleTrackerIt)->getLeg0()->getId(),r0,g0,b0);
      getColor((*peopleTrackerIt)->getLeg1()->getId(),r1,g1,b1);


      double personHeight = 1;
      double personWidth = 0.25;

      person3d.id = counter;
      person3d.type = visualization_msgs::Marker::CYLINDER;
      person3d.pose.position.x = (*peopleTrackerIt)->getEstimate().pos_[0];
      person3d.pose.position.y = (*peopleTrackerIt)->getEstimate().pos_[1];
      person3d.pose.position.z = personHeight/4;
      person3d.ns = "person3d";
      person3d.scale.x = personWidth;
      person3d.scale.y = personWidth;
      person3d.scale.z = personHeight/2;

      person3d.color.r = 0;
      person3d.color.g = 0;
      person3d.color.b = 1;

      counter++;
     // person3d.lifetime = ros::Duration(8);


      // Set the color as the mixture of both leg track colors

      visualization_msgs::Marker personHead;
      personHead.header.stamp = time;
      personHead.header.frame_id = fixed_frame;
      personHead.id = counter;
      personHead.ns = "person3d";
      personHead.type = visualization_msgs::Marker::SPHERE;
      personHead.pose.position.x = (*peopleTrackerIt)->getEstimate().pos_[0];
      personHead.pose.position.y = (*peopleTrackerIt)->getEstimate().pos_[1];
      personHead.pose.position.z = personHeight/2 * 1.2;
      personHead.scale.x = personWidth*1.1;
      personHead.scale.y = personWidth*1.1;
      personHead.scale.z = personWidth*1.1;
      personHead.color.r = (r0+r1)/(2*255.0);
      personHead.color.g = (g0+g1)/(2*255.0);
      personHead.color.b = (b0+b1)/(2*255.0);
      //personHead.lifetime = ros::Duration(8);

      // Static / Dynamic
      if((*peopleTrackerIt)->isDynamic()){
        person3d.color.a = 0.75;
        personHead.color.a = 0.75;

      }
      else{
        person3d.color.a = 0.4;
        personHead.color.a = 0.4;
      }


      personsArray.markers.push_back(person3d);
      personsArray.markers.push_back(personHead);

      counter++;
      }
    }


    // Publish deletion markers
    for(int i = 0; i < n_people_markers_last_published_ - counter; i++){
      visualization_msgs::Marker deletionMarker0;
      deletionMarker0.header.stamp = time;
      deletionMarker0.header.frame_id = fixed_frame;
      deletionMarker0.id = counter + i;
      deletionMarker0.ns = "person3d";
      deletionMarker0.type = visualization_msgs::Marker::DELETE;
      deletionMarker0.scale.x = 1;
      deletionMarker0.scale.y = 1;
      deletionMarker0.scale.z = 1;

      personsArray.markers.push_back(deletionMarker0);

      visualization_msgs::Marker deletionMarker1;
      deletionMarker1.header.stamp = time;
      deletionMarker1.header.frame_id = fixed_frame;
      deletionMarker1.id = counter + i;
      deletionMarker1.ns = "person3d";
      deletionMarker1.type = visualization_msgs::Marker::DELETE;
      deletionMarker1.scale.x = 1;
      deletionMarker1.scale.y = 1;
      deletionMarker1.scale.z = 1;

      personsArray.markers.push_back(deletionMarker1);

    }

    n_people_markers_last_published_ = counter;

    // Publish
    people_visualization_pub_.publish(personsArray);

  }

  // Add Labels to the People Trackers
  void publishFakeMeasPos(std::vector<DetectionPtr> fakeDetections, ros::Time time, tf::Stamped<tf::Point> sensorCoord){

    visualization_msgs::MarkerArray msgArray;

    if(fakeDetections.size() == 0){
      // The geometry message
      visualization_msgs::Marker sphere;
      sphere.type = visualization_msgs::Marker::DELETE;
      sphere.header.frame_id = fixed_frame;
      sphere.header.stamp = time;
      sphere.id = 0;
      // width
      sphere.scale.x = 0.2;
      sphere.scale.y = 0.2;
      sphere.scale.z = 0.2;
      sphere.ns = "FakeMeasurements";
      msgArray.markers.push_back(sphere);
    }

    int counter = 0;
    for (vector<DetectionPtr>::iterator detectionIt = fakeDetections.begin();
        detectionIt != fakeDetections.end();
        detectionIt++)
    {

        // The geometry message
        visualization_msgs::Marker sphere;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.header.frame_id = fixed_frame;
        sphere.header.stamp = time;
        sphere.id = counter;
        sphere.ns = "FakeMeasurements";
        sphere.pose.position.x = (*detectionIt)->getLocation()[0];
        sphere.pose.position.y = (*detectionIt)->getLocation()[1];
        sphere.pose.position.z = 0;
        sphere.color.r = 0.8;
        sphere.color.g = 0;
        sphere.color.b = 0;
        sphere.color.a = 0.8;

        // width
        sphere.scale.x = 0.2;
        sphere.scale.y = 0.2;
        sphere.scale.z = 0.2;

        // Set the color

        int r,g,b;
        r = 255;
        //getColor((*legFeatureIt)->int_id_,r,g,b);

        counter++;

        // Publish the pointcloud
        msgArray.markers.push_back(sphere);


    }

    leg_visualization_pub_.publish(msgArray);
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
          (*peopleIt)->getHistorySize() > 1
      )
      {

        // The geometry message
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = fixed_frame;
        line_list.header.stamp = time;
        line_list.id = (*peopleIt)->getLeg0()->getId()* 1000 + (*peopleIt)->getLeg1()->getId();
        line_list.ns = "people_history";
        //line_list.lifetime = ros::Duration(4);

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

        std::vector< people_history_entry >::iterator prevPointIt;
        std::vector< people_history_entry >::iterator nextPointIt;

        prevPointIt = (*peopleIt)->getPositionHistory().begin();
        nextPointIt = (*peopleIt)->getPositionHistory().begin();
        nextPointIt++;

        while(nextPointIt != (*peopleIt)->getPositionHistory().end()){
          geometry_msgs::Point point0, point1;
          point0.x = (*prevPointIt).position_->getX();
          point0.y = (*prevPointIt).position_->getY();
          point0.z = 0;

          point1.x = (*nextPointIt).position_->getX();
          point1.y = (*nextPointIt).position_->getY();
          point1.z = 0;

          line_list.points.push_back(point0);
          line_list.points.push_back(point1);

          // Set alpha depending on velocity
          line_list.color.a = sigmoid((*nextPointIt).probability_, 10.0, 0.8);

          //std::cout << "[" << counter << "]" << point0.x << " " << point0.y << "---->" << point1.x << " " << point1.y << std::endl;

          prevPointIt++;
          nextPointIt++;

        }
        counter++;

        // Publish the pointcloud
        msgArray.markers.push_back(line_list);

      }
    }

    people_visualization_pub_.publish(msgArray);
    //leg_features_history_vis_pub_.publish(line_list);

    ROS_DEBUG("DualTracker::%s Publishing Leg History on %s", __func__, fixed_frame.c_str());
  }

  void publishDataAssociationVisualization(std::vector<Association*> associationSet, ros::Time time){

    // Parameters
    double associationLineHeight = 0;
    double associationLabelHeight = 1;

    // Parameters for the cylinder connecting line and label
    double cylinderDiameter = 0.02;
    double cylinderAlpha = 0.5;

    // Create a marker array
    visualization_msgs::MarkerArray markerArray;

    // Iterate the associations
    // Print the Associations
    int counter = 0;
    for(int i = 0; i < associationSet.size(); ++i){

      // Get the association
      Association* association = associationSet[i];

      // Get the LegFeaturePtr
      LegFeaturePtr leg = association->getLeg();

      // Get the detection
      DetectionPtr detection = association->getDetection();

      // Abort if the detection was not used in the data association
      if(!detection->usedForUpdate()) continue;

      // Create a marker connecting leg and detection
      visualization_msgs::Marker line;

      line.header.frame_id = fixed_frame;
      line.header.stamp = time;
      line.ns = "association";
      line.id = i;
      line.type = visualization_msgs::Marker::LINE_LIST;
      line.scale.x = 0.05;
      line.scale.y = 0.05;
      line.color.a = 0.8;

      geometry_msgs::Point point0, point1;
      point0.x = leg->getPredictedPosition().getX();
      point0.y = leg->getPredictedPosition().getY();
      point0.z = associationLineHeight;

      line.points.push_back(point0);

      point1.x = detection->getLocation().getX();
      point1.y = detection->getLocation().getY();
      point1.z = associationLineHeight;

      line.points.push_back(point1);

      // Push the line into the array
      markerArray.markers.push_back(line);

      //// Add a label in the center of the connecting line
      visualization_msgs::Marker associationLabel;

      associationLabel.header.frame_id = fixed_frame;
      associationLabel.header.stamp = time;
      associationLabel.ns = "association_label";
      associationLabel.id = i;
      associationLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

      // Add text
      char buf[100];
      sprintf(buf, "as<L%i-M%i>: %g", leg->getId(), detection->getId(), detection->getProbability());
      associationLabel.text = buf;

      // Set the position as center of the line
      associationLabel.pose.position.x = 0.5 * (leg->getPredictedPosition().getX() + detection->getLocation().getX());
      associationLabel.pose.position.y = 0.5 * (leg->getPredictedPosition().getY() + detection->getLocation().getY());
      associationLabel.pose.position.z = associationLabelHeight;

      associationLabel.scale.z = .1;
      associationLabel.color.r = 0;
      associationLabel.color.g = 0;
      associationLabel.color.b = 0;
      associationLabel.color.a = 1.0;

      markerArray.markers.push_back(associationLabel);

      //// Add a cylinder from the line center to the label
      visualization_msgs::Marker associationLineLabelCylinderMarker;
      associationLineLabelCylinderMarker.header.frame_id = fixed_frame;
      associationLineLabelCylinderMarker.header.stamp = time;
      associationLineLabelCylinderMarker.ns = "association_label_line_connection";
      associationLineLabelCylinderMarker.id = i;
      associationLineLabelCylinderMarker.type = visualization_msgs::Marker::CYLINDER;

      double cylinderHeight = 0.9 * associationLabelHeight;

      // Set the position as center of the line
      associationLineLabelCylinderMarker.pose.position.x = 0.5 * (leg->getPredictedPosition().getX() + detection->getLocation().getX());
      associationLineLabelCylinderMarker.pose.position.y = 0.5 * (leg->getPredictedPosition().getY() + detection->getLocation().getY());
      associationLineLabelCylinderMarker.pose.position.z = associationLineHeight + 0.5 * cylinderHeight;
      associationLineLabelCylinderMarker.scale.x = cylinderDiameter;
      associationLineLabelCylinderMarker.scale.y = cylinderDiameter;
      associationLineLabelCylinderMarker.scale.z = cylinderHeight;

      associationLineLabelCylinderMarker.color.r = 0;
      associationLineLabelCylinderMarker.color.g = 0;
      associationLineLabelCylinderMarker.color.b = 0;
      associationLineLabelCylinderMarker.color.a = cylinderAlpha;

      markerArray.markers.push_back(associationLineLabelCylinderMarker);

      counter++;

    }

    // Publish deletion markers
    for(int i = 0; i < n_associations_last_published_ - counter; i++){
      visualization_msgs::Marker deletionMarker0;
      deletionMarker0.header.stamp = time;
      deletionMarker0.header.frame_id = fixed_frame;
      deletionMarker0.id = counter + i;
      deletionMarker0.ns = "association";
      deletionMarker0.type = visualization_msgs::Marker::DELETE;

      markerArray.markers.push_back(deletionMarker0);

      visualization_msgs::Marker deletionMarker1;
      deletionMarker1.header.stamp = time;
      deletionMarker1.header.frame_id = fixed_frame;
      deletionMarker1.id = counter + i;
      deletionMarker1.ns = "association_label";
      deletionMarker1.type = visualization_msgs::Marker::DELETE;

      markerArray.markers.push_back(deletionMarker1);

      visualization_msgs::Marker deletionMarker2;
      deletionMarker2.header.stamp = time;
      deletionMarker2.header.frame_id = fixed_frame;
      deletionMarker2.id = counter + i;
      deletionMarker2.ns = "association_label_line_connection";
      deletionMarker2.type = visualization_msgs::Marker::DELETE;

      markerArray.markers.push_back(deletionMarker2);

    }

    n_associations_last_published_ = counter;

    association_visualization_pub_.publish(markerArray);

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
        MCPdf<StatePosVel>* mc = (*legFeatureIt)->postGet();

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

  void publishParticlesArrows(vector<LegFeaturePtr>& legFeatures, ros::Time time){
    // Marker Array
    visualization_msgs::MarkerArray markerArray;

    int id_counter = 0;

    for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
        legFeatureIt != legFeatures.end();
        legFeatureIt++)
    {
        MCPdf<StatePosVel>* mc = (*legFeatureIt)->postGet();

        vector<WeightedSample<StatePosVel> > samples = mc->ListOfSamplesGet();


        WeightedSample<StatePosVel> maxSample = *std::max_element(samples.begin(), samples.end(), sampleWeightCompare);
        double maxSampleWeight = maxSample.WeightGet();

        int counter=0;

        for(vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){

          // Not a arrow for every particle
          counter++;

          if(counter % 5 != 0) continue;

          geometry_msgs::Point point_start;
          point_start.x = (*sampleIt).ValueGet().pos_[0]  + (*sampleIt).ValueGet().vel_[0] * 1.0/12;;
          point_start.y = (*sampleIt).ValueGet().pos_[1]  + (*sampleIt).ValueGet().vel_[1] * 1.0/12;;
          point_start.z = (*sampleIt).WeightGet();//(*sampleIt).ValueGet().pos_[2];

          geometry_msgs::Point point_end;
          point_end.x = (*sampleIt).ValueGet().pos_[0] + (*sampleIt).ValueGet().vel_[0] * 2.0/12;
          point_end.y = (*sampleIt).ValueGet().pos_[1] + (*sampleIt).ValueGet().vel_[1] * 2.0/12;
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

  void publishParticlesPredArrows(vector<LegFeaturePtr>& legFeatures, ros::Time time){
     // Marker Array
     visualization_msgs::MarkerArray markerArray;

     int id_counter = 0;

     for (vector<LegFeaturePtr>::iterator legFeatureIt = legFeatures.begin();
         legFeatureIt != legFeatures.end();
         legFeatureIt++)
     {
         MCPdf<StatePosVel>* mc = (*legFeatureIt)->postGet();

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
     particles_pred_arrow_pub_.publish(markerArray);

     ROS_DEBUG("DualTracker::%s Publishing Particles Arrows on %s", __func__, fixed_frame.c_str());
   }


};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "dual_tracker");
  g_argc = argc;
  g_argv = argv;
  ros::NodeHandle nh;
  DualTracker ld(nh);

  ROS_INFO("DUAL TRACKER started!");

  ros::spin();

  return 0;
}

