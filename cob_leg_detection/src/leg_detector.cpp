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
#include <cob_leg_detection/constants.h>
#include <cob_leg_detection/leg_detector.h>
#include <cob_leg_detection/LegDetectorConfig.h>
#include <cob_leg_detection/laser_processor.h>
#include <cob_leg_detection/calc_leg_features.h>
//#include <cob_leg_detection/visualization_conversions.h>
#include <benchmarking/timer.h>
#include <cob_leg_detection/saved_feature.h>

// OpenCV includes
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

// Messages
#include <cob_perception_msgs/PositionMeasurement.h>
#include <cob_perception_msgs/PositionMeasurementArray.h>
#include <cob_perception_msgs/Person.h>
#include <cob_perception_msgs/People.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

// Transforms
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

// People tracking
#include <cob_people_tracking_filter/tracker_kalman.h>
#include <cob_people_tracking_filter/state_pos_vel.h>
#include <cob_people_tracking_filter/rgb.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <algorithm>
#include <vector>

// Namespaces
//using namespace std;
//using namespace laser_processor;
//using namespace ros;
//using namespace tf;
//using namespace estimation;
//using namespace BFL;
//using namespace MatrixWrapper;



// Defines
#define LEGDETECTOR_DEBUG 1         // Debug the leg detector
#define LEGDETECTOR_TIME_DEBUG 1    // Debug the calculation time inside the leg_detector

class MatchedFeature
{
public:
  SampleSet* candidate_;  // The point cluster
  SavedFeature* closest_; // The feature/leg tracker
  float distance_;		  // The distance between the
  double probability_;

  MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance, double probability)
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

// actual legdetector node
class LegDetector
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

  list<SavedFeature*> saved_features_; /**< List of SavedFeatures(Legs) */

  boost::mutex saved_mutex_; /**< Mutex to handle the access to the Saved Features */

  int feature_id_;

  bool use_seeds_;

  bool publish_legs_, publish_people_, publish_leg_markers_, publish_people_markers_, publish_clusters_;

  int next_p_id_;

  double leg_reliability_limit_;

  int min_points_per_group;

  // The publishers
  ros::Publisher people_measurements_pub_; /**< Publisher for people measurements */
  ros::Publisher leg_measurements_pub_; /**< Publisher for leg measurements */
  ros::Publisher markers_pub_; /**< Publisher for features */
  ros::Publisher clusters_pub_;/**< Publisher for the clusters generated by scan processor */

  dynamic_reconfigure::Server<cob_leg_detection::LegDetectorConfig> server_; /**< The configuration server*/

  message_filters::Subscriber<cob_perception_msgs::PositionMeasurement> people_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<cob_perception_msgs::PositionMeasurement> people_notifier_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

  LegDetector(ros::NodeHandle nh) :
    nh_(nh),
    mask_count_(0),
    feat_count_(0),
    next_p_id_(0),
    people_sub_(nh_, "people_tracker_filter", 10),
    laser_sub_(nh_, "scan", 10),
    people_notifier_(people_sub_, tfl_, fixed_frame, 10),
    laser_notifier_(laser_sub_, tfl_, fixed_frame, 10)
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
    leg_measurements_pub_ = nh_.advertise<cob_perception_msgs::PositionMeasurementArray>("leg_tracker_measurements", 0);
    people_measurements_pub_ = nh_.advertise<cob_perception_msgs::PositionMeasurementArray>("people_tracker_measurements", 0);
    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud>("clusters", 0);

    if (use_seeds_)
    {
      people_notifier_.registerCallback(boost::bind(&LegDetector::peopleCallback, this, _1));
      people_notifier_.setTolerance(ros::Duration(0.01));
    }

    // Set the laserCallback
    laser_notifier_.registerCallback(boost::bind(&LegDetector::laserCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));

    dynamic_reconfigure::Server<cob_leg_detection::LegDetectorConfig>::CallbackType f;
    f = boost::bind(&LegDetector::configure, this, _1, _2);
    server_.setCallback(f);

    feature_id_ = 0;
  }


  ~LegDetector()
  {
  }

  /**
   *  @brief Handles the configuration of this node
   */

  void configure(cob_leg_detection::LegDetectorConfig &config, uint32_t level)
  {
    connected_thresh_       = config.connection_threshold;
    min_points_per_group    = config.min_points_per_group;
    leg_reliability_limit_  = config.leg_reliability_limit;
    publish_legs_           = config.publish_legs;
    publish_people_         = config.publish_people;
    publish_leg_markers_    = config.publish_leg_markers;
    publish_people_markers_ = config.publish_people_markers;
    publish_clusters_       = true;

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

    kal_p                    = config.kalman_p;
    kal_q                    = config.kalman_q;
    kal_r                    = config.kalman_r;
    use_filter               = config.kalman_on == 0;
  }

  /**
   *  @brief The distance between two legs.
   *
   *  Calculates the euclidian distance between to features(legs)
   */
  double distance(list<SavedFeature*>::iterator it1,  list<SavedFeature*>::iterator it2)
  {
    Stamped<Point> one = (*it1)->position_;
    Stamped<Point> two = (*it2)->position_;

    double distance = (one-two).length();
  }

  // Find the tracker that is closest to this person message
  // If a tracker was already assigned to a person, keep this assignment when the distance between them is not too large.
  void peopleCallback(const cob_perception_msgs::PositionMeasurement::ConstPtr& people_meas)
  {
    assert(false);

    // If there are no legs, return.
    if (saved_features_.empty())
      return;

    Point pt;
    pointMsgToTF(people_meas->pos, pt);
    Stamped<Point> person_loc(pt, people_meas->header.stamp, people_meas->header.frame_id);
    person_loc[2] = 0.0; // Ignore the height of the person measurement.
    Stamped<Point> dest_loc(pt, people_meas->header.stamp, people_meas->header.frame_id); // Holder for all transformed pts.

    // Lock the current scope using the mutex
    boost::mutex::scoped_lock lock(saved_mutex_);

    list<SavedFeature*>::iterator closest = saved_features_.end();
    list<SavedFeature*>::iterator closest1 = saved_features_.end();
    list<SavedFeature*>::iterator closest2 = saved_features_.end();
    float closest_dist = max_meas_jump_m;
    float closest_pair_dist = 2 * max_meas_jump_m;

    list<SavedFeature*>::iterator begin = saved_features_.begin();
    list<SavedFeature*>::iterator end = saved_features_.end();
    list<SavedFeature*>::iterator it1, it2;

    // If there's a pair of legs with the right label and within the max dist, return
    // If there's one leg with the right label and within the max dist,
    //   find a partner for it from the unlabeled legs whose tracks are reasonably new.
    //   If no partners exist, label just the one leg.
    // If there are no legs with the right label and within the max dist,
    //   find a pair of unlabeled legs and assign them the label.
    // If all of the above cases fail,
    //   find a new unlabeled leg and assign the label.

    // For each tracker, get the distance to this person.
    for (it1 = begin; it1 != end; ++it1)
    {
      try
      {
        tfl_.transformPoint((*it1)->id_, people_meas->header.stamp,
                            person_loc, fixed_frame, dest_loc);
        //ROS_INFO("Succesful leg transformation at spot 7");
      }
      catch (...)
      {
        ROS_WARN("TF exception spot 7.");
      }
      (*it1)->dist_to_person_ = dest_loc.length();
    }

    // Try to find one or two trackers with the same label and within the max distance of the person.
    cout << "Looking for two legs" << endl;
    it2 = end;
    for (it1 = begin; it1 != end; ++it1)
    {
      // If this leg belongs to the person...
      if ((*it1)->object_id == people_meas->object_id)
      {
        // and their distance is close enough...
        if ((*it1)->dist_to_person_ < max_meas_jump_m)
        {
          // if this is the first leg we've found, assign it to it2. Otherwise, leave it assigned to it1 and break.
          if (it2 == end)
            it2 = it1;
          else
            break;
        }
        // Otherwise, remove the tracker's label, it doesn't belong to this person.
        else
        {
          // the two trackers moved apart. This should not happen.
          (*it1)->object_id = "";
        }
      }
    }

    // If we found two legs with the right label and within the max distance, all is good, return.
    if (it1 != end && it2 != end)
    {
      cout << "Found matching pair. The second distance was " << (*it1)->dist_to_person_ << endl;
      return;
    }

    // If we only found one close leg with the right label, let's try to find a second leg that
    //   * doesn't yet have a label  (=valid precondition),
    //   * is within the max distance,
    //   * is less than max_second_leg_age_s old.
    cout << "Looking for one leg plus one new leg" << endl;
    float dist_between_legs, closest_dist_between_legs;
    if (it2 != end)
    {
      closest_dist = max_meas_jump_m;
      closest = saved_features_.end();

      for (it1 = begin; it1 != end; ++it1)
      {
        // Skip this leg track if:
        // - you're already using it.
        // - it already has an id.
        // - it's too old. Old unassigned trackers are unlikely to be the second leg in a pair.
        // - it's too far away from the person.
        if ((it1 == it2) || ((*it1)->object_id != "") || ((*it1)->getLifetime() > max_second_leg_age_s) || ((*it1)->dist_to_person_ >= closest_dist))
          continue;

        // Get the distance between the two legs
        try
        {
          tfl_.transformPoint((*it1)->id_, (*it2)->position_.stamp_, (*it2)->position_, fixed_frame, dest_loc);
        }
        catch (...)
        {
          ROS_WARN("TF exception getting distance between legs.");
        }
        dist_between_legs = dest_loc.length();

        // If this is the closest dist (and within range), and the legs are close together and unlabeled, mark it.
        if (dist_between_legs < leg_pair_separation_m)
        {
          closest = it1;
          closest_dist = (*it1)->dist_to_person_;
          closest_dist_between_legs = dist_between_legs;
        }
      }
      // If we found a close, unlabeled leg, set it's label.
      if (closest != end)
      {
        cout << "Replaced one leg with a distance of " << closest_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
        (*closest)->object_id = people_meas->object_id;
      }
      else
      {
        cout << "Returned one matched leg only" << endl;
      }

      // Regardless of whether we found a second leg, return.
      return;
    }

    cout << "Looking for a pair of new legs" << endl;
    // If we didn't find any legs with this person's label, try to find two unlabeled legs that are close together and close to the tracker.
    it1 = saved_features_.begin();
    it2 = saved_features_.begin();
    closest = saved_features_.end();
    closest1 = saved_features_.end();
    closest2 = saved_features_.end();
    closest_dist = max_meas_jump_m;
    closest_pair_dist = 2 * max_meas_jump_m;
    for (; it1 != end; ++it1)
    {
      // Only look at trackers without ids and that are not too far away.
      if ((*it1)->object_id != "" || (*it1)->dist_to_person_ >= max_meas_jump_m)
        continue;

      // Keep the single closest leg around in case none of the pairs work out.
      if ((*it1)->dist_to_person_ < closest_dist)
      {
        closest_dist = (*it1)->dist_to_person_;
        closest = it1;
      }

      // Find a second leg.
      it2 = it1;
      it2++;
      for (; it2 != end; ++it2)
      {
        // Only look at trackers without ids and that are not too far away.
        if ((*it2)->object_id != "" || (*it2)->dist_to_person_ >= max_meas_jump_m)
          continue;

        // Get the distance between the two legs
        try
        {
          tfl_.transformPoint((*it1)->id_, (*it2)->position_.stamp_, (*it2)->position_, fixed_frame, dest_loc);
        }
        catch (...)
        {
          ROS_WARN("TF exception getting distance between legs in spot 2.");
        }
        dist_between_legs = dest_loc.length();

        // Ensure that this pair of legs is the closest pair to the tracker, and that the distance between the legs isn't too large.
        if ((*it1)->dist_to_person_ + (*it2)->dist_to_person_ < closest_pair_dist && dist_between_legs < leg_pair_separation_m)
        {
          closest_pair_dist = (*it1)->dist_to_person_ + (*it2)->dist_to_person_;
          closest1 = it1;
          closest2 = it2;
          closest_dist_between_legs = dist_between_legs;
        }
      }
    }
    // Found a pair of legs.
    if (closest1 != end && closest2 != end)
    {
      (*closest1)->object_id = people_meas->object_id;
      (*closest2)->object_id = people_meas->object_id;
      cout << "Found a completely new pair with total distance " << closest_pair_dist << " and a distance between the legs of " << closest_dist_between_legs << endl;
      return;
    }

    cout << "Looking for just one leg" << endl;
    // No pair worked, try for just one leg.
    if (closest != end)
    {
      (*closest)->object_id = people_meas->object_id;
      cout << "Returned one new leg only" << endl;
      return;
    }

    cout << "Nothing matched" << endl;
  }

  /**
   *  @brief Pair the Saved Features
   *
   *  Pairs the features inside the member saved_features_ with each other based on the distance.
   */
  void pairLegs()
  {
    ROS_DEBUG_COND(LEGDETECTOR_DEBUG,"LegDetector::%s - Pairing Legs",__func__);
    benchmarking::Timer pairLegsTimer; pairLegsTimer.start();

    // Deal With legs that already have ids
    list<SavedFeature*>::iterator begin = saved_features_.begin();
    list<SavedFeature*>::iterator end = saved_features_.end();
    list<SavedFeature*>::iterator leg1, leg2, best, it;

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
      list<SavedFeature*>::iterator best1 = end, best2 = end;
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
    ROS_DEBUG_COND(LEGDETECTOR_TIME_DEBUG,"LegDetector::%s - Pairing Legs took %f ms",__func__, pairLegsTimer.stopAndGetTimeMs());


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
    ROS_DEBUG_COND(LEGDETECTOR_DEBUG,"LegDetector::%s - Received Laserscan",__func__);

    // Process the incoming scan
    benchmarking::Timer processTimer; processTimer.start();
    ScanProcessor processor(*scan, mask_);
    processor.splitConnected(connected_thresh_);
    processor.removeLessThan(3);
    ROS_DEBUG_COND(LEGDETECTOR_TIME_DEBUG,"LegDetector::%s - Process %f ms",__func__, processTimer.stopAndGetTimeMs());

    // if no measurement matches to a tracker in the last <no_observation_timeout>  seconds: erase tracker
    ros::Time purge = scan->header.stamp + ros::Duration().fromSec(-no_observation_timeout_s);

    benchmarking::Timer removeTimer; removeTimer.start();
    // Iterate through the saved features and remove those who havent been observed since (no_observation_timeout_s)
    int deletionCounter = 0;
    list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
    while (sf_iter != saved_features_.end())
    {
      // If there was no measurement of this feature in the last 'no_observation_timeout_s' seconds-> removed it and clear the link of its partner
      // IDEA_ make this dependent on the observation model
      if ((*sf_iter)->meas_time_ < purge)
      {
        if ((*sf_iter)->other) // If leg is paired
          (*sf_iter)->other->other = NULL; //Remove link to itself from partner
        delete(*sf_iter);
        saved_features_.erase(sf_iter++);
        ++deletionCounter;
      }
      else
        ++sf_iter;
    }
    removeTimer.stop();
    ROS_DEBUG_COND(LEGDETECTOR_DEBUG,"LegDetector::%s - Removed %i features because the havent been detected in the last %f seconds",__func__, deletionCounter, no_observation_timeout_s);
    ROS_DEBUG_COND(LEGDETECTOR_TIME_DEBUG,"LegDetector::%s - Removing features took %f ms",__func__, removeTimer.getElapsedTimeMs());


    // System update of trackers, and copy updated ones in propagate list
    benchmarking::Timer propagationTimer; propagationTimer.start();
    list<SavedFeature*> propagated;
    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++)
    {
      (*sf_iter)->propagate(scan->header.stamp);
      propagated.push_back(*sf_iter);
    }
    propagationTimer.stop();
    ROS_DEBUG_COND(LEGDETECTOR_DEBUG,"LegDetector::%s - Propagated %i SavedFeatures",__func__, (int) propagated.size());
    ROS_DEBUG_COND(LEGDETECTOR_TIME_DEBUG,"LegDetector::%s - Propagating took %f ms",__func__, propagationTimer.getElapsedTimeMs());


    // Detection step: build up the set of "candidate" clusters
    // For each candidate, find the closest tracker (within threshold) and add to the match list
    // If no tracker is found, start a new one
    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

    unsigned int newTrackCounter = 0;
    unsigned int matchesCounter = 0;
    multiset<MatchedFeature> matches;
    for (list<SampleSet*>::iterator clusterIt = processor.getClusters().begin();
         clusterIt != processor.getClusters().end();
         clusterIt++)
    {
      // Calculate the features of the clusters
      vector<float> f = calcLegFeatures(*clusterIt, *scan); // Calculate the single features -> results in a vector

      for (int k = 0; k < feat_count_; k++){
        tmp_mat->data.fl[k] = (float)(f[k]);
      }

      // Predict the probability of this cluster in beeing a leg
      float probability = forest.predict_prob(tmp_mat);  // Predict the probability using the forest
      Stamped<Point> loc((*clusterIt)->center(), scan->header.stamp, scan->header.frame_id);
      try
      {
        tfl_.transformPoint(fixed_frame, loc, loc); //Transform using odometry information into the fixed frame
      }
      catch (...)
      {
        ROS_WARN("TF exception spot 3.");
      }

      // Find the closest tracker (Note that the tracker has been updated using the kalman filter!)
      // Multiple measurements could be assigned to the same tracker! This problem is solved below. Better methods could be thought of.
      // IDEA_ Do this better! The closest is no necessarily the right one
      list<SavedFeature*>::iterator closest = propagated.end();
      float closest_dist = max_track_jump_m;

      // Iterate through the trackers
      for (list<SavedFeature*>::iterator pf_iter = propagated.begin();
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
        list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
        ++newTrackCounter;
      }
      // Add the candidate, the tracker and the distance to a match list
      else{
        matches.insert(MatchedFeature(*clusterIt, *closest, closest_dist, probability));
        ++matchesCounter;
      }

    }// end iterate the clusters

    ROS_DEBUG_COND(LEGDETECTOR_DEBUG,"LegDetector::%s - Associated tracks to legs - %i matches - %i new tracks",__func__, matchesCounter, newTrackCounter);
    // IDEA_ The next step contains one flaw, it is random which closest tracker get choosen and update the tracker, this unacceptable
    // IDEA_ Deploy Linear Association Problem Solver Here
    // Update the matched trackers
    while (matches.size() > 0)
    {
      multiset<MatchedFeature>::iterator matched_iter = matches.begin();  // Matches iterator
      bool found = false;
      list<SavedFeature*>::iterator pf_iter = propagated.begin(); // Tracker iterator
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

          // Update the tracker with the candidate location
          matched_iter->closest_->update(loc, matched_iter->probability_);

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
      // try to assign the candidate to another tracker
      // Explanation: since multiple features could be assigned to the same tracker this can happend. The solution here to this however is not optimal.
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

        list<SavedFeature*>::iterator closest = propagated.end();
        float closest_dist = max_track_jump_m;

        for (list<SavedFeature*>::iterator remain_iter = propagated.begin();
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
        if (closest == propagated.end())
          list<SavedFeature*>::iterator new_saved = saved_features_.insert(saved_features_.end(), new SavedFeature(loc, tfl_));
        else
          matches.insert(MatchedFeature(matched_iter->candidate_, *closest, closest_dist, matched_iter->probability_));
        matches.erase(matched_iter);
      }
    }
    ROS_DEBUG_COND(LEGDETECTOR_DEBUG,"LegDetector::%s - Updated the trackers",__func__);


    // Clearance
    cvReleaseMat(&tmp_mat);
    tmp_mat = 0;
    if (use_seeds_)
      pairLegs();

    // Publish Data!
    int i = 0;
    vector<cob_perception_msgs::PositionMeasurement> people;
    vector<cob_perception_msgs::PositionMeasurement> legs;

    // Iterate the features
    for (list<SavedFeature*>::iterator sf_iter = saved_features_.begin();
         sf_iter != saved_features_.end();
         sf_iter++, i++)
    {
      // reliability
      double reliability = (*sf_iter)->getReliability();

      if ((*sf_iter)->getReliability() > leg_reliability_limit_ && publish_legs_)
      {
        cob_perception_msgs::PositionMeasurement pos;
        pos.header.stamp = scan->header.stamp;
        pos.header.frame_id = scan->header.frame_id;
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




      if (publish_leg_markers_)
      {
        // Publish the cluster as Sphere
        visualization_msgs::Marker::Ptr m(new visualization_msgs::Marker);
        //visualization::savedFeatureToSphereLegMarkerMsg((*sf_iter), m, fixed_frame, i);
        markers_pub_.publish(m);

        // Publish the cluster properties as text
        visualization_msgs::Marker::Ptr m_text(new visualization_msgs::Marker);
        //visualization::clusterToTextMarkerMsg((*sf_iter), m_text, fixed_frame, i);
        markers_pub_.publish(m_text);
      }

      if (publish_people_ || publish_people_markers_)
      {
        SavedFeature* other = (*sf_iter)->other;

        // If pairs
        if (other != NULL && other < (*sf_iter))
        {
          SavedFeature* leg1;
          SavedFeature* leg2;

          leg1 = (*sf_iter);
          leg2 = (*sf_iter)->other;

          tf::Vector3 peoplePos = (leg1->position_ + leg2->position_ ) / 2.0;

          if (publish_people_)
          {
            reliability = reliability * other->reliability;
            cob_perception_msgs::PositionMeasurement pos;
            pos.header.stamp = (*sf_iter)->time_;
            pos.header.frame_id = fixed_frame;
            pos.name = (*sf_iter)->object_id;
            pos.object_id = (*sf_iter)->id_ + "|" + other->id_;
            pos.pos.x = peoplePos[0];
            pos.pos.y = peoplePos[1];
            pos.pos.z = peoplePos[2];
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
            people.push_back(pos);
          }

          if (publish_people_markers_)
          {
            visualization_msgs::Marker::Ptr pPeopleSphereMsg(new visualization_msgs::Marker);
            visualization_msgs::Marker::Ptr plegLineMsg(new visualization_msgs::Marker);
            //visualization::savedFeatureToPeopleMarkerMsg(leg1, leg2, pPeopleSphereMsg, plegLineMsg, fixed_frame, peoplePos, i);
            markers_pub_.publish(pPeopleSphereMsg);
            markers_pub_.publish(plegLineMsg);

          }
        }
      }
    }

    // Publish the clusters in the fixed_frame
    if(publish_clusters_){
        // Visualize the clusters by creating a pointcloud, each cluster has the same color
        ROS_DEBUG_COND(LEGDETECTOR_DEBUG,"Publishing Clusters!");
        sensor_msgs::PointCloud clusters;
        sensor_msgs::ChannelFloat32 rgb_channel;
        rgb_channel.name="rgb";
        clusters.channels.push_back(rgb_channel);
        clusters.header = scan->header;

        //clusters->channels.a
        int count = 0;
        for (list<SampleSet*>::iterator i = processor.getClusters().begin();
             i != processor.getClusters().end();
             i++)
        {

            int r[3] = { 0, 125, 255};
            int g[3] = { 0, 125, 255};
            int b[3] = { 0, 125, 255};

            int r_ind = count % 3;
            int g_ind = (count/3) % 3;
            int b_ind = (count/9) % 3;
            count++;

            (*i)->appendToCloud(clusters,r[r_ind],g[g_ind],b[b_ind]);

            visualization_msgs::Marker m_text;
            m_text.header = clusters.header;
            m_text.ns = "CLUSTERS_LABEL";
            m_text.id = count;
            m_text.type = m_text.TEXT_VIEW_FACING;
            m_text.pose.position.x = (*i)->center()[0]+0.15;
            m_text.pose.position.y = (*i)->center()[1]+0.15;
            m_text.pose.position.z = (*i)->center()[2];
            m_text.scale.z = .15;
            m_text.color.a = 1;
            m_text.lifetime = ros::Duration(0.5);

            // Add text
            char buf[100];
            m_text.color.r = r[r_ind]/255.0;
            m_text.color.g = g[g_ind]/255.0;
            m_text.color.b = b[b_ind]/255.0;
            sprintf(buf, "#%d",count);

            m_text.text = buf;

            markers_pub_.publish(m_text);
        }

        clusters_pub_.publish(clusters);
    }



    cob_perception_msgs::PositionMeasurementArray array;
    array.header.stamp =  scan->header.stamp;
    array.header.frame_id = scan->header.frame_id;
    if (publish_legs_)
    {
      array.people = legs;
      leg_measurements_pub_.publish(array);
      ROS_DEBUG("Publishing legs positions on %s", array.header.frame_id.c_str());
    }
    if (publish_people_)
    {
      array.people = people;
      people_measurements_pub_.publish(array);

    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_detector");
  g_argc = argc;
  g_argv = argv;
  ros::NodeHandle nh;
  LegDetector ld(nh);
  ros::spin();

  return 0;
}

