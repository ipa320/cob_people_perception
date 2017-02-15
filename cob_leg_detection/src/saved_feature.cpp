/*
 * features.cpp
 *
 *  Created on: Apr 15, 2015
 *      Author: frm-ag
 */


// Own includes
#include <cob_leg_detection/saved_feature.h>

int SavedFeature::nextid = 0;

static std::string fixed_frame = "odom_combined";  // The fixed frame in which ? //TODO find out

static double kal_p = 4, kal_q = .002, kal_r = 10;
static bool use_filter = false;

// one leg tracker
SavedFeature::SavedFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl)
  : tfl_(tfl),
    sys_sigma_(tf::Vector3(0.05, 0.05, 0.05), tf::Vector3(1.0, 1.0, 1.0)),
    filter_("tracker_name", sys_sigma_),
    reliability(-1.), p(4),
    use_filter_(true)
{
  int_id_ = nextid++;

  char id[100];
  snprintf(id, 100, "legtrack%d", int_id_);
  id_ = std::string(id);

  ROS_DEBUG_COND(DEBUG_SAVED_FEATURE,"SavedFeature::%s Created new SavedFeature with id: %s", __func__, id_.c_str());

  object_id = "";
  time_ = loc.stamp_;
  meas_time_ = loc.stamp_;
  other = NULL;

  try
  {
    tfl_.transformPoint(fixed_frame, loc, loc);
  }
  catch (...)
  {
    ROS_WARN("TF exception spot 6.");
  }
  tf::StampedTransform pose(tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
  tfl_.setTransform(pose);

  BFL::StatePosVel prior_sigma(tf::Vector3(0.1, 0.1, 0.1), tf::Vector3(0.0000001, 0.0000001, 0.0000001));
  filter_.initialize(loc, prior_sigma, time_.toSec());

  BFL::StatePosVel est;
  filter_.getEstimate(est);

  updatePosition();
}

/**
 * Propagate the Position of the Feature using the Kalman filter
 */
void SavedFeature::propagate(ros::Time time)
{
  time_ = time;

  filter_.updatePrediction(time.toSec());

  updatePosition();
}

void SavedFeature::update(tf::Stamped<tf::Point> loc, double probability)
{
  tf::StampedTransform pose(tf::Pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), loc), loc.stamp_, id_, loc.frame_id_);
  tfl_.setTransform(pose);

  meas_time_ = loc.stamp_;
  time_ = meas_time_;

  MatrixWrapper::SymmetricMatrix cov(3);
  cov = 0.0;
  cov(1, 1) = 0.0025;
  cov(2, 2) = 0.0025;
  cov(3, 3) = 0.0025;

  filter_.updateCorrection(loc, cov);

  updatePosition();

  if (reliability < 0 || !use_filter_)
  {
    reliability = probability;
    p = kal_p;
  }
  else
  {
    p += kal_q;
    double k = p / (p + kal_r);
    reliability += k * (probability - reliability);
    p *= (1 - k);
  }
}

void SavedFeature::updatePosition()
{
  BFL::StatePosVel est;
  filter_.getEstimate(est);

  position_[0] = est.pos_[0];
  position_[1] = est.pos_[1];
  position_[2] = est.pos_[2];
  position_.stamp_ = time_;
  position_.frame_id_ = fixed_frame_;
  double nreliability = fmin(1.0, fmax(0.1, est.vel_.length() / 0.5));
  //reliability = fmax(reliability, nreliability);
}

