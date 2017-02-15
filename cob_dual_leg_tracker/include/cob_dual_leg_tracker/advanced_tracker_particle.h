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

/* Based on the work of Author: Wim Meeussen */

#ifndef __ADVANCED_TRACKER_PARTICLE__
#define __ADVANCED_TRACKER_PARTICLE__


// ROS includes
#include <tf/tf.h>

// ROS Messages
#include <sensor_msgs/PointCloud.h>

// BFL includes
#include <filter/bootstrapfilter.h>

// People Stack includes
#include <cob_people_tracking_filter/tracker.h>
#include <cob_people_tracking_filter/state_pos_vel.h>
#include <cob_people_tracking_filter/mcpdf_pos_vel.h>

// Own includes
#include <cob_dual_leg_tracker/people_particle_filter.h>
#include <cob_dual_leg_tracker/models/advanced_sysmodel_pos_vel.h>
#include <cob_dual_leg_tracker/models/advanced_measmodel_pos.h>
#include <cob_dual_leg_tracker/models/occlusion_model.h>

namespace estimation
{

class AdvancedTrackerParticle: public Tracker
{
public:
  /// constructor
    AdvancedTrackerParticle(const std::string& name,
                            unsigned int num_particles,
                            const BFL::StatePosVel& sysnoise,
                            double v_max,
                            double position_factor,
                            double velocity_factor);

  /// destructor
  virtual ~AdvancedTrackerParticle();

  /// initialize tracker
  virtual void initialize(const BFL::StatePosVel& mu, const BFL::StatePosVel& sigma, const double time);

  /// return if tracker was initialized
  virtual bool isInitialized() const
  {
    return tracker_initialized_;
  };

  /// return measure for tracker quality: 0=bad 1=good
  virtual double getQuality() const
  {
    return quality_;
  };

  void setOcclusionModel(OcclusionModelPtr ocm){
    occlusion_model_ = ocm;
  }

  /// return the lifetime of the tracker
  virtual double getLifetime() const;

  /// return the time of the tracker
  virtual double getTime() const;

  /// update tracker
  virtual bool updatePrediction(const double time);
  virtual bool updatePrediction(const double time, const MatrixWrapper::SymmetricMatrix& cov); /**< Update without high level prediction */
  virtual bool updatePrediction(const double time, const MatrixWrapper::SymmetricMatrix& cov, double gaitFactor, tf::Vector3 velVec, tf::Vector3 hipVec, double associationProbability); /**< Update with high level prediction */


  //virtual bool updatePrediction(const double time, StatePosVel highLevelPrediction);
  virtual bool updateCorrection(const tf::Vector3& meas,
                                const MatrixWrapper::SymmetricMatrix& cov);

  bool updateJPDA(const MatrixWrapper::SymmetricMatrix& cov, const std::vector<DetectionPtr>& detections, Eigen::VectorXd& assignmentProbabilities, OcclusionModelPtr occlusionModel);

  bool dynamicResample();

  /// Get probability for a certain measurement
  double getMeasProbability(const tf::Vector3&  meas,
                            const MatrixWrapper::SymmetricMatrix& cov);

  /// Calculate the occlusion probability
  double getOcclusionProbability(OcclusionModelPtr occlusionModel);

  /// get filter posterior
  virtual void getEstimate(BFL::StatePosVel& est) const;
  virtual void getEstimate(cob_perception_msgs::PositionMeasurement& est) const;

  // get evenly spaced particle cloud
  void getParticleCloud(const tf::Vector3& step, double threshold, sensor_msgs::PointCloud& cloud) const;

  /// Get histogram from certain area
  MatrixWrapper::Matrix getHistogramPos(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const;
  MatrixWrapper::Matrix getHistogramVel(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const;

  MCPdf<StatePosVel>* postGet() const{
    return this->getFilter()->PostGet();
  }


private:
  // pdf / model / filter
  BFL::MCPdfPosVel                                          prior_; // The particles pdf function
  //BFL::BootstrapFilter<BFL::StatePosVel, tf::Vector3>*      filter_;
  PeopleParticleFilter*      filter_; /**< The particle filter */

  // Models
  BFL::AdvancedSysModelPosVel                               sys_model_; /**< The system model */
  BFL::AdvancedMeasModelPos                                 meas_model_;/**< The measurement model */
  OcclusionModelPtr                                         occlusion_model_;/**< The occlusion model */

  // vars
  bool tracker_initialized_;
  double init_time_; /**< The initialization time of the Tracker */
  double filter_time_; /**< The last filter time */
  double quality_;
  unsigned int num_particles_;

  //// Get the filter
  PeopleParticleFilter* getFilter() const{
    return filter_;
  }

}; // class

}; // namespace

#endif
