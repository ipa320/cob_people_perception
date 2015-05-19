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

/* Author: Wim Meeussen */

#ifndef ADVANCED_SYSMODEL_POS_VEL_H
#define ADVANCED_SYSMODEL_POS_VEL_H


#include "state_pos_vel.h"
#include <people_tracking_filter/gaussian_pos_vel.h>
#include <people_tracking_filter/multivariate_gaussian_pos_vel.h>
#include <model/systemmodel.h>
#include <pdf/conditionalpdf.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <string>

#define DEBUG_ADVANCEDSYSMODELPOSVEL 1

namespace BFL
{

class AdvancedSysPdfPosVel
  : public ConditionalPdf<StatePosVel, StatePosVel>
{
public:

  /// Constructor
  AdvancedSysPdfPosVel(const StatePosVel& sigma);

  /// Destructor
  virtual ~AdvancedSysPdfPosVel();

  // set covariance
  void CovarianceSet(const  MatrixWrapper::SymmetricMatrix& cov);

  // Set the multivariate Covariance
  void MultivariateCovarianceSet(const MatrixWrapper::SymmetricMatrix& cov);

  void HighLevelInformationSet(tf::Vector3 vel, tf::Vector3 hipVec);

  // set time
  void SetDt(double dt)
  {
    dt_ = dt;
  };

  // get the time (for debugging)
  double getDt()
  {
    return dt_;
  };

  // Set wether high level prediction should be used
  void setUseHighlevelPrediction(bool useHighLevelPrediction){
    this->useHighLevelPrediction_ = useHighLevelPrediction;
  }

  // Redefining pure virtual methodsw
  virtual bool SampleFrom(BFL::Sample<StatePosVel>& one_sample, int method, void *args) const;
  virtual StatePosVel ExpectedValueGet() const; // not applicable
  virtual Probability ProbabilityGet(const StatePosVel& state) const; // not applicable
  virtual MatrixWrapper::SymmetricMatrix  CovarianceGet() const; // Not applicable


private:
  GaussianPosVel noise_;
  MultivariateGaussianPosVel noise_nl_;
  double dt_;

  bool useHighLevelPrediction_; /**< Update using the high level people tracker prediction */

}; // class



class AdvancedSysModelPosVel
  : public SystemModel<StatePosVel>
{
public:
    /**
     * Constructor of the system Model
     * @param sigma The system noise
     */
    AdvancedSysModelPosVel(const StatePosVel& sigma)
    : SystemModel<StatePosVel>(new AdvancedSysPdfPosVel(sigma))
  {};

  /// destructor
  ~AdvancedSysModelPosVel()
  {
    delete SystemPdfGet();
  };

  // set time
  void SetDt(double dt)
  {
    ROS_DEBUG_COND(DEBUG_ADVANCEDSYSMODELPOSVEL,"------AdvancedSysModelPosVel::%s",__func__);
    ((AdvancedSysPdfPosVel*)SystemPdfGet())->SetDt(dt);
  };

}; // class



} //namespace


#endif
