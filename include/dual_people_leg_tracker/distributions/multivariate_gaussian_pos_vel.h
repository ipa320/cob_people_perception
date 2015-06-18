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


#ifndef MULTIVARIATE_GAUSSIAN_POS_VEL_H
#define MULTIVARIATE_GAUSSIAN_POS_VEL_H

#include <pdf/pdf.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/gaussian_vector.h>

// Own includes
#include <dual_people_leg_tracker/eigenmvn/eigenmvn.h>

#define DEBUG_MULTIVARIATEGAUSSIANPOSVEL 1

namespace BFL
{
/// Class representing gaussian pos_vel
class MultivariateGaussianPosVel: public Pdf<StatePosVel>
{
public:
    Eigen::Matrix<double,6,6> sigma_; /**< Covariance */
    Eigen::Matrix<double,6,1> mu_;  /**< Mean */

    // Eigenvalues
    Eigen::Matrix<double,3,1> eigv1_;
    Eigen::Matrix<double,3,1> eigv2_;

    // High level probability
    double highLevelProbability_;

    // The Random Generator
    boost::shared_ptr<Eigen::EigenMultivariateNormal<double> > normX_solver_;

private:

  mutable double sqrt_; /**< Nominator of the density function, precalculated and stored for faster calculation */

  //GaussianVector gauss_pos_, gauss_vel_;
  mutable double dt_;
  mutable bool sigma_changed_;

public:
  /// Constructor
  MultivariateGaussianPosVel();

  /// Constructor
  MultivariateGaussianPosVel(const Eigen::Matrix<double,6,1>& mu, const Eigen::Matrix<double,6,6>& sigma);

  /// Destructor
  virtual ~MultivariateGaussianPosVel();

  /// clone function
  virtual MultivariateGaussianPosVel* Clone() const;

  /// output stream for GaussianPosVel
  friend std::ostream& operator<< (std::ostream& os, const MultivariateGaussianPosVel& g);

  /// Set the covar (as Eigen)
  void sigmaSet(const Eigen::Matrix<double,6,6>& sigma);

  /// Set the covar (as SymmetricMatrix)
  void sigmaSet(const MatrixWrapper::SymmetricMatrix& cov);

  void eigenvectorsSet(tf::Vector3 eigv1, tf::Vector3 eigv2);

  /**
   * Set the probability of the high level filter
   * @param highLevelProbability
   */
  void highLevelProbabilitySet(double highLevelProbability);

  Eigen::Matrix<double,6,1>
  getMu() const{
    return this->mu_;
  }

  const Eigen::Matrix<double,6,6>
  getSigma(){
    return sigma_;
  }

  // set time
  void SetDt(double dt) const
  {
    dt_ = dt;
  };

  // Redefinition of pure virtuals
  virtual Probability ProbabilityGet(const StatePosVel& input) const;
  bool SampleFrom(vector<Sample<StatePosVel> >& list_samples, const int num_samples, int method = DEFAULT, void * args = NULL) const;
  virtual bool SampleFrom(Sample<StatePosVel>& one_sample, int method = DEFAULT, void * args = NULL) const;

  virtual StatePosVel ExpectedValueGet() const;
  virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;

};

} // end namespace
#endif
