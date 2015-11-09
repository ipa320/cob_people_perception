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

#include <dual_people_leg_tracker/distributions/multivariate_gaussian_pos_vel.h>
#include <wrappers/rng/rng.h>
#include <cmath>
#include <cassert>


using namespace tf;

namespace BFL
{

/// Constructor
MultivariateGaussianPosVel::MultivariateGaussianPosVel():
  sigma_(Eigen::Matrix<double,6,6>::Zero()),
  mu_(Eigen::Matrix<double,6,1>::Zero()),
  eigv1_(Eigen::Matrix<double,3,1>::Zero()),
  eigv2_(Eigen::Matrix<double,3,1>::Zero()),
  sigma_changed_(false),
  dt_(0.0),
  sqrt_(0.0),
  highLevelProbability_(0.0)
{
  //assert(false);
  //normX_solver_ = boost::shared_ptr<Eigen::EigenMultivariateNormal<double> >( new Eigen::EigenMultivariateNormal<double>(mu_,sigma_));
  // Initialize sigma with zeros
  //sigma_ =
  //mu_ = Eigen::Matrix<double,6,1>::Zero();
}

MultivariateGaussianPosVel::MultivariateGaussianPosVel(const Eigen::Matrix<double,6,1>& mu, const Eigen::Matrix<double,6,6>& sigma)
  : Pdf<StatePosVel> (1),
    mu_(mu),
    sigma_(sigma),
    sigma_changed_(false),
    dt_(0.0),
    sqrt_(0.0),
    highLevelProbability_(0.0),
    gaitFactor_(0.0)
    //gauss_pos_(mu.pos_, sigma.pos_),
    //gauss_vel_(mu.vel_, sigma.vel_)
{
  normX_solver_ = boost::shared_ptr<Eigen::EigenMultivariateNormal<double> >( new Eigen::EigenMultivariateNormal<double>(mu_,sigma_));
}


MultivariateGaussianPosVel::~MultivariateGaussianPosVel() {}

MultivariateGaussianPosVel* MultivariateGaussianPosVel::Clone() const
{
  return new MultivariateGaussianPosVel(mu_, sigma_);
}

std::ostream& operator<< (std::ostream& os, const MultivariateGaussianPosVel& g)
{
//  os << "\nMu pos :\n"    << g.ExpectedValueGet().pos_ << endl
//     << "\nMu vel :\n"    << g.ExpectedValueGet().vel_ << endl
//     << "\nSigma:\n" << g.CovarianceGet() << endl;
//  return os;
}

void MultivariateGaussianPosVel::sigmaSet(const Eigen::Matrix<double,6,6>& sigma)
{

  sigma_ = sigma;
  sigma_changed_ = true;

  normX_solver_->setCovar(sigma_);

  assert(false); // Never been here! Carefull!
}

void MultivariateGaussianPosVel::sigmaSet(const MatrixWrapper::SymmetricMatrix& cov)
{
  // Transform the sigma into a eigen Matrix

  for (unsigned int i = 0; i < 6; i++){
    for (unsigned int j = 0; j < 6; j++){
      sigma_(i,j) = cov(i + 1, j + 1);
    }
  }

  sigma_changed_ = true;

  normX_solver_->setCovar(sigma_);

}

void MultivariateGaussianPosVel::eigenvectorsSet(tf::Vector3 eigv1, tf::Vector3 eigv2)
{
  eigv1_(0) = eigv1.getX();
  eigv1_(1) = eigv1.getY();
  eigv1_(2) = eigv1.getZ();

  eigv2_(0) = eigv2.getX();
  eigv2_(1) = eigv2.getY();
  eigv2_(2) = eigv1.getZ();

}

void MultivariateGaussianPosVel::setGaitFactor(double gaitFactor){
  gaitFactor_ = gaitFactor;
}

void MultivariateGaussianPosVel::highLevelProbabilitySet(double highLevelProbability){
  highLevelProbability_ = highLevelProbability;
}


Probability MultivariateGaussianPosVel::ProbabilityGet(const StatePosVel& input) const
{
  if (sigma_changed_)
  {
    sigma_changed_ = false;

    sqrt_ = 1 / sqrt(pow(2*M_PI,6) * sigma_.determinant());
  }

  // Convert to Eigen
  Eigen::Matrix<double,6,1> in_eig;
  in_eig[0] = input.pos_.getX();// << input.pos_.getY() << input.pos_.getZ() << input.vel_.getX() << input.vel_.getY() << input.vel_.getZ();

  // Calculate and return the probability
  double exponent = -0.5 * ((in_eig-mu_).transpose() * sigma_.inverse() * (in_eig-mu_))(0);

  assert(false);

  return sqrt_ * exp(exponent);

}


bool
MultivariateGaussianPosVel::SampleFrom(vector<Sample<StatePosVel> >& list_samples, const int num_samples, int method, void * args) const
{
  assert(false);

  Eigen::EigenMultivariateNormal<double> normX_solver(this->mu_,this->sigma_);
  std::cout << normX_solver.samples(500).transpose() << std::endl;

  assert(false); // NOT TESTED

  list_samples.resize(num_samples);
  vector<Sample<StatePosVel> >::iterator sample_it = list_samples.begin();
  for (sample_it = list_samples.begin(); sample_it != list_samples.end(); sample_it++)
    SampleFrom(*sample_it, method, args);

  return true;
}


bool
MultivariateGaussianPosVel::SampleFrom(Sample<StatePosVel>& one_sample, int method, void * args) const
{
  // EigV1 is the velocity in direction of movement
  // EigV2 is the hip Vector
  // The gaitFactor corresponds to the distance towards a endpoint

  double alpha    = eigv1_.norm(); // Influence of the leg velocity
  double alpha_mu = eigv1_.norm() * gaitFactor_; // Mu of the Distribution
  double beta = 1;  // Spread in direction of the hip vector

  Eigen::Matrix<double,3,1> sample_pos = Eigen::Matrix<double,3,1>::Zero();
  Eigen::Matrix<double,3,1> sample_vel = Eigen::Matrix<double,3,1>::Zero();

  Eigen::Matrix<double,3,1> vel_norm = eigv1_;
  vel_norm.normalize();

  ROS_ASSERT(alpha >= 0);

  // Generate sample for the velocity
  double vel_rand = rnorm(alpha_mu,alpha);


  double width_rand = rnorm(0,abs(vel_rand) * beta);

  //std::cout << "vel_rand:" << vel_rand << " width_rand: " << width_rand << std::endl;
  //std::cout << "velocity is changed by factor" << vel_rand << std::endl;


  //sample_pos = eigv1_ * vel_rand * dt_ * 1.5 + eigv2_ * width_rand * dt_ * 2.3;


  sample_pos = eigv1_ * vel_rand * dt_ * 0.8 + eigv2_ * width_rand * dt_ * 0.8;
  sample_vel = eigv1_ * vel_rand * 1.6 + eigv2_ * width_rand * 1.6;


  //sample_pos = eigv1_ * vel_rand * dt_ * 5 + eigv2_ * width_rand * dt_ * 5;
  //std::cout << "dt_" << dt_ << std::endl;


  //std::cout << "vel_rand: " << vel_rand << " width_rand: " << width_rand << std::endl;

//  std::cout << "eigv1_" << std::endl << eigv1_ << std::endl;
//  std::cout << "eigv2_" << std::endl << eigv2_ << std::endl;
//  std::cout << "samples_pos" << std::endl << sample_pos << std::endl;
//  std::cout << "samples_vel" << std::endl << sample_vel << std::endl;

  //sample = normX_solver_->samples(1);

  one_sample.ValueSet(StatePosVel(Vector3(sample_pos[0],
                                          sample_pos[1],
                                          0),
                                  Vector3(sample_vel[0],
                                          sample_vel[1],
                                		      0))); // TODO make this better

  return true;
}


StatePosVel
MultivariateGaussianPosVel::ExpectedValueGet() const
{
  StatePosVel expectedValue;
  //expectedValue.pos_.setX(mu_(0,0));

  assert(false);

  return expectedValue;
}

SymmetricMatrix
MultivariateGaussianPosVel::CovarianceGet() const
{
  SymmetricMatrix sigma(6);

  sigma = 0;
  for (unsigned int i = 0; i < 6; i++){
    for (unsigned int j = 0; j < 6; j++){
      sigma(i + 1, j + 1) = sigma_(i,j);
    }
  }
  assert(false); // Test this!
  return sigma;

  //return this->sigma_;
}


} // End namespace BFL
