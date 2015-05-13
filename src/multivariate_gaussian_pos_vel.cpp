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


#include <people_tracking_filter/multivariate_gaussian_pos_vel.h>
#include <wrappers/rng/rng.h>
#include <cmath>
#include <cassert>


using namespace tf;

namespace BFL
{
MultivariateGaussianPosVel::MultivariateGaussianPosVel(const Eigen::Matrix<double,6,1>& mu, const Eigen::Matrix<double,6,6>& sigma)
  : Pdf<StatePosVel> (1),
    mu_(mu),
    sigma_(sigma)
    //gauss_pos_(mu.pos_, sigma.pos_),
    //gauss_vel_(mu.vel_, sigma.vel_)
{

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

  //assert(false); // Never been here! Carefull!
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
  return sqrt_ * exp(exponent);

}


bool
MultivariateGaussianPosVel::SampleFrom(vector<Sample<StatePosVel> >& list_samples, const int num_samples, int method, void * args) const
{
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
  assert(false); // NOT YET IMPLEMENTED
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
    for (unsigned int j = 0; j < 6; i++){
      sigma(i + 1, j + 1) = sigma_(i,j);
    }
  }
  assert(false); // Test this!
  return sigma;

  //return this->sigma_;
}


} // End namespace BFL
