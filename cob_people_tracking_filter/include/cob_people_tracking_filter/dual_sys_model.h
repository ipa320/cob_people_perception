#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_SYS_MODEL_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_SYS_MODEL_H_

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


#include "state_pos_vel.h"
#include "gaussian_pos_vel.h"
#include <model/systemmodel.h>
#include <pdf/conditionalpdf.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <string>

namespace BFL
{

class DualSysPdfPosVel
  : public ConditionalPdf<StatePosVel, StatePosVel>
{
public:

  /// Constructor
  DualSysPdfPosVel(const StatePosVel& sigma);

  /// Destructor
  virtual ~DualSysPdfPosVel();

  // set time
  void SetDt(double dt)
  {
    dt_ = dt;
  };

  // Redefining pure virtual methods
  virtual bool SampleFrom(BFL::Sample<StatePosVel>& one_sample, int method, void *args) const;
  virtual StatePosVel ExpectedValueGet() const; // not applicable
  virtual Probability ProbabilityGet(const StatePosVel& state) const; // not applicable
  virtual MatrixWrapper::SymmetricMatrix  CovarianceGet() const; // Not applicable


private:
  GaussianPosVel noise_;
  double dt_;

}; // class




class DualSysModelPosVel
  : public SystemModel<StatePosVel>
{
public:
    DualSysModelPosVel(const StatePosVel& sigma)
    : SystemModel<StatePosVel>(new SysPdfPosVel(sigma))
  {};

  /// destructor
  ~DualSysModelPosVel()
  {
    delete SystemPdfGet();
  };

  // set time
  void SetDt(double dt)
  {
    ((SysPdfPosVel*)SystemPdfGet())->SetDt(dt);
  };

}; // class



} //namespace

#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_SYS_MODEL_H_ */
