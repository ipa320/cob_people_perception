/*
 * people_particle_filter.cpp
 *
 *  Created on: Apr 21, 2015
 *      Author: frm-ag
 */

// Copyright (C) 2003 Klaas Gadeyne <first dot last at gmail dot com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
// $Id: bootstrapfilter.cpp 29495 2008-08-13 12:57:49Z tdelaet $

// ROS includes
#include <ros/console.h>

//BFL includes
#include <filter/particlefilter.h>

// Own includes
#include <people_tracking_filter/people_particle_filter.h>
#include <sample/weightedsample.h>


#define StateVar SVar
#define MeasVar MVar

using namespace BFL;


PeopleParticleFilter::PeopleParticleFilter(MCPdf<StatePosVel> * prior,
              MCPdf<StatePosVel> * post,
              int resampleperiod,
              double resamplethreshold,
              int resamplescheme)
  : BFL::ParticleFilter<StatePosVel,tf::Vector3>(prior,post,NULL,resampleperiod,
             resamplethreshold,
             resamplescheme)
{
  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  // for a bootstrapfilter, the proposal does not depend on the
  // measurement
  this->_proposal_depends_on_meas = false;
}

PeopleParticleFilter::~PeopleParticleFilter(){}

bool
PeopleParticleFilter::UpdateInternal(BFL::AdvancedSysModelPosVel* const sysmodel,
             const StatePosVel& u,
             BFL::MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
             const tf::Vector3& z,
             const StatePosVel& s)
{
  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  bool result = true;

  std::vector<WeightedSample<StatePosVel> > samples;

  ////////////////////////////////////
  //// System Update (Prediction)
  ////////////////////////////////////

  // Update using the system model
  if (sysmodel != NULL){
    ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s -> System Model Update",__func__);

    // Proposal is the same as the SystemPdf
    this->ProposalSet(sysmodel->SystemPdfGet());

    // Check this before next
    assert(this->_proposal != NULL);
    assert(this->_post != NULL);

    samples = ((MCPdf<StatePosVel> *) this->_post)->ListOfSamplesGet();

    for(std::vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
      StatePosVel sample = (*sampleIt).ValueGet();
      double weight = (*sampleIt).WeightGet();
      //std::cout << "Sample " << sample << "Weight: " << weight << std::endl;
    }

    result = this->ParticleFilter<StatePosVel,tf::Vector3>::UpdateInternal(sysmodel,u,NULL,z,s) && result;

//    std::cout << "Update ###############################" << std::endl;
//    std::cout << "Update ###############################" << std::endl;
//    std::cout << "Delta T: " << ((AdvancedSysPdfPosVel*) sysmodel->SystemPdfGet())->getDt() << std::endl;


    samples = ((MCPdf<StatePosVel> *) this->_post)->ListOfSamplesGet();

    for(std::vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
      //std::cout << (*sampleIt) << std::endl;
    }

    ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s -> Internal Update done",__func__);
    //result = this->ParticleFilter<SVar,MVar>::UpdateInternal(sysmodel,u,NULL,z,s) && result;

  }

  ////////////////////////////////////
  //// Measurement Update
  ////////////////////////////////////

  if (measmodel != NULL){
    ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s -> Measurement Update with %f %f %f",__func__, z.getX(), z.getY(), z.getZ());

    result = this->ParticleFilter<StatePosVel,tf::Vector3>::UpdateInternal(NULL,u,measmodel,z,s) && result;

    samples = ((MCPdf<StatePosVel> *) this->_post)->ListOfSamplesGet();
    for(std::vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
      std::cout << (*sampleIt) << std::endl;
    }

    assert(false);

  }

  return result;
}




