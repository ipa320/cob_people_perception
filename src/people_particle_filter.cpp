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
#include <dual_people_leg_tracker/people_particle_filter.h>
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

/**
 * This is the main Update function of the PeopleParticleFilter
 * @param sysmodel
 * @param u
 * @param measmodel
 * @param z
 * @param s
 * @return
 */
bool
PeopleParticleFilter::UpdateInternal(BFL::AdvancedSysModelPosVel* const sysmodel,
             const StatePosVel& u,
             BFL::MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
             const tf::Vector3& z,
             const StatePosVel& s,
             OcclusionModelPtr occlusionmodel)
{
  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  bool result = true;

  //std::vector<WeightedSample<StatePosVel> > samples;

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

/*    samples = ((MCPdf<StatePosVel> *) this->_post)->ListOfSamplesGet();

    for(std::vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
      StatePosVel sample = (*sampleIt).ValueGet();
      double weight = (*sampleIt).WeightGet();
      //std::cout << "Sample " << sample << "Weight: " << weight << std::endl;
    }*/

    assert(this->_dynamicResampling == true); // TODO
    //result = result && this->StaticResampleStep(); // TODO necessary?

    result = result && this->ProposalStepInternal(sysmodel,u,measmodel,z,s);

    //result = this->ParticleFilter<StatePosVel,tf::Vector3>::UpdateInternal(sysmodel,u,NULL,z,s) && result;

//    std::cout << "Update ###############################" << std::endl;
//    std::cout << "Update ###############################" << std::endl;
//    std::cout << "Delta T: " << ((AdvancedSysPdfPosVel*) sysmodel->SystemPdfGet())->getDt() << std::endl;


/*    samples = ((MCPdf<StatePosVel> *) this->_post)->ListOfSamplesGet();

    for(std::vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
      //std::cout << (*sampleIt) << std::endl;
    }*/

    ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s -> Internal Update done",__func__);
    //result = this->ParticleFilter<SVar,MVar>::UpdateInternal(sysmodel,u,NULL,z,s) && result;

  }

  ////////////////////////////////////
  //// Measurement Update
  ////////////////////////////////////

  if (measmodel != NULL){
    ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s -> Measurement Update with %f %f %f",__func__, z.getX(), z.getY(), z.getZ());

    //result = this->ParticleFilter<StatePosVel,tf::Vector3>::UpdateInternal(NULL,u,measmodel,z,s) && result;

    result = result && this->UpdateWeightsInternal(sysmodel,u,measmodel,z,s);

    // If a occlusion model is set
    if(occlusionmodel){
      //result = result && this->UpdateWeightsUsingOcclusionModel(occlusionmodel);
    }
    //result = result && this->ParticleFilter<StatePosVel,tf::Vector3>::DynamicResampleStep();
    result = result && this->DynamicResampleStep();

/*    samples = ((MCPdf<StatePosVel> *) this->_post)->ListOfSamplesGet();
    for(std::vector<WeightedSample<StatePosVel> >::iterator sampleIt = samples.begin(); sampleIt != samples.end(); sampleIt++){
      std::cout << (*sampleIt) << std::endl;
    }*/

  }

  //assert(false);

  return result;
}

bool
PeopleParticleFilter::UpdateWeightsInternal(BFL::AdvancedSysModelPosVel* const sysmodel,
             const StatePosVel& u,
             MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
             const tf::Vector3& z,
             const StatePosVel& s){

  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  assert(sysmodel == NULL);

  Probability weightfactor = 1;

  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();
  _os_it = _old_samples.begin();

  // Iterate through the samples
  for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++){

    const StatePosVel& x_new = _ns_it->ValueGet();
    //const StatePosVel& x_old = _os_it->ValueGet();

    weightfactor = measmodel->ProbabilityGet(z,x_new);
    // TODO apply occlusion model here

    //std::cout << "Weight Update: " << _ns_it->WeightGet() << "    -->     ";
    _ns_it->WeightSet(_ns_it->WeightGet() * weightfactor);
    //std::cout << _ns_it->WeightGet() << std::endl;
  }

  return (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);
}

bool
PeopleParticleFilter::UpdateWeightsUsingOcclusionModel(OcclusionModelPtr occlusionmodel){
  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  //assert(false);

  double weightfactor = 1.0;

  unsigned int counter = 0;


  tf::Stamped<tf::Point> point;
  point.stamp_ = occlusionmodel->scan_.header.stamp; // TODO ugly!
  point.frame_id_ = occlusionmodel->scan_.header.frame_id;

  // Get a list of samples
  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();

  // Iterate through the samples
  for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++){

    // Get the value of this sample
    const StatePosVel& x_new = _ns_it->ValueGet();
    //const StatePosVel& x_old = _os_it->ValueGet();

    point.setData(x_new.pos_);

    weightfactor = occlusionmodel->getOcclusionProbability(point);
    // TODO apply occlusion model here
    weightfactor = min(0.8, weightfactor);
    weightfactor = 0.0;


    if(weightfactor != 1.0){
      std::cout << "Update using occlusion model weightfactor: " << weightfactor << "  " << _ns_it->WeightGet() << "  -->  ";
    }
    //std::cout << "Weight Update: " << _ns_it->WeightGet() << "    -->     ";
    _ns_it->WeightSet(_ns_it->WeightGet() * (1-weightfactor));
    //std::cout << _ns_it->WeightGet() << std::endl;

    if(weightfactor != 1.0){
      std::cout << _ns_it->WeightGet() << std::endl;
    }

    counter++;
  }

  return (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);
}

bool
PeopleParticleFilter::DynamicResampleStep()
{

  // Resampling?
  bool resampling = false;
  double sum_sq_weigths = 0.0;

  // Resampling if necessary
  if ( this->_dynamicResampling)
  {
    // Check if sum of 1 / \sum{(wi_normalised)^2} < threshold
    // This is the criterion proposed by Liu
    // BUG  foresee other methods of approximation/calculating
    // effective sample size.  Let the user implement this in !
    _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();
    for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++)
      {
        sum_sq_weigths += pow(_ns_it->WeightGet(),2);
      }
    if ((1.0 / sum_sq_weigths) < _resampleThreshold)
      {
        resampling = true;
      }
  }
    if (resampling == true)
      return this->Resample();
    else
      return true;
}

/**
 * This is the resample Function copied from the BFL resampling of the particle filter class.
 * This is done the gain better access to this function without having to change the library.
 * @return
 */
bool
PeopleParticleFilter::Resample()
{

  int NumSamples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->NumSamplesGet();

  // Note: At this time only one sampling method is implemented!
  switch(_resampleScheme)
    {
    case MULTINOMIAL_RS:
      {
        (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->SampleFrom(_new_samples_unweighted, NumSamples,RIPLEY,NULL);
        break;
      }
    case SYSTEMATIC_RS:{break;}
    case STRATIFIED_RS:{break;}
    case RESIDUAL_RS:{break;}
    default:
      {
        cerr << "Sampling method not supported" << endl;
        break;
      }
    }
  bool result = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples_unweighted);

  return result;
}


bool
PeopleParticleFilter::ProposalStepInternal(SystemModel<StatePosVel> * const sysmodel,
              const StatePosVel & u,
              MeasurementModel<tf::Vector3,StatePosVel> * const measmodel,
              const tf::Vector3 & z,
              const StatePosVel & s)
{

  // Get all samples from the current post through proposal density
  _old_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();

  _ns_it = _new_samples.begin();
  for ( _os_it=_old_samples.begin(); _os_it != _old_samples.end() ; _os_it++)
    {
      const StatePosVel& x_old = _os_it->ValueGet();
      _proposal->ConditionalArgumentSet(0,x_old);

      // Bug, make sampling method a parameter!
      _proposal->SampleFrom(_sample, DEFAULT,NULL);
      _ns_it->ValueSet(_sample.ValueGet());
      _ns_it->WeightSet(_os_it->WeightGet());
      _ns_it++;
    }

  (this->_timestep)++; // TODO needed?

  // Update the list of samples
  return (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);

}
