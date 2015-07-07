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
#include <dual_people_leg_tracker/visualization/color_definitions.h>
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

    std::vector<WeightedSample<StatePosVel> > samples = ((MCPdf<StatePosVel> *) this->_post)->ListOfSamplesGet();

    //std::cout << "Samples before update" << std::endl;
    for(int i = 0; i<10; i++){
      StatePosVel sample = samples[i].ValueGet();
      double weight = samples[i].WeightGet();
      //std::cout << "Sample " << sample << "Weight: " << weight << std::endl;
    }

    //assert(this->_dynamicResampling == true); // TODO
    //result = result && this->StaticResampleStep(); // TODO necessary?

    //result = result && this->DynamicResampleStep();

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
PeopleParticleFilter::UpdateWeightsJPDA(MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
             const std::vector<DetectionPtr> z,
             Eigen::VectorXd assignmentProbabilities,
             OcclusionModelPtr occlusionModel){

  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  //std::cout << " #### Update Weights JPDA" << std::endl;
  //std::cout << "Assignment Probabilities" << std::endl << assignmentProbabilities << std::endl;


  // Number of measurements
  unsigned int m_k = assignmentProbabilities.rows();

  for(unsigned int j = 1; j < m_k; j++){
    //if(assignmentProbabilities[j] > 0.0)
    //  std::cout << "Update is done with measurement " << j-1 << " at " << z[j-1]->point_[0] << "   " << z[j-1]->point_[1] << " with meas prob:" << assignmentProbabilities[j] << std::endl;
  }

  double weightfactor = 1;

  // Get the posterior samples
  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();
  //_os_it = _old_samples.begin();

  // Iterate through the samples
  for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++){

    // Get the value of this sample
    const StatePosVel& x_new = _ns_it->ValueGet();

    double weightSum = 0;

    // Handle occlusion probability
    if(assignmentProbabilities[0] > 0.0){
      double occProb = occlusionModel->getOcclusionProbability(x_new.pos_);
      weightSum += assignmentProbabilities[0] * occProb;
    }

    // Sum the weight over this sample using all measurement probabilities
    for(unsigned int j = 1; j < m_k; j++){
      if(assignmentProbabilities[j] > 0.0){


          double measProb = measmodel->ProbabilityGet(z[j-1]->point_,x_new);
          //std::cout << "assignmentProbability=" << assignmentProbabilities[j] << "  meas Prob=" << measProb;
          weightSum += assignmentProbabilities[j] * measProb;
          //std::cout << " weightSum=" << weightSum << std::endl;
      }
    }

    //std::cout << "Weight Update: " << _ns_it->WeightGet() << "    -->    ";
    //_ns_it->WeightSet(_ns_it->WeightGet() * weightSum);// <-- See Thrun "Probabilistic Robotics", p 99-100 eq (4.24) this method is inferior...
    _ns_it->WeightSet(weightSum);
    //std::cout << _ns_it->WeightGet() << std::endl;

    // _ns_it->WeightSet(_ns_it->WeightGet() * weightfactor);

    // OR //TODO
    //_ns_it->WeightSet(weightfactor);
  }

  // Normalize the weight
  //_new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->NormalizeWeights();


  // Update the posterior (Automatic Normalization is included)
  bool result = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);

  // Resampling
  //result = result && this->DynamicResampleStep();

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

  // Get the posterior samples
  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();
  _os_it = _old_samples.begin();

  // Iterate through the samples
  for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++){

    const StatePosVel& x_new = _ns_it->ValueGet();
    //const StatePosVel& x_old = _os_it->ValueGet();

    weightfactor = measmodel->ProbabilityGet(z,x_new);
    // TODO apply occlusion model here

    //std::cout << "Weight Update: " << _ns_it->WeightGet() << "    -->     ";
    //_ns_it->WeightSet(_ns_it->WeightGet() * weightfactor);// <-- See Thrun "Probabilistic Robotics", p 99-100 eq (4.24) this method is inferior...

    _ns_it->WeightSet(weightfactor);
    //if(weightfactor.getValue() > 0.01)
    //std::cout << "weight set" << weightfactor.getValue() << std::endl;
    //std::cout << _ns_it->WeightGet() << std::endl;
  }
  // Update the posterior
  bool result = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);


  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();
  for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++){
    //if(_ns_it->WeightGet() > 0.01)
    //std::cout << "weight get after update" <<  _ns_it->WeightGet() << std::endl;
  }

  return (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);
}

/*// Update the weights according to the assignment probabilities
bool
PeopleParticleFilter::UpdateWeightsJPDA(MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
             const tf::Vector3& z,
             const StatePosVel& s){

  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);


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
}*/

double
PeopleParticleFilter::getMeasurementProbability(
    MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
    const tf::Vector3& z)
  {
  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  Probability weightfactor = 1;
  Probability zeroProb = ((AdvancedMeasPdfPos*) measmodel->MeasurementPdfGet())->getZeroProbability();

  double probability = 0;
  double probability_sum = 0;

  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();

  // Iterate through the samples (posterior)
  for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++){

    const StatePosVel& x_new = _ns_it->ValueGet();

    // Get the probability of this particle
    Probability prop = measmodel->ProbabilityGet(z,x_new);

    //std::cout << "prob " << prop.getValue() << std::endl;
//
//    assert(prop >= 0.0);
//    assert(prop <= 1.0);

    probability_sum += prop;

  }

  // Normalizing
  probability_sum = probability_sum / zeroProb.getValue();

  // Mean
  probability = probability_sum/_new_samples.size();
//
//  std::cout << "Probability " << probability << " number of Samples" << _new_samples.size() << std::endl;

  return probability;
  }

double
PeopleParticleFilter::getOcclusionProbability(OcclusionModelPtr occlusionModel)
  {
  ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER, "----PeopleParticleFilter::%s",__func__);

  Probability weightfactor = 1;

  double probability = 0;
  double probability_sum = 0;

  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();

  // Iterate through the samples (posterior)
  for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++){

    const StatePosVel& x_new = _ns_it->ValueGet();

    Probability prop = occlusionModel->getOcclusionProbability(x_new.pos_);

    probability_sum += prop;

  }

  return probability_sum/_new_samples.size();
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

  // Get the list of current posterior
  _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();

  // Iterate through the samples to assign weights
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

    // Check if sum of 1 / \sum{(wi_normalised)^2} < threshold (Name: Effective sample size)
    // This is the criterion proposed by Liu
    // BUG  foresee other methods of approximation/calculating
    // effective sample size.  Let the user implement this in !
    _new_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();
    for ( _ns_it=_new_samples.begin(); _ns_it != _new_samples.end() ; _ns_it++)
      {
        sum_sq_weigths += pow(_ns_it->WeightGet(),2);
      }

    double effectiveSampleSize = 1.0 / sum_sq_weigths;
    //std::cout << "effectiveSampleSize " << effectiveSampleSize << " threshold(" << _resampleThreshold << ")" << std::endl;

    if (effectiveSampleSize < _resampleThreshold)
      {

        resampling = true;
      }
  }
    if (resampling == true){
      //std::cout << RED << "RESAMPLE! " << RESET << std::endl;
      return this->LowVarianceResample();
      //return this->Resample();
    }
      //return this->Resample();
    else
      return true;
}

/**
 * This is the resample Function copied from the BFL resampling of the particle filter class.
 * This is done the gain better access to this function without having to change the library.
 * @return
 */
bool
PeopleParticleFilter::LowVarianceResample()
{

  // Update the weights of the posterior!

  int NumSamples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->NumSamplesGet();

  // Low Variance Sampling

  // Set old samples to the posterior of the previous run
  _old_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();

  // Get pointer to the new samples
  _ns_it = _new_samples.begin();

  // Calculate the total weight

  double r = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX)) * (1.0/NumSamples);
  double c = _old_samples[0].WeightGet(); // Set to the weight of the first sample

  //std::cout << "random " << r << std::endl;

  unsigned int i = 0;

  double invM =  1.0/NumSamples;

  double U = 0.0;
  for(int m = 0; m < NumSamples; m++){
    U = r + (m - 1) * invM;

    while(U > c){
      i++;
      c += _old_samples[i].WeightGet();
    }
    // Set the weight from the old sample
    _ns_it->ValueSet(_old_samples[i].ValueGet());
    _ns_it->WeightSet(_old_samples[i].WeightGet());

    // Go to the next iterator
    _ns_it++;

  }

  //std::cout << "LOW VARIANCE RESAMPLING!!!" << std::endl;



/*  std::cout << "r: " << r << std::endl;

  assert(false);

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
    }*/
  //bool result = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples_unweighted);

  bool result = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);

  return result;
}

/**
 * Generate New Samples using the proposal
 * @param sysmodel
 * @param u
 * @param measmodel
 * @param z
 * @param s
 * @return
 */
bool
PeopleParticleFilter::ProposalStepInternal(SystemModel<StatePosVel> * const sysmodel,
              const StatePosVel & u,
              MeasurementModel<tf::Vector3,StatePosVel> * const measmodel,
              const tf::Vector3 & z,
              const StatePosVel & s)
{

  // Set old samples to the posterior of the previous run
  _old_samples = (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesGet();

  // Get pointer to the new samples
  _ns_it = _new_samples.begin();

  // Iteratively update the new samples by sampling from the proposal
  for ( _os_it=_old_samples.begin(); _os_it != _old_samples.end() ; _os_it++)
    {

      // Get old value
      const StatePosVel& x_old = _os_it->ValueGet();

      // Set the proposal as conditional argument
      _proposal->ConditionalArgumentSet(0,x_old);

      // Generate sample from the proposal
      _proposal->SampleFrom(_sample, DEFAULT,NULL);

      // Set the Value from the
      _ns_it->ValueSet(_sample.ValueGet());

      // Set the weight from the old sample
      _ns_it->WeightSet(_os_it->WeightGet());

      // Go to the next iterator
      _ns_it++;
    }

  (this->_timestep)++; // TODO needed?

  // Update the posterior
  return (dynamic_cast<MCPdf<StatePosVel> *>(this->_post))->ListOfSamplesUpdate(_new_samples);

}
