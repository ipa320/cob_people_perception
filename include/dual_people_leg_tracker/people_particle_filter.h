#ifndef PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_PEOPLE_PARTICLE_FILTER_H_
#define PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_PEOPLE_PARTICLE_FILTER_H_

//Own includes
#include <dual_people_leg_tracker/models/advanced_sysmodel_pos_vel.h>
#include <dual_people_leg_tracker/models/advanced_measmodel_pos.h>

// People Stack includes
#include <filter/particlefilter.h>

#define DEBUG_PEOPLE_PARTICLE_FILTER 1

using namespace BFL;

/**
 *  The Filter used for Tracking Legs and their People
 */
class PeopleParticleFilter
  : public BFL::ParticleFilter<BFL::StatePosVel, tf::Vector3>
{

  public:

    // Constructor
    PeopleParticleFilter(MCPdf<StatePosVel> * prior,
        MCPdf<StatePosVel> * post,
        int resampleperiod,
        double resamplethreshold,
        int resamplescheme);

    // Destructor
    virtual ~PeopleParticleFilter();

    /**
     * Update(Prediction) using the system model
     * @param sysmodel
     * @return true on success
     */
    bool
    Update(BFL::AdvancedSysModelPosVel* const sysmodel)
    {
      ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER,"----PeopleParticleFilter::%s -> System Model Update",__func__);

      StatePosVel s; // Sensing Parameter ??!
      tf::Vector3 z; // Measurement
      StatePosVel u; // Input to the system

      OcclusionModelPtr nullPtr; // TODO ugly!

      return this->UpdateInternal(sysmodel,u,NULL,z,s,nullPtr);
    }

    /**
    * Do a measurement Update
    * @param measmodel
    * @param meas
    * @return
    */
    bool
    Update(BFL::AdvancedMeasModelPos* const measmodel, const tf::Vector3& meas)
    {
      ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER,"----PeopleParticleFilter::%s -> System Model Update",__func__);

      StatePosVel s; // Sensing Parameter ??!
      tf::Vector3 z; // Measurement
      StatePosVel u; // Input to the system

      z = meas;

      OcclusionModelPtr nullPtr; // TODO ugly!

      return this->UpdateInternal(NULL,u,measmodel,z,s,nullPtr);
    }


    bool
    Update(BFL::AdvancedMeasModelPos* const measmodel, const tf::Vector3& meas, OcclusionModelPtr occlusionmodel)
    {
      ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER,"----PeopleParticleFilter::%s -> System Model Update",__func__);

      StatePosVel s; // Sensing Parameter ??!
      tf::Vector3 z; // Measurement
      StatePosVel u; // Input to the system

      z = meas;

      return this->UpdateInternal(NULL,u,measmodel,z,s,occlusionmodel);
    }

    /**
     * Do a internal update
     * @param sysmodel pointer to the used system model
     * @param u input param for proposal density
     * @param measmodel pointer to the used measurementmodel
     * @param z measurement param for proposal density
     * @param s sensor param for proposal density
     * @return
     */
    bool UpdateInternal(BFL::AdvancedSysModelPosVel* const sysmodel,
                 const StatePosVel& u,
                 MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
                 const tf::Vector3& z,
                 const StatePosVel& s,
                 OcclusionModelPtr occlusionmodel);

    /**
     * Update the weights using the measurement models
     * @param sysmodel
     * @param u
     * @param measmodel
     * @param z
     * @param s
     * @return
     */
    bool UpdateWeightsInternal(BFL::AdvancedSysModelPosVel* const sysmodel,
                 const StatePosVel& u,
                 MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,
                 const tf::Vector3& z,
                 const StatePosVel& s);

    /**
     * Get the probability of a given measurement, this is needed for the JPDA filter
     * @param measmodel
     * @param z
     * @return The probability
     */
    double
    getMeasurementProbability(MeasurementModel<tf::Vector3,StatePosVel>* const measmodel,const tf::Vector3& z);

    bool
    UpdateWeightsUsingOcclusionModel(OcclusionModelPtr occlusionmodel);


    /**
     * Do a dynamic resampling. Draw new samples if(!) there are a lot of samples with low weight.
     * @return True on success.
     */
    bool
    DynamicResampleStep();

    /**
     * Draw new sample from the posterior distribution.
     * @return
     */
    bool
    Resample();


    bool
    ProposalStepInternal(SystemModel<StatePosVel> * const sysmodel,
                  const StatePosVel & u,
                  MeasurementModel<tf::Vector3,StatePosVel> * const measmodel,
                  const tf::Vector3 & z,
                  const StatePosVel & s);
};

#endif /* PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_PEOPLE_PARTICLE_FILTER_H_ */
