#ifndef PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_PEOPLE_PARTICLE_FILTER_H_
#define PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_PEOPLE_PARTICLE_FILTER_H_

//BFL includes

#include <filter/particlefilter.h>
#include <people_tracking_filter/sysmodel_pos_vel.h>
#include <people_tracking_filter/advanced_sysmodel_pos_vel.h>

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
    PeopleParticleFilter(BFL::MCPdf<BFL::StatePosVel> * prior,
        BFL::MCPdf<BFL::StatePosVel> * post,
        int resampleperiod,
        double resamplethreshold,
        int resamplescheme);

    // Destructor
    virtual ~PeopleParticleFilter();

    bool
    Update(BFL::AdvancedSysModelPosVel* const sysmodel)
    {
      ROS_DEBUG_COND(DEBUG_PEOPLE_PARTICLE_FILTER,"----PeopleParticleFilter::%s -> System Model Update",__func__);

      StatePosVel s; // Sensing Parameter ??!
      tf::Vector3 z; // Measurement
      StatePosVel u; // Input to the system
      return this->UpdateInternal(sysmodel,u,NULL,z,s);
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
                 const StatePosVel& s);
};

#endif /* PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_PEOPLE_PARTICLE_FILTER_H_ */
