/*
 * detection.h
 *
 *  Created on: Jun 8, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_DETECTION_DETECTION_H_
#define PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_DETECTION_DETECTION_H_

#include <tf/transform_datatypes.h>
#include <cob_leg_detection/laser_processor.h>

class Detection{
  private:
    bool is_real_; /**< Needed for 'fake' measurements */

    double fake_probability_;

    tf::Stamped<tf::Point> point_;          /**< Location of the detection, given within fixed frame */

    laser_processor::SampleSet* cluster_;

    unsigned int id_;                       /**< Unique id (within cycle)*/

    bool used_for_update_; /** Check if this has been used for a update */

  public:
    // Detection for real
    Detection(int id, tf::Stamped<tf::Point> point, laser_processor::SampleSet* cluster):
      id_(id),
      point_(point),
      cluster_(cluster),
      is_real_(true),
      fake_probability_(0),
      used_for_update_(false)
    {

    }
    // Fake Detection
    Detection(int id, tf::Stamped<tf::Point> point, double probability):
      id_(id),
      point_(point),
      fake_probability_(probability),
      is_real_(false),
      cluster_(NULL),
      used_for_update_(false)
    {

    }

    bool isReal(){
      return is_real_;
    }

    double getProbability(){
      if(isReal()){
        return cluster_->getProbability();
      }
      return fake_probability_;
    }

    tf::Stamped<tf::Point>& getLocation(){
      return this->point_;
    }

    unsigned int getId() const{
      return this->id_;
    }

    std::string getIdStr() const{
      // Generate the string id
      char id[100];
      snprintf(id, 100, "LM[%d]", this->id_);

      return std::string(id);
    }

    laser_processor::SampleSet* getCluster() const{
      return this->cluster_;
    }

    void setUsedForUpdate(bool used){
      this->used_for_update_ = used;
    }

    bool usedForUpdate() const{
      return this->used_for_update_;
    }



};

typedef boost::shared_ptr<Detection> DetectionPtr;


#endif /* PEOPLE_cob_dual_leg_tracker_INCLUDE_cob_dual_leg_tracker_DETECTION_DETECTION_H_ */
