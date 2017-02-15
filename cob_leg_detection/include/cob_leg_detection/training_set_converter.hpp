/*
 * training_set_converter.hpp
 *
 *  Created on: Mar 2, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_TRAINING_SET_CONVERTER_HPP_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_TRAINING_SET_CONVERTER_HPP_

#include <cob_leg_detection/ClusterMsg.h>
#include <cob_leg_detection/LabeledRangeScanMsg.h>

#include <sensor_msgs/LaserScan.h>

class LabeledScanData{
private:
    sensor_msgs::LaserScan::Ptr pLaserScan;
    cob_leg_detection::LabeledRangeScanMsg::Ptr pLabeledRangeScanMsg;

public:
    LabeledScanData(sensor_msgs::LaserScan::Ptr pLaserScan, cob_leg_detection::LabeledRangeScanMsg::Ptr pLabeledRangeScanMsg){
        this->pLaserScan = pLaserScan;
        this->pLabeledRangeScanMsg = pLabeledRangeScanMsg;
    }

    void convertTrainingSet(const char* file);

};

/**
* The TrainingSetCreator Class helps creating bagfiles with annotated and visualized labels.
*/
class TrainingSetConverter {
private:
    std::list<LabeledScanData*> labeledScanList;

public:
    /** Constructor */
    TrainingSetConverter(){}

    /** Deconstructor */
    ~TrainingSetConverter() {}

    void convertTrainingSet(const char* file);

};

#endif /* PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_TRAINING_SET_CONVERTER_HPP_ */
