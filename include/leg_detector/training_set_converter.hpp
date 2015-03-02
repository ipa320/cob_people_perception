/*
 * training_set_converter.hpp
 *
 *  Created on: Mar 2, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_TRAINING_SET_CONVERTER_HPP_
#define PEOPLE_LEG_DETECTOR_INCLUDE_LEG_DETECTOR_TRAINING_SET_CONVERTER_HPP_

#include <leg_detector/ClusterMsg.h>
#include <leg_detector/LabeledRangeScanMsg.h>

#include <sensor_msgs/LaserScan.h>

class LabeledScanData{
private:
    sensor_msgs::LaserScan::Ptr pLaserScan;
    leg_detector::LabeledRangeScanMsg::Ptr pLabeledRangeScanMsg;

public:
    LabeledScanData(sensor_msgs::LaserScan::Ptr pLaserScan, leg_detector::LabeledRangeScanMsg::Ptr pLabeledRangeScanMsg){
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
