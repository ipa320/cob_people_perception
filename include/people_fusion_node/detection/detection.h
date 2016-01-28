#ifndef PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTION_H_
#define PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTION_H_

// Own includes
#include <people_fusion_node/state_pos_vel.h>

/**
 * This class represents a single person detection.
 */
class Detection{

  private:

    unsigned int id_; /**< The id of the detection, actually just needed for comfortable debugging */

    StatePosVel state_; /**< The state of this detection */

    ros::Time detectionTime_; /**< The time of the detection */

    std::string detection_type_; /**< The detection type, (laser, 3d, etc...) */

  public:
    Detection(double x, double y, ros::Time time, unsigned int id, std::string type);

    unsigned int getId() const { return this->id_; };


    StatePosVel getState() const { return this->state_; };

    ros::Time getTime() const { return this->detectionTime_; };

    std::string getDetectionType() const { return this->detection_type_; };

    // For easy outstreaming
    friend std::ostream& operator<<(std::ostream&, const Detection&);

};

// Define the Share pointer
typedef boost::shared_ptr<Detection> DetectionPtr;


#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTION_H_ */
