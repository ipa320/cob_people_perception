#ifndef PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTION_H_
#define PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTION_H_

// Own includes
#include <people_fusion_node/state_pos_vel.h>

class Detection{

  private:
    unsigned int id_;
    StatePosVel state_;
    ros::Time detectionTime_;

  public:
    Detection(double x, double y, ros::Time time, unsigned int id);

    unsigned int getId() const { return this->id_; };
    StatePosVel getState() const { return this->state_; };
    ros::Time getTime() const { return this->detectionTime_; };

    // For easy outstreaming
    friend std::ostream& operator<<(std::ostream&, const Detection&);


};

// Define the Share pointer
typedef boost::shared_ptr<Detection> DetectionPtr;


#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTION_H_ */
