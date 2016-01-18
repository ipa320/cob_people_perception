// Own includes
#include <people_fusion_node/fusion_node.h>
#include <people_fusion_node/tracker.h>
#include <people_fusion_node/state_pos_vel.h>


Tracker::Tracker(StatePosVel init, ros::Time initialTime):
    id_(trackerIdCounter++),
    initiationTime_(initialTime),
    initialState_(init),
    currentTime_(initiationTime_),
    currentState_(initialState_){

}

void Tracker::update(DetectionPtr detection){
  this->currentState_ = detection->getState();
  this->currentTime_ = detection->getTime();
}

void Tracker::predict(ros::Time predictionTime){
  // TODO implement!
}

std::ostream& operator<<(std::ostream &strm, const Tracker &tracker) {
  return strm << "Tracker[" << tracker.id_ << "]" << tracker.currentState_;
}
