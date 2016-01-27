// Own includes
#include <people_fusion_node/fusion_node.h>
#include <people_fusion_node/tracker.h>
#include <people_fusion_node/state_pos_vel.h>
#include <people_fusion_node/consts.h>


Tracker::Tracker(StatePosVel init, ros::Time initialTime, std::vector<detector_config> detector_configs):
    id_(trackerIdCounter++),
    initiationTime_(initialTime),
    initialState_(init),
    currentTime_(initiationTime_),
    currentState_(initialState_),
    score_(0)
    {

    // Initialize the count maps
    for(size_t i = 0; i < detector_configs.size(); i++){
      counts_.insert ( std::pair<std::string,size_t>(detector_configs[i].name,0) );
      frequencies_.insert ( std::pair<std::string,double>(detector_configs[i].name,0) );
    }
}

void Tracker::update(DetectionPtr detection){
  this->currentState_ = detection->getState();
  this->currentTime_ = detection->getTime();
  this->updates_.push_back(detection);

  // Trim the detection list from the front to a desired time horizon
  DetectionPtr oldestDetection = this->updates_.front();
  DetectionPtr newestDetection = this->updates_.back();

  double currentHorizon = (newestDetection->getTime() - oldestDetection->getTime()).toSec();

  while(currentHorizon > timeHorizon){

    // Decrease the counter
    counts_[oldestDetection->getDetectionType()]--;

    // Remove first element
    this->updates_.erase( this->updates_.begin() );

    // Set the new oldest element
    oldestDetection = this->updates_.front();

    currentHorizon = (newestDetection->getTime() - oldestDetection->getTime()).toSec();

  }

  // Update the counter
  counts_[detection->getDetectionType()]++;

  // Update the frequencies
  for(std::map<std::string, size_t>::iterator mapIt = counts_.begin(); mapIt != counts_.end(); mapIt++) {
    if(currentHorizon > 0){
      frequencies_[detection->getDetectionType()] = counts_[detection->getDetectionType()] / currentHorizon;
    }
    else
    {
      frequencies_[detection->getDetectionType()] = 0;
    }
  }

}

std::map<std::string, size_t>& Tracker::getUpdateCounts(){
  return counts_;
}

std::map<std::string, double>& Tracker::getUpdateFrequencies(){
  return frequencies_;
}

void Tracker::predict(ros::Time predictionTime){
  // TODO implement!
}

std::string Tracker::getIdStr() const{
  std::stringstream ss;
  ss << *this;

  return ss.str();
}

size_t Tracker::getDiversity() const{
  size_t diversity = 0;

  // Update the frequencies
  for(std::map<std::string, size_t>::const_iterator mapIt = counts_.begin(); mapIt != counts_.end(); mapIt++) {
    if(mapIt->second > 0) diversity++;
  }

  return diversity;
}

std::ostream& operator<<(std::ostream &strm, const Tracker &tracker) {
  return strm << "Tracker[" << tracker.id_ << " score:" << tracker.getScore() << "]";
}
