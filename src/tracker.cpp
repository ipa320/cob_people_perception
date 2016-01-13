// Own includes
#include <people_fusion_node/tracker.h>
#include <people_fusion_node/state_pos_vel.h>


Tracker::Tracker(StatePosVel init, ros::Time initialTime):
		initiationTime_(initialTime),
		initialState_(init),
		currentTime_(initiationTime_),
		currentState_(initialState_){

}

void Tracker::update(StatePosVel state, ros::Time updateTime){
	// TODO implement!
}

void Tracker::predict(ros::Time predictionTime){
	// TODO implement!
}

std::ostream& operator<<(std::ostream &strm, const Tracker &tracker) {
  return strm << "Tracker" << tracker.currentState_;
}
