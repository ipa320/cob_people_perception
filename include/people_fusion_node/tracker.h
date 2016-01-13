#ifndef PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_
#define PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_

// Ros includes
#include <ros/ros.h>

// Own includes
#include <people_fusion_node/state_pos_vel.h>


class Tracker{

	private:
		ros::Time initiationTime_;
		StatePosVel initialState_;

		ros::Time currentTime_;
		StatePosVel currentState_;

		// For easy outstreaming
		friend std::ostream& operator<<(std::ostream&, const Tracker&);


	public:
		Tracker(StatePosVel init, ros::Time initialTime);

		ros::Time getCurrentTime() { return this->currentTime_; };

		StatePosVel getCurrentState() { return this->currentState_; };

		void predict(ros::Time updateTime);

		void update(StatePosVel state, ros::Time time);

};

// Define the Share pointer
typedef boost::shared_ptr<Tracker> TrackerPtr;




#endif /* PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_ */
