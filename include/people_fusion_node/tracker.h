#ifndef PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_
#define PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_

// Ros includes
#include <ros/ros.h>

// Own includes
#include <people_fusion_node/state_pos_vel.h>
#include <people_fusion_node/detection/detection.h>
#include <people_fusion_node/detector_config.h>



class Tracker{

	private:
    unsigned int id_;

    double score_;

		ros::Time initiationTime_;
		StatePosVel initialState_;

		ros::Time currentTime_;
		StatePosVel currentState_;

		std::vector<DetectionPtr> updates_;
	  std::map<std::string, size_t> counts_;
    std::map<std::string, double> frequencies_;

		// For easy outstreaming
		friend std::ostream& operator<<(std::ostream&, const Tracker&);


	public:
		Tracker(StatePosVel init, ros::Time initialTime, std::vector<detector_config> detector_configs);

		unsigned int getId() const { return this->id_; };

		std::string getIdStr() const;

		ros::Time getCurrentTime() const { return this->currentTime_; };

		ros::Time getInitTime() const { return this->initiationTime_; };

		StatePosVel getCurrentState() { return this->currentState_; };

		void predict(ros::Time updateTime);

		void update(DetectionPtr detection);

		std::map<std::string, size_t>& getUpdateCounts();

		std::map<std::string, double>& getUpdateFrequencies();

		StatePosVel getCurrentState() const { return this->currentState_; };

		size_t getDiversity() const;

		double getScore() const { return this->score_; };

		void setScore(double score) {

		  this->score_ = std::min(score,1.0);

		};

		inline double getLifetimeSec(ros::Time currentTime) const  {
		  return (currentTime - this->getInitTime()).toSec();
		}

};

// Define the Share pointer
typedef boost::shared_ptr<Tracker> TrackerPtr;




#endif /* PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_ */
