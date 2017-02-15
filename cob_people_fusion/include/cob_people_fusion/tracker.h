#ifndef PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_
#define PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_TRACKER_H_

// Ros includes
#include <ros/ros.h>

// Own includes
#include <cob_people_fusion/state_pos_vel.h>
#include <cob_people_fusion/detection/detection.h>
#include <cob_people_fusion/detector_config.h>

/**
 * Represents a tracked object/person
 */

class Tracker{

	private:
    unsigned int id_; /**< assigned id */

    double score_; /**< score[0..1] corresponding to the reliability in beeing a person */

    double timeHorizon_;

		ros::Time initiationTime_; /**< time this tracker was initialized */
		StatePosVel initialState_; /**< state on initialization */

		ros::Time currentTime_; /**< the time of the current state */
		StatePosVel currentState_; /**< the current state */

		std::vector<DetectionPtr> updates_; /**< the updates this tracker received(trimmed to time horizon) */
	  std::map<std::string, size_t> counts_; /**< mapping from detection type names to number of this detections */
    std::map<std::string, double> frequencies_; /**< mapping from detection type names to frequency of this detections */

		// For easy outstreaming
		friend std::ostream& operator<<(std::ostream&, const Tracker&);


	public:
		Tracker(StatePosVel init, ros::Time initialTime, std::vector<detector_config> detector_configs, double timeHorizon);

		unsigned int getId() const { return this->id_; };

		std::string getIdStr() const;

		ros::Time getCurrentTime() const { return this->currentTime_; };

		ros::Time getInitTime() const { return this->initiationTime_; };

		StatePosVel getCurrentState() { return this->currentState_; };

		void predict(ros::Time updateTime);

		void update(DetectionPtr detection);
		void update_with_old_detection(DetectionPtr detection);

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
