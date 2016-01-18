#ifndef PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_STATE_POS_VEL_H_
#define PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_STATE_POS_VEL_H_

//System includes
#include <iostream>

#include <tf/tf.h>

class StatePosVel{
private: // Todo make private!

  tf::Vector3 pos_;
  tf::Vector3 vel_;

private:
	// For easy outstreaming
	friend std::ostream& operator<<(std::ostream&, const StatePosVel&);

public:
	StatePosVel(double x, double y);

	StatePosVel(double x, double y, double vx_, double vy_);

	const tf::Vector3 getPos() const { return this->pos_; };
	const tf::Vector3 getVel() const { return this->vel_; };


};


#endif /* PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_STATE_POS_VEL_H_ */
