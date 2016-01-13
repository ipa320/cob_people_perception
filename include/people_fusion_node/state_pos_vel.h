#ifndef PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_STATE_POS_VEL_H_
#define PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_STATE_POS_VEL_H_

//System includes
#include <iostream>

class StatePosVel{
public: // Todo make private!
	double x_;
	double y_;
	double vx_;
	double vy_;

private:
	// For easy outstreaming
	friend std::ostream& operator<<(std::ostream&, const StatePosVel&);

public:
	StatePosVel(double x, double y);

	StatePosVel(double x, double y, double vx_, double vy_);


};


#endif /* PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_STATE_POS_VEL_H_ */
