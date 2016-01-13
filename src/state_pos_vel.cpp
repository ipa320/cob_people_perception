#include <people_fusion_node/state_pos_vel.h>

StatePosVel::StatePosVel(double x, double y):
	x_(x),
	y_(y),
	vx_(0),
	vy_(0)
{
}

StatePosVel::StatePosVel(double x, double y, double vx, double vy):
	x_(x),
	y_(y),
	vx_(vx),
	vy_(vy){

}

std::ostream& operator<<(std::ostream &strm, const StatePosVel &state) {
  return strm << "<x:" << state.x_ << ", y:" << state.y_ << "| vx: " << state.vx_ << ", vy: " << state.vy_ << ">";
}
