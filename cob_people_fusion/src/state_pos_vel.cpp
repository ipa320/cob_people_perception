#include <cob_people_fusion/state_pos_vel.h>

StatePosVel::StatePosVel(double x, double y):
  pos_(x,y,0),
  vel_(0,0,0)
{
}

StatePosVel::StatePosVel(double x, double y, double vx, double vy):
  pos_(x,y,0),
  vel_(vx,vy,0)
{

}

std::ostream& operator<<(std::ostream &strm, const StatePosVel &state) {
  return strm << "<x:" << state.getPos().getX() << ", y:" << state.getPos().getY() << "| vx: " << state.getVel().getX() << ", vy: " << state.getVel().getY() << ">";
}
