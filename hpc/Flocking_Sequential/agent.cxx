#include "agent.hxx"
#include "octree.hxx"


int Agent::curr_state =0;


Agent::Agent(const Vector &pos, const Vector &vel, const Vector &dir){
//TODO Use of position/velocity/direction lists ? + Parallelisr les opérations
//sur direction/velocity/position
  position[this->curr_state] = pos;
  velocity[this->curr_state] = vel;
  direction[this->curr_state] = dir;
}


Vector Agent::separation(TemporaryContainer &agent_list, size_t index, double rad) {
  Vector force = Zeros();
  int count = 0;
  for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      force -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized();
      ++count;
    }
  }
  return ( count>0 ? force/count : force);
}

Vector Agent::cohesion(TemporaryContainer &agent_list, size_t index, double rad) {
  Vector force = Zeros();

  int count = 0;
  for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if (i != index && dist < rad){
      /* TODO the comparison is no longer needed */
      force += agent_list[i]->position[this->curr_state];
      ++count;
    }
  }
  return ( count>0 ? force/count : force);
}

Vector Agent::alignment(TemporaryContainer&agent_list, size_t index, double rad) {
  Vector force = Zeros();

  int count = 0;
  for(size_t i = 0; i < agent_list.size(); i++) {
    double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      force += agent_list[i]->velocity[this->curr_state];
      ++count;
    }
  }
  return ( count>0 ? force/count : force);
}

/*size_t Agent::find_closest(Container &agent_list, size_t index) {
  size_t closest_agent = index;
  double min_dist = 1000;

  double dist;

  for(size_t i = 0; i < agent_list.size(); i++) {
    if (i != index) {
      dist= (this->position[this->curr_state] - agent_list[i].position[this->curr_state]).norm();

      if(dist < min_dist) {
        min_dist = dist;
        closest_agent = i;
      }
    }
  }
  return closest_agent;
}*/
