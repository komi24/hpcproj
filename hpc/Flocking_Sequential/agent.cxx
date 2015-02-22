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


Vector Agent::separation(TemporaryContainer &agent_list, double rad) {
  Real forcex = 0;
  Real forcey = 0;
  Real forcez = 0;
  //int count = 0;
  #pragma omp parallel 
  {
  #pragma omp for reduction( - : forcex)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcex -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized().x;
      //++count;
    //}
    }
  #pragma omp for reduction( - : forcey)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcey -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized().y;
      //++count;
    //}
    }
  #pragma omp for reduction( - : forcez)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcez -= (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).normalized().z;
      //++count;
    //}
    }
  }
  Vector force(forcex,forcey,forcez);
  return ( agent_list.size() >0 ? force/agent_list.size() : force);
}

Vector Agent::cohesion(TemporaryContainer &agent_list,  double rad) {
  Real forcex = 0;
  Real forcey = 0;
  Real forcez = 0;
  //int count = 0;
  #pragma omp parallel 
  {
  #pragma omp for reduction( + : forcex)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcex += agent_list[i]->position[this->curr_state].x;
      //++count;
    //}
    }
  #pragma omp for reduction( + : forcey)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcey += agent_list[i]->position[this->curr_state].y;
      //++count;
    //}
    }
  #pragma omp for reduction( + : forcez)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcez += agent_list[i]->position[this->curr_state].z;
      //++count;
    //}
    }
  }
  Vector force(forcex,forcey,forcez);
  return ( agent_list.size() >0 ? force/agent_list.size() : force);
}

Vector Agent::alignment(TemporaryContainer&agent_list, double rad) {
  Real forcex = 0;
  Real forcey = 0;
  Real forcez = 0;
  //int count = 0;
  #pragma omp parallel 
  {
  #pragma omp for reduction( + : forcex)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcex += agent_list[i]->velocity[this->curr_state].x;
      //++count;
    //}
    }
  #pragma omp for reduction( + : forcey)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcey += agent_list[i]->velocity[this->curr_state].y;
      //++count;
    //}
    }
  #pragma omp for reduction( + : forcez)
    for(size_t i = 0; i < agent_list.size(); i++) {
    //double dist = (this->position[this->curr_state] - agent_list[i]->position[this->curr_state]).norm();
    //if (i != index && dist < rad) {
      /* TODO the comparison is no longer needed */
      forcez += agent_list[i]->velocity[this->curr_state].z;
      //++count;
    //}
    }
  }
  Vector force(forcex,forcey,forcez);
  return ( agent_list.size() >0 ? force/agent_list.size() : force);
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

/* Return all the neighbours of an agent without itself */

void Agent::add_neighbours(Octree *parent, Vector pos_leaf,
  Real ra, TemporaryContainer &a,
  Real rb, TemporaryContainer &b,
  Real rc, TemporaryContainer &c){

  if (parent->width > Octree::widthmin){
  
    for (int i = 0; i < 8; i++){
      if (parent->child[i] != NULL){
      
        Vector child_pos = parent->child[i]->position;
        if ((pos_leaf >= child_pos) && ((child_pos + (parent->child[i]->width)) > pos_leaf)){
           

           add_neighbours(parent->child[i],pos_leaf,ra,a,rb,b,rc,c);
           return;
        }
      }
    }     
  }
  else {

    for (std::vector<Agent*>::iterator it = parent->agents.begin(); it != parent->agents.end(); it++){

        Real norm = (((*it)->position[curr_state])-position[curr_state]).norm();
        if (norm < ra)
          a.push_back(*it);
        if (norm < rb)
          b.push_back(*it);
        if (norm < rc)         
          c.push_back(*it);
    }
  }
}






