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

/* Return all the neighbours of an agent without itself */
void Agent::returnNeighbours(
  Real ra, TemporaryContainer &a,
  Real rb, TemporaryContainer &b,
  Real rc, TemporaryContainer &c){

  Octree *lf = leaf[curr_state];
  std::list<Vector> positions;
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y,lf->position.z));
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y,lf->position.z - lf->width));  
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y + lf->width,lf->position.z));
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y + lf->width,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y + lf->width,lf->position.z - lf->width));
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y - lf->width,lf->position.z));
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y - lf->width,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x + lf->width,lf->position.y - lf->width ,lf->position.z - lf->width));

  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y,lf->position.z));
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y,lf->position.z - lf->width));  
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y + lf->width,lf->position.z));
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y + lf->width,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y + lf->width,lf->position.z - lf->width));
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y - lf->width,lf->position.z));
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y - lf->width,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x - lf->width,lf->position.y - lf->width ,lf->position.z - lf->width));

  positions.push_back(Vector(lf->position.x ,lf->position.y ,lf->position.z));
  positions.push_back(Vector(lf->position.x ,lf->position.y + lf->width,lf->position.z));
  positions.push_back(Vector(lf->position.x ,lf->position.y,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x ,lf->position.y,lf->position.z - lf->width));
  positions.push_back(Vector(lf->position.x ,lf->position.y + lf->width,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x ,lf->position.y + lf->width,lf->position.z - lf->width));
  positions.push_back(Vector(lf->position.x ,lf->position.y - lf->width,lf->position.z));
  positions.push_back(Vector(lf->position.x ,lf->position.y - lf->width,lf->position.z + lf->width));
  positions.push_back(Vector(lf->position.x ,lf->position.y - lf->width ,lf->position.z - lf->width));

  
  Octree *ptr = lf;
 
 
  while (ptr->parent != NULL){

    ptr = ptr->parent;
 
       
    Vector pos_node = ptr->position;

      for (std::list<Vector>::iterator it = positions.begin(); it != positions.end();){
        
        if (((*it) >= pos_node) && ( (pos_node+ ptr->width)> (*it))){  
        
          
          add_neighbours(ptr,*it,ra,a,rb,b,rc,c);
          std::list<Vector>::iterator it2 = it;
          it++;
          positions.erase(it2);
        
        }
        else
          it++;
      }    
  }
}
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






