#ifndef  WORKSPACE
#define  WORKSPACE

#include "octree.hxx"
#include "agent.hxx"

Real Octree::widthmin;
LeafContainer Octree::Leafs;

Octree::Octree(Real width, 
	Octree *parent, 
	Vector &position, 
	int index){

	this->index = index;
	this->width = width;
	this->parent = parent;
	this->position = *(new Vector (position.x, position.y, position.z));
	for(int i =0; i<8; i++)
		child[i]=NULL;
	if (width <= widthmin)
		Leafs.push_back(this);
	
}

Octree::Octree(Real wmin, Real width){
	widthmin = wmin;
	this->width = width;
	this->parent = NULL;
	this->position = *(new Vector(0,0,0));
	for(int i =0; i<8; i++)
		child[i]=NULL;
}


void Octree::add(Agent &a) {
	if (width > widthmin) {
		int i = 0;
		Vector new_position(position.x, position.y, position.z);
		if (a.position[Agent::curr_state].x > position.x + width/2){
				// s'assurer que la taille de l'espace est une puissance de 2
			i += 1;
			new_position.x += width/2;
		}
		if (a.position[Agent::curr_state].y > position.y + width/2){
				// s'assurer que la taille de l'espace est une puissance de 2
			i += 2;
			new_position.y += width/2;
		}
		if (a.position[Agent::curr_state].z > position.z + width/2){
				// s'assurer que la taille de l'espace est une puissance de 2
			i += 4;
			new_position.z += width/2;
		}

		if (child[i] == NULL)
			child[i] = new Octree(width/2, this, new_position, i); 
		child[i]->add(a);
	} else {
		a.leaf[Agent::curr_state] = this;
		agents.push_back(&a);
	}
}
void Octree::returnNeighboursLeaf(LeafContainer leafs){
Octree *lf = this;

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
        
          
          add_neighbours(leafs);
          std::list<Vector>::iterator it2 = it;
          it++;
          positions.erase(it2);
        
        }
        else
          it++;
      }    
  }
}
void Octree::add_neighbours(Octree *parent, Vector pos_leaf,LeafContainer leafs){

  if (parent->width > Octree::widthmin){
  
    for (int i = 0; i < 8; i++){
      if (parent->child[i] != NULL){
      
        Vector child_pos = parent->child[i]->position;
        if ((pos_leaf >= child_pos) && ((child_pos + (parent->child[i]->width)) > pos_leaf)){
           

           add_neighbours(parent->child[i],pos_leaf,leafs);
           return;
        }
      }
    }     
  }
  else {
  	leafs.push_back(parent);
	}
    
}

void Octree::delete_leaves(){
	if(agents.size() == 0 ){
		Octree *p= parent;
		parent->child[index]=NULL;
		delete this;
		p->delete_leaves();
	}
}

#endif