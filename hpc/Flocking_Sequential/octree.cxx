#ifndef  WORKSPACE
#define  WORKSPACE

#include "octree.hxx"
#include "agent.hxx"


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
	/* Return all the neighbours of an agent without itself */
void Octree::returnNeighbours(const Agent a1,
	Real ra, TemporaryContainer &a,
	Real rb, TemporaryContainer &b,
	Real rc, TemporaryContainer &c){

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