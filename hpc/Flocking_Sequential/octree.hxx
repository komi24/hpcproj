#ifndef PARSER
#define PARSER

#include <vector>
#include "vector.hxx"
#include "types.hxx"

class Agent;

class Octree {

private:
	/* Ordre spatial à respecter x,y,z croissant successif */
	Octree *child[8];
	Octree *parent;
	Real width; //width of the spatial cell
	Vector position; //x,y,z les plus petits
	int index; // index position in its parent node


public:	
	TemporaryContainer agents; // we may use a different data structure than for neighbours
	static Real widthmin;
	
	Octree(Real wmin, Real width);
	Octree(Real width, Octree *parent, Vector &position, int index);
	/* Add an agent to its corresponding leaf 
	BEWARE add(reference) but save pointers */
	void add(Agent &a);


	/* Return all the neighbours of an agent without itself */
	void returnNeighbours(const Agent a1,
	 Real ra, TemporaryContainer &a,
	 Real rb, TemporaryContainer &b,
	 Real rc, TemporaryContainer &c);

	/* TODO return smartly the next agent to compute 
	to avoid re-computing too many distances */


	void  delete_leaves();
};

#endif