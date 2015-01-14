#ifndef PARSER
#define PARSER

#include <agent.hxx>
#include <vector>
#include <vector.hxx>
#include <types.hxx>

class Octree {

private:
	/* Ordre spatial à respecter x,y,z croissant successif */
	Octree[8] child;
	Octree *parent;
	static Real widthMin;
	Real width; //width of the spatial cell
	Vector position; //x,y,z les plus petits
	TemporaryContainer agents; // we may use a different data structure than for neighbours


public:	
	/* Add an agent to its corresponding leaf */
	void add(Agent &a);

	/* Update curr_state and move agent that needed
	to be moved */
	void update_state();

	/* Return all the neighbours of an agent without itself */
	void returnNeighbours(const Agent a1,
	 Real ra, TemporaryContainer &a,
	 Real rb, TemporaryContainer &b,
	 Real rc, TemporaryContainer &c);

	/* TODO return smartly the next agent to compute 
	to avoid re-computing too many distances */
};

#endif