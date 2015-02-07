#ifndef TESTER
#define TESTER

class Octree;

class Tester{
public:
	void testConstruction();
	//void testConstruction2();
	void printOctree(Octree *oc);
	void printChild(Octree *oc, int p);
};

#endif