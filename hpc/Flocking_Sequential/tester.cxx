#include "tester.hxx"

#include <iostream>
#include "octree.hxx"

void Tester::testConstruction(){
}

void Tester::printOctree(Octree *oc){
	std::cout << "Tête" ;
	for(int i=0, i<8, i++){
		//pintOctree(oc->child[i]);
		std::cout << "	Fils " << i <<endl;
	}
}