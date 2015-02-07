#include "tester.hxx"

#include <iostream>
#include "octree.hxx"

void Tester::testConstruction(){
	Octree oc = Octree(0.5,1);
	
	Vector position(0, 0, 0);    
    oc.add(Agent(position, Zeros(), Zeros()));

    Vector position(0, 0.5, 0);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0.5, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

    printOctree(&oc);

}


void Tester::testConstruction2(){

	Octree oc = Octree(0.5,1);
	
	Vector position(0, 0, 0); 
	Agent *a1 =  new Agent(position, Zeros(), Zeros())
    oc.add(*a1);

    Vector position(0, 0.5, 0);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0.5, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

    printOctree(&oc);

  
}

void Tester::printOctree(Octree *oc){
	std::cout << "Tête" ;
	for(int i=0, i<8, i++){
		//pintOctree(oc->child[i]);
		std::cout << "	Fils " << i <<endl;
	}
}