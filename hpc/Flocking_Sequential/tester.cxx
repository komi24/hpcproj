#include "tester.hxx"

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


    TemporaryContainer a,b,c;
    


}


void Tester::testConstruction2(){

	Octree oc = Octree(0.5,1);
	
	Vector position(0, 0, 0); 
	Agent *a1 =  new Agent(position, Zeros(), Zeros());
    oc.add(*a1);

    Vector position(0, 0.5, 0);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

	Vector position(0, 0.5, 0.5);    
    oc.add(Agent(position, Zeros(), Zeros()));

    printOctree(&oc);

  
}

void Tester::printOctree(){
}