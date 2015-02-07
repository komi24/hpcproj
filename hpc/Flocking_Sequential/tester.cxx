#include "tester.hxx"

#include <iostream>
#include "octree.hxx"

void Tester::testConstruction(){
	Octree oc = Octree(0.5,1);
	
	Vector position(0, 0, 0);
	Agent a(position, Zeros(), Zeros());
    oc.add(a);


    Vector position1(0, 0.5, 0);    
	Agent a1(position1, Zeros(), Zeros());
	oc.add(a1);


	Vector position2(0, 0, 0.5);    
	Agent a2(position2, Zeros(), Zeros());
	oc.add(a2);

	Vector position3(1, 0.5, 0.5);
	Agent a3(position3, Zeros(), Zeros());
    oc.add(a3);

    printOctree(&oc);
    TemporaryContainer a,b,c;
    a.returnNeighbours(0.5,a,0,b,1,c);



    TemporaryContainer a,b,c;
    


}
void Tester::printContainter(TemporaryContainer &c){
	


	
}

/*void Tester::testConstruction2(){

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

  
}*/

void Tester::printOctree(Octree *oc){
	std::cout << " Tête " << std::endl ;
	for(int i=0; i<8; i++){
		if(oc->child[i]!=NULL){
				//std::cout << " feuille " << i << std::endl;
				printChild(oc->child[i],0);
			}else{
				//std::cout << " vide " << std::endl;
			}
//		std::cout << "	Fils " << i <<std::endl;
	}
}

void Tester::printChild(Octree *oc, int p){
	if(oc->agents.size()==0){
		for(int i=0; i<8; i++){
			if(oc->child[i]!=NULL){
				//for (int n=0; n<p; n++) std::cout << "	";
				//std::cout << " Suite " << std::endl;
				printChild(oc->child[i], p++);
			}else{
				//std::cout << " vide " << std::endl;
			}
//		std::cout << "	Fils " << i <<std::endl;
		}
	}else{
		std::cout << "Feuille Position " << oc->position << " Width " << oc->width << std::endl;
		for(TemporaryContainer::iterator it = oc->agents.begin(); it != oc->agents.end(); it++){
			//printChild(oc->child[i],i);
			//for (int n=0; n<p+1; n++) std::cout << "	";
			std::cout << "	Agent " << " position " << (*it)->position[Agent::curr_state] << std::endl;
		}
	}

}