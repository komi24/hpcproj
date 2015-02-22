#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
#include <omp.h>

#include "workspace.hxx"
#include "agent.hxx"
#include "vector.hxx"
#include "tester.hxx"
//#include "octree.hxx"


Workspace::Workspace(ArgumentParser &parser)
{

  na = parser("agents").asInt();

  wCohesion = parser("wc").asDouble();
  wAlignment = parser("wa").asDouble();
  wSeparation = parser("ws").asDouble();

  rCohesion = parser("rc").asDouble();
  rAlignment = parser("ra").asDouble();
  rSeparation = parser("rs").asDouble();
  dt= 0.05;
  maxU = 2.0;
  time = 0.,//;

  this->init();}

Workspace::Workspace(size_t nAgents,
             Real wc, Real wa, Real ws,
             Real rc, Real ra, Real rs) :
             na(nAgents), dt(.05), time(0),
             wCohesion(wc), wAlignment(wa), wSeparation(ws),
             rCohesion(rc), rAlignment(ra), rSeparation(rs),
             maxU(2.)
{ this->init();}

void  Workspace::init(){
    domainsize = 1.0;

    // Random generator seed
    srand48(std::time(0));

    //Initializing Octree head
    Real maxR;
    maxR = (rCohesion > rSeparation) ? rCohesion : rSeparation;
    maxR = (maxR > rAlignment) ? maxR : rAlignment;
    oc = *(new Octree(2*maxR,domainsize));

    // Initialize agents
    // This loop may be quite expensive due to random number generation
    //agents.reserve(na);
    //#pragma omp parallel
    //{
      //#pragma omp for
      for(size_t j = 0; j < na; j++){
      // Create random position
        Vector position(drand48(), drand48(), drand48());

      // Create random velocity
      //agents.push_back(Agent(position, Zeros(), Zeros()));
        agents.push_back(Agent(position, Zeros(), Zeros()));
        oc.add(agents[j]);
      }
    //}

    /*for(size_t j = 0; j < na; j++){
      // Create random position
      Vector position(drand48(), drand48(), drand48());

      // Create random velocity
      //agents.push_back(Agent(position, Zeros(), Zeros()));
      //agents.assign(j,Agent(position, Zeros(), Zeros()));
      oc.add(agents[j]);
    }*/

    /* TODO build the octree */

}

void Workspace::move(int step)//TODO erase step (just for tests)
{
  Vector s,c,a;
  TemporaryContainer bufS,bufC,bufA;

    #pragma omp parallel private(s,c,a,bufS,bufC,bufA)
  {
    /*int threadId = omp_get_thread_num(), nthread = omp_get_num_thread();
    int start = threadId *3*na /nthread;
    int end = ((threadId+1) * 3 * na / nthread > na) ? na : (threadId+1) * 3 * na / nthread;*/
    #pragma omp for
    for(size_t k = 0; k< na; k++){
      bufS.clear();
      bufC.clear();
      bufA.clear();
      //TODO "agents" argument should be only those which are close enough
      //it will then depends on k and on the radius needed.
      //std::cout << "ok1 " << k << " na " << na << std::endl; 
      agents[k].returnNeighbours(
        rSeparation, bufS,
        rCohesion, bufC,
        rAlignment, bufA);

//      std::cout << omp_get_thread_num() << std::endl;

      Tester tst;
     // tst.printContainer(bufC);
      //tst.printContainer(bufS);

      /* TODO no longer need of k */
      /*#pragma omp parallel num_threads(3)
      {
        #pragma omp sections
        {
        #pragma omp section
        {
        s = agents[k].separation(bufS, k, rSeparation);
        std::cout << "a" << omp_get_thread_num() << std::endl;
        }
        #pragma omp section
        {
          c = agents[k].cohesion(bufC, k, rCohesion);
          std::cout << "b" << omp_get_thread_num() << std::endl;
        }
        #pragma omp section
        {
          a = agents[k].alignment(bufA, k, rAlignment);
          std::cout << "c" << omp_get_thread_num() << std::endl;
        }}
      }*/
        s = agents[k].separation(bufS, k, rSeparation);
        c = agents[k].cohesion(bufC, k, rCohesion);
        a = agents[k].alignment(bufA, k, rAlignment);
/*      if(threadId%3==0){
        s = agents[k].separation(bufS, k, rSeparation);
      } elsif(threadId%3==1) {
        c = agents[k].cohesion(bufC, k, rCohesion);
      } else {
        a = agents[k].alignment(bufA, k, rAlignment);
      }*/
      //std::cout << " s : " << s << std::endl;
      //std::cout << " c : " << c << std::endl;
      //std::cout << " a : " << a << std::endl;

      agents[k].direction[1-Agent::curr_state] = wCohesion*c + wAlignment*a + wSeparation*s;
      
      agents[k].velocity[1-Agent::curr_state] = agents[k].velocity[Agent::curr_state] + agents[k].direction[1-Agent::curr_state];

      double speed = agents[k].velocity[1-Agent::curr_state].norm();
      if ((speed > maxU)) {
        agents[k].velocity[1-Agent::curr_state] = agents[k].velocity[1-Agent::curr_state] * maxU/speed;
        //std::cout << " maxU/speed " << maxU/speed <<std::endl;
      }

      agents[k].position[1-Agent::curr_state] = agents[k].position[Agent::curr_state] + dt*agents[k].velocity[Agent::curr_state];

      //std::cout << " direction " << agents[k].direction[Agent::curr_state] <<std::endl;
      //std::cout << " velocity " << agents[k].velocity[Agent::curr_state] <<std::endl;
      //std::cout << " speed " << speed <<std::endl;
      //std::cout << " position " << agents[k].position[Agent::curr_state] <<std::endl;

      agents[k].position[1-Agent::curr_state].x= fmod(agents[k].position[1-Agent::curr_state].x,domainsize);
      agents[k].position[1-Agent::curr_state].y= fmod(agents[k].position[1-Agent::curr_state].y,domainsize);
      agents[k].position[1-Agent::curr_state].z= fmod(agents[k].position[1-Agent::curr_state].z,domainsize);

    }
  }

    // Integration in time using euler method
    //TODO Remark for report : parallelism gain thx to curr_state
    Agent::curr_state = 1 - Agent::curr_state;
    /*for(size_t k = 0; k< na; k++){
      //std::cout << "ok2 " << k << " na " << na << std::endl; 
      agents[k].velocity[Agent::curr_state] = agents[k].velocity[1-Agent::curr_state] + agents[k].direction[Agent::curr_state];

      double speed = agents[k].velocity[Agent::curr_state].norm();
      if ((speed > maxU)) {
        agents[k].velocity[Agent::curr_state] = agents[k].velocity[Agent::curr_state] * maxU/speed;
        //std::cout << " maxU/speed " << maxU/speed <<std::endl;
      }

      agents[k].position[Agent::curr_state] = agents[k].position[1-Agent::curr_state] + dt*agents[k].velocity[1-Agent::curr_state];

      //std::cout << " direction " << agents[k].direction[Agent::curr_state] <<std::endl;
      //std::cout << " velocity " << agents[k].velocity[Agent::curr_state] <<std::endl;
      //std::cout << " speed " << speed <<std::endl;
      //std::cout << " position " << agents[k].position[Agent::curr_state] <<std::endl;

      agents[k].position[Agent::curr_state].x= fmod(agents[k].position[Agent::curr_state].x,domainsize);
      agents[k].position[Agent::curr_state].y= fmod(agents[k].position[Agent::curr_state].y,domainsize);
      agents[k].position[Agent::curr_state].z= fmod(agents[k].position[Agent::curr_state].z,domainsize);

      //agents[k].velocity[Agent::curr_state].x= fmod(agents[k].velocity[Agent::curr_state].x,10000);
      //agents[k].velocity[Agent::curr_state].y= fmod(agents[k].velocity[Agent::curr_state].y,10000);
      //agents[k].velocity[Agent::curr_state].z= fmod(agents[k].velocity[Agent::curr_state].z,10000);

    }*/

    //std::cout << "caca " << step << std::endl; 
    update();
    //std::cout << "caca " << step  << std::endl; 
    //std::cout << "state " << Agent::curr_state  << std::endl; 
}

void Workspace::move2(int step)//TODO erase step (just for tests)
{
    Vector s,c,a;
    TemporaryContainer bufS,bufC,bufA;

    //Run a smart foreage on leaves (having always neighbours' leaves)
    for(size_t k = 0; k< na; k++){
      bufS.clear();
      bufC.clear();
      bufA.clear();
      //TODO "agents" argument should be only those which are close enough
      //it will then depends on k and on the radius needed.
      //std::cout << "ok1 " << k << " na " << na << std::endl; 
      agents[k].returnNeighbours(
        rSeparation, bufS,
        rCohesion, bufC,
        rAlignment, bufA);

      Tester tst;
     // tst.printContainer(bufC);
      //tst.printContainer(bufS);

      /* TODO no longer need of k */
      //#pragma omp parallel sections
      //{
      //#pragma omp section
        //{
          s = agents[k].separation(bufS, k, rSeparation);
        //}
      //#pragma omp section
        //{
          c = agents[k].cohesion(bufC, k, rCohesion);
        //}
      //#pragma omp section
        //{
          a = agents[k].alignment(bufA, k, rAlignment);
        //}
      //}
      //std::cout << " s : " << s << std::endl;
      //std::cout << " c : " << c << std::endl;
      //std::cout << " a : " << a << std::endl;

      agents[k].direction[1-Agent::curr_state] = wCohesion*c + wAlignment*a + wSeparation*s;

    }

    // Integration in time using euler method
    //TODO Remark for report : parallelism gain thx to curr_state
    Agent::curr_state = 1 - Agent::curr_state;
    for(size_t k = 0; k< na; k++){
      //std::cout << "ok2 " << k << " na " << na << std::endl; 
      agents[k].velocity[Agent::curr_state] = agents[k].velocity[1-Agent::curr_state] + agents[k].direction[Agent::curr_state];

      double speed = agents[k].velocity[Agent::curr_state].norm();
      if ((speed > maxU)) {
        agents[k].velocity[Agent::curr_state] = agents[k].velocity[Agent::curr_state] * maxU/speed;
        //std::cout << " maxU/speed " << maxU/speed <<std::endl;
      }

      agents[k].position[Agent::curr_state] = agents[k].position[1-Agent::curr_state] + dt*agents[k].velocity[1-Agent::curr_state];

      //std::cout << " direction " << agents[k].direction[Agent::curr_state] <<std::endl;
      //std::cout << " velocity " << agents[k].velocity[Agent::curr_state] <<std::endl;
      //std::cout << " speed " << speed <<std::endl;
      //std::cout << " position " << agents[k].position[Agent::curr_state] <<std::endl;

      agents[k].position[Agent::curr_state].x= fmod(agents[k].position[Agent::curr_state].x,domainsize);
      agents[k].position[Agent::curr_state].y= fmod(agents[k].position[Agent::curr_state].y,domainsize);
      agents[k].position[Agent::curr_state].z= fmod(agents[k].position[Agent::curr_state].z,domainsize);

      //agents[k].velocity[Agent::curr_state].x= fmod(agents[k].velocity[Agent::curr_state].x,10000);
      //agents[k].velocity[Agent::curr_state].y= fmod(agents[k].velocity[Agent::curr_state].y,10000);
      //agents[k].velocity[Agent::curr_state].z= fmod(agents[k].velocity[Agent::curr_state].z,10000);

    }

    //std::cout << "caca " << step << std::endl; 
    update();
    //std::cout << "caca " << step  << std::endl; 
    //std::cout << "state " << Agent::curr_state  << std::endl; 
}

void Workspace::update(){
  //#pragma omp parallel for
  for(size_t k = 0; k< na; k++){
    Octree *lf = agents[k].leaf[1-Agent::curr_state];
    //Retirer de la liste si nécessaire et rajouter au bon endroit
    if((lf->position > agents[k].position[Agent::curr_state]) 
      || (agents[k].position[Agent::curr_state] >= (lf->position + Vector(1,1,1)*lf->width))) {
        lf->agents.erase(std::find(lf->agents.begin(),
          lf->agents.end(),
          &agents[k]));
        //lf->agents.remove(&agents[k]);
        lf->delete_leaves();
        oc.add(agents[k]);
      } else {
        agents[k].leaf[Agent::curr_state]=lf;
      }
    }
  }


void Workspace::simulate(int nsteps) {
  // store initial position[Agent::curr_state]s
    save(0);
    Tester tst;

    // perform nsteps time steps of the simulation
    int step = 0;
    while (step++ < nsteps) {
    //std::cout << "coco" << step << std::endl; 
      this->move(step);
      //tst.printOctree(& this->oc);
      // store every 20 steps
      if (step%20 == 0) save(step);
    }
}

void Workspace::save(int stepid) {
  std::ofstream myfile;

  myfile.open("boids2.xyz", stepid==0 ? std::ios::out : std::ios::app);

    myfile << std::endl;
    myfile << na << std::endl;
    for (size_t p=0; p<na; p++)
        myfile << "B " << agents[p].position[Agent::curr_state];

    myfile.close();
  }
