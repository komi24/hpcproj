#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>

#include "agent.hxx"
#include "vector.hxx"
#include "workspace.hxx"

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
  time = 0.,

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

    // Initialize agents
    // This loop may be quite expensive due to random number generation
    for(size_t j = 0; j < na; j++){
      // Create random position
      Vector position(drand48(), drand48(), drand48());

      // Create random velocity
      agents.push_back(Agent(position, Zeros(), Zeros()));
    }

    /* TODO build the octree */

}

void Workspace::move()
{
    Vector s,c,a;

    for(size_t k = 0; k< na; k++){
      //TODO "agents" argument should be only those which are close enough
      //it will then depends on k and on the radius needed.
      oc.returnNeighbours(agents[k],
        rSeparation, TemporaryContainer &a,
        rCohesion, TemporaryContainer &b,
        rAlignment, TemporaryContainer &c);

      /* TODO no longer need of k */
      s = agents[k].separation(agents, k, rSeparation);
      c = agents[k].cohesion(agents, k, rCohesion);
      a = agents[k].alignment(agents, k, rAlignment);

      agents[k].direction = wCohesion*c + wAlignment*a + wSeparation*s;
    }

    // Integration in time using euler method
    Agent.curr_state = 1 - Agent.curr_state;
    for(size_t k = 0; k< na; k++){
      agents[k].velocity[curr_state] = agents[k].velocity[1-curr_state] + agents[k].direction;

      double speed = agents[k].velocity.norm();
      if (speed > maxU) {
        agents[k].velocity[curr_state] = agents[k].velocity[1-curr_state] * maxU/speed;
      }
      agents[k].position[Agent.curr_state] = agents[k].position[1-Agent.curr_state] + dt*agents[k].velocity[1-Agent.curr_state];

      agents[k].position[Agent.curr_state].x= fmod(agents[k].position[Agent.curr_state].x,domainsize);
      agents[k].position[Agent.curr_state].y= fmod(agents[k].position[Agent.curr_state].y,domainsize);
      agents[k].position[Agent.curr_state].z= fmod(agents[k].position[Agent.curr_state].z,domainsize);

    }
}

void Workspace::simulate(int nsteps) {
  // store initial position[Agent.curr_state]s
    save(0);

    // perform nsteps time steps of the simulation
    int step = 0;
    while (step++ < nsteps) {
      this->move();
      // store every 20 steps
      if (step%20 == 0) save(step);
    }
}

void Workspace::save(int stepid) {
  std::ofstream myfile;

  myfile.open("boids.xyz", stepid==0 ? std::ios::out : std::ios::app);

    myfile << std::endl;
    myfile << na << std::endl;
    for (size_t p=0; p<na; p++)
        myfile << "B " << agents[p].position[Agent.curr_state];

    myfile.close();
  }
