/*
*/
#ifndef  WORKSPACE
#define  WORKSPACE

#include "parser.hxx"
#include "types.hxx"
#include "octree.hxx"


class Workspace
{
protected:
  Container agents;
  /* Spatial data structure for agent search */
  Octree oc;
  TemporaryContainer a,b,c;
  unsigned int na;

  Real dt;
  int time;
  Real wCohesion, wAlignment, wSeparation;
  Real rCohesion, rAlignment, rSeparation;
  Real maxU;

  Real tUpload, tDownload, tCohesion, tAlignment, tSeparation;

  Real domainsize;
  void init();
public:
  Workspace(ArgumentParser &parser);

  Workspace(size_t nAgents,
  Real wc, Real wa, Real ws,
  Real rc, Real ra, Real rs);

  void move();
  void simulate(int nsteps);
  void save(int stepid);
};

#endif