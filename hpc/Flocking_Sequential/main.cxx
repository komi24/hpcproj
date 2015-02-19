
// Agent (particle model)

//#include "types.hxx"
#include "workspace.hxx"
#include "parser.hxx"
#include "tester.hxx"
//#include "agent.hxx"

// Main class for running the parallel flocking sim
int main(int argc, char **argv) {
  // Create parser
  //Tester t;
  //t.testConstruction();
  //t.testUpdate();
  
  ArgumentParser parser;
  Tester tst;

  // Add options to parser
  parser.addOption("agents", 640);//640 originally
  parser.addOption("steps", 500);//500
  parser.addOption("wc", 6);
  parser.addOption("wa", 15);
  parser.addOption("ws", 35);

  parser.addOption("rc", 0.21);
  parser.addOption("ra", 0.25);
  parser.addOption("rs", 0.01);

  // Parse command line arguments
  parser.setOptions(argc, argv);

  // Create workspace
  Workspace workspace(parser);


  // Launch simulation
  int nSteps = parser("steps").asInt();
  workspace.simulate(nSteps);

  return 0;
}
