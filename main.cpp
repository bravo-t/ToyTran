#include <cstdio>
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"

int main(int argc, char** argv) 
{
  if (argc == 1) {
    printf("Input file missing, please provide a circuit netlist\n");
    return 1;
  }
  Tran::NetlistParser parser(argv[1]);
  Tran::Circuit circuit(parser);
  Tran::Simulator simulator(circuit);
  simulator.setSimulationEndTime(simulator.simulationTick()*10);
  simulator.run();
  return 0;
}
