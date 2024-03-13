#include <cstdio>
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"
#include "TR0Writer.h"

std::string
fileNameWithoutSuffix(const char* fname)
{
  size_t nameLength = strlen(fname);
  size_t nameEnd = 0;
  for (size_t i=0; i<nameLength; ++i) {
    nameEnd = i;
    if (fname[i] == '.') {
      break;
    }
  }
  return std::string(fname, nameEnd);
}

int main(int argc, char** argv) 
{
  if (argc < 2) {
    printf("Input file missing, please provide a circuit netlist\n");
    return 1;
  }
  std::string tr0File;
  if (argc > 3) {
    tr0File = argv[2];
  } else {
    tr0File = fileNameWithoutSuffix(argv[1]);
    tr0File += ".tr0";
  }

  Tran::NetlistParser parser(argv[1]);
  printf("Transient simualtion will be run for %g seconds with %g step\n", 
    parser.simulationTime(), parser.simulationTick());
  Tran::Circuit circuit(parser);
  Tran::Simulator simulator(circuit);
  simulator.setSimulationEndTime(parser.simulationTime());
  simulator.setSimTick(parser.simulationTick());
  simulator.run();
  printf("Simulation finished, writing data to %s\n", tr0File.data());

  Tran::TR0Writer writer(circuit, tr0File);
  writer.adjustNumberWidth(parser.simulationTick(), parser.simulationTime());
  writer.writeData(simulator.simulationResult());

  return 0;
}
