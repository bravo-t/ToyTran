#include <cstdio>
#include <ctime>
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"
#include "TR0Writer.h"
#include "Plotter.h"

inline uint64_t
timeDiffNs(const timespec& endTime, const timespec& startTime)
{
  return (endTime.tv_sec - startTime.tv_sec) * 1e9 + 
         (endTime.tv_nsec - startTime.tv_nsec);
}

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

  Tran::NetlistParser parser(argv[1]);
  Tran::Circuit circuit(parser);
  Tran::Simulator simulator(circuit);
  simulator.setSimTick(parser.simulationTick());
  simulator.setSimulationEndTime(parser.simulationTime());
  simulator.setIntegrateMethod(parser.integrateMethod());
  printf("Starting transient simulation\n");
  
  timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  simulator.run();
  timespec end;
  clock_gettime(CLOCK_REALTIME, &end);
  printf("Simulation finished, %lu steps simulated in %.3f seconds\n", 
         simulator.simulationResult().size(), 1e-9*timeDiffNs(end, start));

  if (parser.dumpData()) {
    std::string tr0File;
    if (argc > 3) {
      tr0File = argv[2];
    } else {
      tr0File = fileNameWithoutSuffix(argv[1]);
      tr0File += ".tr0";
    }
    printf("Writing simulation data to %s\n", tr0File.data());
    Tran::TR0Writer writer(circuit, tr0File);
    writer.adjustNumberWidth(parser.simulationTick(), parser.simulationTime());
    writer.writeData(simulator.simulationResult());
  }

  if (parser.needPlot()) {
    Tran::Plotter plt(parser, circuit, simulator.simulationResult());
    plt.plot();
  }

  return 0;
}
