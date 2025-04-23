#include <cstdio>
#include <ctime>
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"
#include "PoleZero.h"
#include "TR0Writer.h"
#include "Plotter.h"
#include "Measure.h"

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

  timespec parseStart;
  clock_gettime(CLOCK_REALTIME, &parseStart);
  NA::NetlistParser parser(argv[1]);
  timespec parseEnd;
  clock_gettime(CLOCK_REALTIME, &parseEnd);
  timespec cktStart;
  clock_gettime(CLOCK_REALTIME, &cktStart);
  NA::Circuit circuit(parser);
  timespec cktEnd;
  clock_gettime(CLOCK_REALTIME, &cktEnd);
  printf("Time spent in netlist parsing: %.3f milliseconds\n"
         "Time spent in building circuit: %.3f milliseconds\n", 
         1e-6*timeDiffNs(parseEnd, parseStart), 1e-6*timeDiffNs(cktEnd, cktStart));

  std::vector<NA::SimResult> results;
  const std::vector<NA::AnalysisParameter>& params = parser.analysisParameters();
  for (const NA::AnalysisParameter& param : params) {
    switch (param._type) {
      case NA::AnalysisType::Tran: {
        NA::Simulator tranSim(circuit);
        tranSim.setParameters(param);
        printf("Starting transient simulation\n");
        timespec start;
        clock_gettime(CLOCK_REALTIME, &start);
        tranSim.run();
        timespec end;
        clock_gettime(CLOCK_REALTIME, &end);
        printf("Simulation finished, %lu steps simulated in %.3f seconds\n", 
               tranSim.simulationResult().size(), 1e-9*timeDiffNs(end, start));
        results.push_back(tranSim.simulationResult());
        if (parser.dumpData()) {
          std::string tr0File;
          if (argc > 3) {
            tr0File = argv[2];
          } else {
            tr0File = fileNameWithoutSuffix(argv[1]);
            tr0File += ".tr0";
          }
          printf("Writing simulation data to %s\n", tr0File.data());
          NA::TR0Writer writer(circuit, tr0File);
          writer.adjustNumberWidth(param._simTick, param._simTime);
          writer.writeData(tranSim.simulationResult());
        }
        break;
      }
      case NA::AnalysisType::PZ: {
        NA::PoleZeroAnalysis pz(circuit, param);
        pz.run();
        results.push_back(pz.result());
        delete param._inNode;
        delete param._outNode;
        break;
      }
      default:
        // Do nothing for now
        break;
    }
  }

  for (const NA::SimResult& result : results) {
    if (parser.needPlot()) {
      NA::Plotter plt(parser, circuit, result);
      plt.plot();
    }

    if (parser.haveMeasurePoints()) {
      NA::Measure measure(result, parser.measurePoints());
      measure.run();
    }

  } 

  return 0;
}
