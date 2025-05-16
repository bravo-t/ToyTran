#include <cstdio>
#include "NetworkAnalyzer.h"
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"
#include "PoleZero.h"
#include "TR0Writer.h"
#include "Plotter.h"
#include "Measure.h"
#include "Timer.h"

namespace NA {

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

void
NetworkAnalyzer::run(const char* inFile) 
{
  NA::NetlistParser parser(inFile);

  std::vector<NA::Circuit> circuits;
  std::vector<NA::SimResult> results;
  const std::vector<NA::AnalysisParameter>& params = parser.analysisParameters();
  for (const NA::AnalysisParameter& param : params) {
    NA::Circuit circuit(parser, param);
    circuits.push_back(circuit);
    switch (param._type) {
      case NA::AnalysisType::Tran: {
        NA::Simulator tranSim(circuit, param);
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
          tr0File = fileNameWithoutSuffix(inFile);
          tr0File += ".tr0";
          printf("Writing simulation data to %s\n", tr0File.data());
          NA::TR0Writer writer(circuit, tr0File);
          writer.adjustNumberWidth(param._simTick, param._simTime);
          writer.writeData(tranSim.simulationResult());
        }
        if (param._hasMeasurePoints) {
          NA::Measure measure(tranSim.simulationResult(), parser.measurePoints(param._name));
          measure.run();
        }
        break;
      }
      case NA::AnalysisType::PZ: {
        NA::PoleZeroAnalysis pz(circuit, param);
        pz.run();
        results.push_back(pz.result());
        if (param._hasMeasurePoints) {
          NA::Measure measure(pz.result(), parser.measurePoints(param._name));
          measure.run();
        }
        break;
      }
      default:
        // Do nothing for now
        break;
    }
  }

  if (parser.needPlot()) {
    NA::Plotter plt(parser, circuits, results);
    plt.plot();
  }

}

}
