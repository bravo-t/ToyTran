#include <sys/ioctl.h>
#include <cstdio>
#include <unistd.h>
#include "Plotter.h"
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"

namespace Tran {

static void
terminalSize(size_t& width, size_t& height)
{
  struct winsize w;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
  width = w.ws_col;
  height = w.ws_row;
}

Plotter::Plotter(const NetlistParser& parser, const Circuit& ckt, const SimulatorResult& result)
 : _parser(parser), _circuit(ckt), _result(result) {}

std::vector<std::pair<double, double>>
simData(const SimulatorResult& result, size_t rowIndex, double& max)
{
  std::vector<std::pair<double, double>> data;
  max = std::numerical_limits<double>::lowest();
  size_t resultVectorSize = result._map.size();
  for (size_t tIndex=0; t<result._ticks.size(); ++tIndex) {
    size_t valueIndex = tIndex*resultVectorSize+rowIndex;
    double value = result._values[valueIndex];
    if (!std::isnan(value) && !std::isinf(value)) {
      max = std::max(max, value);
      data.push_back(std::make_pair(result._ticks[tIndex], value));
    }
  }
}




}