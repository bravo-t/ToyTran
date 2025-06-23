#include <sys/ioctl.h>
#include <cstdio>
#include <unistd.h>
#include <algorithm>
#include "Plotter.h"
#include "NetlistParser.h"
#include "Circuit.h"
#include "Simulator.h"

namespace NA {

static unsigned int widthLimit = 200;
static unsigned int heightLimit = 100;

static void
terminalSize(size_t& width, size_t& height)
{
  struct winsize w;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
  width = w.ws_col > widthLimit ? widthLimit : w.ws_col;
  height = w.ws_row > heightLimit ? heightLimit : w.ws_row;
}

Plotter::Plotter(const NetlistParser& parser, const std::vector<Circuit>& ckts, 
                 const std::vector<SimResult>& results)
 : _parser(parser), _circuits(ckts), _results(results) {}


const Circuit*
findCircuitByName(const std::vector<Circuit>& ckts, const std::string& simName)
{
  for (const Circuit& ckt : ckts) {
    if (ckt.simName() == simName) {
      return &ckt;
    }
  }
  return nullptr;
}

void
initCanvas(size_t width, size_t height, 
           std::vector<std::string>& canvas)
{
  if (width == static_cast<size_t>(-1) || height == static_cast<size_t>(-1)) {
    terminalSize(width, height);
    if (width == 0 || height == 0) {
      return;
    }
  }
  canvas.reserve(height);
  for (size_t i=0; i<height-1; ++i) {
    std::string line(width, ' ');
    line[0] = '|';
    canvas.push_back(line);
  }
  std::string coor(width, '-');
  coor[0] = '|';
  canvas.push_back(coor);
}

void 
plotData(const std::vector<WaveformPoint>& data, 
         double max, double min, 
         std::vector<std::string>& canvas, char marker)
{
  size_t width = canvas[0].size() - 1;
  size_t height = canvas.size() - 2;
  double dataScale = (max - min) / height;
  double timeScale = data.back()._time / width;
  for (const auto& point : data) {
    double offsetValue = point._value - min;
    size_t y = offsetValue / dataScale;
    y = height - y;
    size_t x = point._time / timeScale;
    //printf("DEBUG: y value: %g, y div: %lu, x value: %g, x div: %lu, dataScale: %g, timeScale: %g\n", 
    //  point.second, y, point.first, x, dataScale, timeScale);
    canvas[y][x] = marker;
  }
}

const SimResult*
findResultByName(const std::vector<SimResult>& results, const std::string& name)
{
  for (const auto& result : results) {
    if (result.name() == name) {
      return &(result);
    }
  }
  return nullptr;
}

void
Plotter::plot(const PlotData& data) const
{
  double max = std::numeric_limits<double>::lowest();
  double min = std::numeric_limits<double>::max();
  
  std::vector<char> markers = {'*', 'o', 'x', '+'};
  std::vector<std::string> legend;

  size_t plotCounter = 0;
  std::vector<std::vector<WaveformPoint>> simData;
  for (size_t i=0; i<data._nodeToPlot.size(); ++i) {
    const std::string& nodeName = data._nodeToPlot[i];
    const std::string& simName = data._nodeSimName[i];
    const SimResult* result = findResultByName(_results, simName);
    if (result == nullptr) {
      printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
      return;
    }
    const Circuit* ckt = findCircuitByName(_circuits, simName);
    if (ckt == nullptr) {
      printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
      return;
    }
    double nodeMax = 0;
    double nodeMin = 0;
    const std::vector<WaveformPoint>& nodeData = result->nodeVoltageWaveform(nodeName, nodeMax, nodeMin).data();
    if (nodeData.size() > 0) {
      simData.push_back(nodeData);
      max = std::max(max, nodeMax);
      min = std::min(min, nodeMin);
      std::string l(1, markers[plotCounter]);
      l += ": Voltage of node ";
      l += nodeName;
      legend.push_back(l);
      ++plotCounter;
    }
  }
  for (size_t i=0; i<data._deviceToPlot.size(); ++i) {
    const std::string& devName = data._deviceToPlot[i];
    const std::string& simName = data._devSimName[i];
    const SimResult* result = findResultByName(_results, simName);
    if (result == nullptr) {
      printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
      return;
    }
    const Circuit* ckt = findCircuitByName(_circuits, simName);
    if (ckt == nullptr) {
      printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
      return;
    }
    double devMax = 0;
    double devMin = 0;
    const std::vector<WaveformPoint>& devData = result->deviceCurrentWaveform(devName, devMax, devMin).data();
    if (devData.size() > 0) {
      simData.push_back(devData);
      max = std::max(max, devMax);
      min = std::min(min, devMin);
      std::string l(1, markers[plotCounter]);
      l += ": Current of device ";
      l += devName;
      legend.push_back(l);
      ++plotCounter;
    }
  }
  std::vector<std::string> canvas;
  initCanvas(_parser.plotWidth(), _parser.plotHeight(), canvas);
  for (size_t i=0; i<simData.size(); ++i) {
    char marker = markers[i];
    plotData(simData[i], max, min, canvas, marker);
  }
  for (const std::string& line : canvas) {
    printf("%s\n", line.data());
  }
  for (const std::string& line : legend) {
    printf("  %s\n", line.data());
  }
}

void
Plotter::plotNodeVoltage(const std::string& nodeName, const std::string& simName, 
                         const std::vector<SimResult>& results) const
{
  const SimResult* result = findResultByName(results, simName);
  if (result == nullptr) {
    printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
    return;
  }
  double max = 0;
  double min = 0;
  const std::vector<WaveformPoint>& data = result->nodeVoltageWaveform(nodeName, max, min).data();
  if (data.size() > 0) {
    std::vector<std::string> canvas;
    initCanvas(_parser.plotWidth(), _parser.plotHeight(), canvas);
    plotData(data, max, min, canvas, '*');
    for (const std::string& line : canvas) {
      printf("%s\n", line.data());
    }
    printf("  Voltage of node %s\n", nodeName.data());
  }
}

void 
Plotter::plotDeviceCurrent(const std::string& devName, const std::string& simName, 
                           const std::vector<SimResult>& results) const
{
  const SimResult* result = findResultByName(results, simName);
  if (result == nullptr) {
    printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
    return;
  }
  double max = 0;
  double min = 0;
  const std::vector<WaveformPoint>& data = result->deviceCurrentWaveform(devName, max, min).data();
  if (data.size() > 0) {
    std::vector<std::string> canvas;
    initCanvas(_parser.plotWidth(), _parser.plotHeight(), canvas);
    plotData(data, max, min, canvas, '*');
    for (const std::string& line : canvas) {
      printf("%s\n", line.data());
    }
    printf("  Current of device %s\n", devName.data());
  }
}

void
Plotter::plot() const
{
  const std::vector<PlotData>& plotCmds = _parser.plotData();

  for (const PlotData& cmd : plotCmds) {
    if (cmd._canvasName == "") {
      for (size_t i=0; i<cmd._nodeToPlot.size(); ++i) {
        const std::string& nodeName = cmd._nodeToPlot[i];
        const std::string& simName = cmd._nodeSimName[i];
        const Circuit* ckt = findCircuitByName(_circuits, simName);
        if (ckt == nullptr) {
          printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
          continue;
        }
        plotNodeVoltage(nodeName, simName, _results);
      }
      for (size_t i=0; i<cmd._deviceToPlot.size(); ++i) {
        const std::string& devName = cmd._deviceToPlot[i];
        const std::string& simName = cmd._devSimName[i];
        const Circuit* ckt = findCircuitByName(_circuits, simName);
        if (ckt == nullptr) {
          printf("Plot ERROR: Analysis named \"%s\" does not exist\n", simName.data());
          continue;
        }
        plotDeviceCurrent(devName, simName, _results);
      }
    } else {
      plot(cmd);
    }
  }

}



}
