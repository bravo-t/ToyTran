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

Plotter::Plotter(const NetlistParser& parser, const Circuit& ckt, const SimResult& result)
 : _parser(parser), _circuit(ckt), _result(result) {}

std::vector<std::pair<double, double>>
simData(const SimResult& result, size_t rowIndex, double& max, double& min)
{
  std::vector<std::pair<double, double>> data;
  max = std::numeric_limits<double>::lowest();
  min = std::numeric_limits<double>::max();
  size_t resultVectorSize = result.indexMap().size();
  for (size_t tIndex=0; tIndex<result.ticks().size(); ++tIndex) {
    size_t valueIndex = tIndex*resultVectorSize+rowIndex;
    double value = result.value(valueIndex);
    if (!std::isnan(value) && !std::isinf(value)) {
      max = std::max(max, value);
      min = std::min(min, value);
      data.push_back(std::make_pair(result.tick(tIndex), value));
    }
  }
  return data;
}

typedef std::pair<double, double> SimTimeData;

std::vector<SimTimeData>
nodeSimData(const SimResult& result, const std::string& nodeName, 
            const Circuit& ckt, double& max, double& min) 
{
  const Node& node = ckt.findNodeByName(nodeName);
  if (node._nodeId == static_cast<size_t>(-1)) {
    printf("Node %s not found\n", nodeName.data());
    return std::vector<std::pair<double, double>>();
  }
  size_t rowIndex = result.nodeVectorIndex(node._nodeId);
  return simData(result, rowIndex, max, min);
}

std::vector<SimTimeData>
deviceSimData(const SimResult& result, const std::string& devName, 
              const Circuit& ckt, double& max, double& min) 
{
  const Device& device = ckt.findDeviceByName(devName);
  if (device._devId == static_cast<size_t>(-1)) {
    printf("Device %s not found\n", devName.data());
    return std::vector<std::pair<double, double>>();
  }
  size_t rowIndex = result.deviceVectorIndex(device._devId);
  return simData(result, rowIndex, max, min);
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
plotData(const std::vector<std::pair<double, double>>& data, 
         double max, double min, 
         std::vector<std::string>& canvas, char marker)
{
  size_t width = canvas[0].size() - 1;
  size_t height = canvas.size() - 2;
  double dataScale = (max - min) / height;
  double timeScale = data.back().first / width;
  for (const auto& point : data) {
    double offsetValue = point.second - min;
    size_t y = offsetValue / dataScale;
    y = height - y;
    size_t x = point.first / timeScale;
    //printf("DEBUG: y value: %g, y div: %lu, x value: %g, x div: %lu, dataScale: %g, timeScale: %g\n", 
    //  point.second, y, point.first, x, dataScale, timeScale);
    canvas[y][x] = marker;
  }
}

void
Plotter::plot(const PlotData& data) const
{
  double max = std::numeric_limits<double>::lowest();
  double min = std::numeric_limits<double>::max();
  
  std::vector<char> markers = {'*', 'o', 'x', '+'};
  std::vector<std::string> legend;

  size_t plotCounter = 0;
  std::vector<std::vector<SimTimeData>> simData;
  for (const std::string& nodeName : data._nodeToPlot) {
    double nodeMax = 0;
    double nodeMin = 0;
    const std::vector<SimTimeData>& nodeData = nodeSimData(_result, nodeName, _circuit, nodeMax, nodeMin);
    simData.push_back(nodeData);
    max = std::max(max, nodeMax);
    min = std::min(min, nodeMin);
    std::string l(1, markers[plotCounter]);
    l += ": Voltage of node ";
    l += nodeName;
    legend.push_back(l);
    ++plotCounter;
  }
  for (const std::string& devName : data._deviceToPlot) {
    double devMax = 0;
    double devMin = 0;
    const std::vector<SimTimeData>& devData = deviceSimData(_result, devName, _circuit, devMax, devMin);
    simData.push_back(devData);
    max = std::max(max, devMax);
    min = std::min(min, devMin);
    std::string l(1, markers[plotCounter]);
    l += ": Current of device ";
    l += devName;
    legend.push_back(l);
    ++plotCounter;
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
Plotter::plotNodeVoltage(const std::string& nodeName, const Circuit& ckt, const SimResult& result) const
{
  double max = 0;
  double min = 0;
  auto data = nodeSimData(result, nodeName, ckt, max, min);
  std::vector<std::string> canvas;
  initCanvas(_parser.plotWidth(), _parser.plotHeight(), canvas);
  plotData(data, max, min, canvas, '*');
  for (const std::string& line : canvas) {
    printf("%s\n", line.data());
  }
  printf("  Voltage of node %s\n", nodeName.data());
}

void 
Plotter::plotDeviceCurrent(const std::string& devName, const Circuit& ckt, const SimResult& result) const
{
  double max = 0;
  double min = 0;
  auto data = deviceSimData(result, devName, ckt, max, min);
  std::vector<std::string> canvas;
  initCanvas(_parser.plotWidth(), _parser.plotHeight(), canvas);
  plotData(data, max, min, canvas, '*');
  for (const std::string& line : canvas) {
    printf("%s\n", line.data());
  }
  printf("  Current of device %s\n", devName.data());
}

void
Plotter::plot() const
{
  const std::vector<PlotData>& plotCmds = _parser.plotData();

  for (const PlotData& cmd : plotCmds) {
    if (cmd._canvasName == "") {
      for (const std::string& nodeName : cmd._nodeToPlot) {
        plotNodeVoltage(nodeName, _circuit, _result);
      }
      for (const std::string& devName : cmd._deviceToPlot) {
        plotDeviceCurrent(devName, _circuit, _result);
      }
    } else {
      plot(cmd);
    }
  }

}



}