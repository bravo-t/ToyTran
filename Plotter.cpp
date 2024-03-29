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

Plotter::Plotter(const NetlistParser& parser, const Circuit& ckt, const SimResult& result)
 : _parser(parser), _circuit(ckt), _result(result) {}

std::vector<std::pair<double, double>>
simData(const SimResult& result, size_t rowIndex, double& max, double& min)
{
  std::vector<std::pair<double, double>> data;
  max = std::numeric_limits<double>::lowest();
  min = std::numeric_limits<double>::max();
  size_t resultVectorSize = result._map.size();
  for (size_t tIndex=0; tIndex<result._ticks.size(); ++tIndex) {
    size_t valueIndex = tIndex*resultVectorSize+rowIndex;
    double value = result._values[valueIndex];
    if (!std::isnan(value) && !std::isinf(value)) {
      max = std::max(max, value);
      min = std::min(min, value);
      data.push_back(std::make_pair(result._ticks[tIndex], value));
    }
  }
  return data;
}

std::vector<std::pair<double, double>>
nodeSimData(const SimResult& result, const std::string& nodeName, 
            const Circuit& ckt, double& max, double& min) 
{
  const Node& node = ckt.findNodeByName(nodeName);
  if (node._nodeId == static_cast<size_t>(-1)) {
    printf("Node %s not found\n", nodeName.data());
    return std::vector<std::pair<double, double>>();
  }
  size_t rowIndex = result.nodeVectorIndex(node._nodeId);
  printf("Node %s id %lu rowIndex: %lu\n", node._name.data(), node._nodeId, rowIndex);
  return simData(result, rowIndex, max, min);
}

std::vector<std::pair<double, double>>
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
plotData(const std::vector<std::pair<double, double>>& data, 
         double max, double min)
{
  if (data.empty()) {
    return;
  }
  size_t width = 0;
  size_t height = 0;
  terminalSize(width, height);
  if (width == 0 || height == 0) {
    return;
  }

  std::vector<std::string> canvas;
  canvas.resize(height);
  for (size_t i=0; i<height; ++i) {
    if (i == height - 1) {
      canvas[i].resize(width, '-');
    } else {
      canvas[i].resize(width, ' ');
    }
    canvas[i][0] = '|';
  }
  width -= 1;
  height -= 2;
  double dataScale = (max - min) / height;
  double timeScale = data.back().first / width;
  for (const auto& point : data) {
    double offsetValue = point.second - min;
    size_t y = offsetValue / dataScale;
    size_t x = point.first / timeScale;
    //printf("y value: %g, y div: %lu, x value: %g, x div: %lu, dataScale: %g, timeScale: %g\n", 
    //  offsetValue, y, point.first, x, dataScale, timeScale);
    canvas[y][x] = '*';
  }

  for (const auto& line : canvas) {
    printf("%s\n", line.data());
  }
}

void
plotNodeVoltage(const std::string& nodeName, const Circuit& ckt, const SimResult& result)
{
  double max = 0;
  double min = 0;
  auto data = nodeSimData(result, nodeName, ckt, max, min);
  plotData(data, max, min);
  //printf("Node %s, data size: %lu, max: %g, min: %g, last time: %g\n", nodeName.data(), data.size(), max, min, data.back().first);
  printf("  Voltage of node %s\n", nodeName.data());
}

void plotDeviceCurrent(const std::string& devName, const Circuit& ckt, const SimResult& result)
{
  double max = 0;
  double min = 0;
  auto data = deviceSimData(result, devName, ckt, max, min);
  plotData(data, max, min);
  printf("  Current of device %s\n", devName.data());
}

void
Plotter::plot()
{
  const std::vector<std::string>& nodes = _parser.nodesToPlot();
  const std::vector<std::string>& devices = _parser.devicesToPlot();

  for (const std::string& nodeName : nodes) {
    plotNodeVoltage(nodeName, _circuit, _result);
  }
  for (const std::string& devName : devices) {
    plotDeviceCurrent(devName, _circuit, _result);
  }
}



}