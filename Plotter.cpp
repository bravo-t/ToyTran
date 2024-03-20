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
simData(const SimulatorResult& result, size_t rowIndex, double& max, double& min, double& tick)
{
  std::vector<std::pair<double, double>> data;
  max = std::numerical_limits<double>::lowest();
  min = std::numerical_limits<double>::max();
  tick = std::numerical_limits<double>::max();
  prevTick = std::numerical_limits<double>::max();
  size_t resultVectorSize = result._map.size();
  for (size_t tIndex=0; t<result._ticks.size(); ++tIndex) {
    size_t valueIndex = tIndex*resultVectorSize+rowIndex;
    double value = result._values[valueIndex];
    if (!std::isnan(value) && !std::isinf(value)) {
      max = std::max(max, value);
      max = std::min(min, value);
      if (prevTick == std::numerical_limits<double>::max()) {
        prevTick = result._ticks[tIndex];
      } else {
        double interval = reult._ticks[tIndex] - prevTick;
        tick = std::min(tick, interval);
        prevTick = result._ticks[tIndex];
      }
      data.push_back(std::make_pair(result._ticks[tIndex], value));
    }
  }
}

std::vector<std::pair<double, double>>
nodeSimData(const SimulatorResult& result, const std::string& nodeName, 
            const Circuit& ckt, double& max, double& min, double& tick) 
{
  const Node& node = ckt.findNodeByName(nodeName);
  if (node._nodeId == static_cast<size_t>(-1)) {
    printf("Node %s not found\n", nodeName.data());
    return std::vector<std::pair<double, double>>();
  })
  size_t rowIndex = result.nodeVectorIndex(node._nodeId);
  return simData(result, rowIndex, max, min, tick);
}

std::vector<std::pair<double, double>>
deviceSimData(const SimulatorResult& result, const std::string& devName, 
              const Circuit& ckt, double& max, double& min, double& tick) 
{
  const Device& device = ckt.findDeviceByName(devName);
  if (device._devId == static_cast<size_t>(-1)) {
    printf("Device %s not found\n", devName.data());
    return std::vector<std::pair<double, double>>();
  })
  size_t rowIndex = result.deviceVectorIndex(device._devId);
  return simData(result, rowIndex, max, min, tick);
}

void 
plotData(const std::vector<std::pair<double, double>>& data, 
         double max, double min, double tick)
{
  if (data.empty()) {
    return;
  }
  size_t width = 0;
  size_t height = 0;
  terminalSize(width, height);
  std::vector<std::string> canvas;
  canvas.resize(height);
  for (size_t i=0; i<height; ++i) {
    canvas[i].resize(width, ' ');
  }
  double dataScale = (max - min) / height;
  double timeScale = (data.back().first / tick) / width;
  for (const auto& point : data) {
    size_t y = (point.second - min) / dataScale;
    size_t x = point.first / timeScale;
    canvas[y][x] = '*';
  }

  for (const auto& line : canvas) {
    printf("%s\n", line.data());
  }
}




}