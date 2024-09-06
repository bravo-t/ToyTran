#include "SimResult.h"
#include <cassert>

namespace Tran {

size_t 
SimResult::deviceVectorIndex(size_t deviceId) const 
{
  assert(deviceId < _map._deviceCurrentMap.size());
  return _map._deviceCurrentMap[deviceId];
}

size_t
SimResult::nodeVectorIndex(size_t nodeId) const
{
  assert(nodeId < _map._nodeVoltageMap.size());
  return _map._nodeVoltageMap[nodeId];
}

double 
SimResult::currentTime() const
{
  if (_ticks.size() == 0) {
    return 0;
  } 
  return _ticks.back();
}

double
SimResult::nodeVoltage(size_t nodeId, size_t steps) const
{
  assert(steps > 0 && "Incorrect input parameter");
  if (_ticks.size() < steps) {
    /// Initial condition. Normally this is computed with a DC OP analysis
    /// For now we use 0
    return 0;
  }
  size_t nodeIndex = nodeVectorIndex(nodeId);
  assert(nodeIndex != static_cast<size_t>(-1) && "Incorrect nodeId");
  steps -= 1;
  size_t resultIndex = (_ticks.size() - steps - 1) * _map.size() + nodeIndex;
  return _values[resultIndex];
}

double 
SimResult::deviceCurrent(size_t deviceId, size_t steps) const
{
  assert(steps > 0 && "Incorrect input parameter");
  if (_ticks.size() < steps) {
    /// Initial condition. Normally this is computed with a DC OP analysis
    /// For now we use 0
    return 0;
  }
  size_t devIndex = deviceVectorIndex(deviceId);
  assert(devIndex != static_cast<size_t>(-1) && "Incorrect deviceId");
  steps -= 1;
  size_t resultIndex = (_ticks.size() - steps - 1) * _map.size() + devIndex;
  return _values[resultIndex];
}

double 
SimResult::stepSize(size_t steps) const
{
  if (_ticks.size() < steps + 2) {
    return .0f;
  }
  size_t index = _ticks.size() - 1 - steps;
  return _ticks[index] - _ticks[index-1];
}





}