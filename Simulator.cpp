#include <algorithm>
#include "Simulator.h"
#include "Circuit.h"
#include "MNAStamper.h"
#include "Debug.h"

namespace Tran {

IntegrateMethod
Simulator::integrateMethod() const
{
  if (_intMethod == IntegrateMethod::BackwardEuler) {
    return IntegrateMethod::BackwardEuler;
  } else if (_intMethod == IntegrateMethod::Gear2) {
    if (_result._ticks.size() < 1) {
      return IntegrateMethod::BackwardEuler;
    } else {
      return IntegrateMethod::Gear2;
    }
/*} else if (intMethod == IntegrateMethod::RK4) {
    if (prevData._ticks.size() < 1) {
      return IntegrateMethod::BackwardEuler;
    } else if (prevData._ticks.size() < 3) {
      return IntegrateMethod::Gear2;
    } else {
      return IntegrateMethod::RK4;
    } */
  }
  return IntegrateMethod::Gear2;
}

static inline bool
needExtraDim(const Device& dev) 
{
  if (dev._type == DeviceType::VoltageSource ||
      dev._type == DeviceType::VCVS ||
      dev._type == DeviceType::CCVS || 
      dev._type == DeviceType::Inductor) {
    return true;
  } 
  return false;
}

size_t 
SimResult::deviceVectorIndex(size_t deviceId) const 
{
  const std::unordered_map<size_t, size_t>& map = _map._deviceCurrentMap;
  const auto& found = map.find(deviceId);
  if (found != map.end()) {
    return found->second;
  }
  return static_cast<size_t>(-1);
}

size_t 
branchDeviceMatrixIndex(const Device& dev, const Simulator* sim)
{
  if (!needExtraDim(dev)) {
    return static_cast<size_t>(-1);
  } 
  return sim->simulationResult().deviceVectorIndex(dev._devId);
}

size_t
SimResult::nodeVectorIndex(size_t nodeId) const
{
  return nodeId;
  const std::unordered_map<size_t, size_t>& map = _map._nodeVoltageMap;
  const auto& found = map.find(nodeId);
  if (found != map.end()) {
    return found->second;
  }
  return static_cast<size_t>(-1);
}

double 
SimResult::currentTime() const
{
  if (_ticks.size() == 0) {
    return 0;
  } 
  return _ticks.back();
}

size_t 
nodeMatrixIndex(const Node& node, const Simulator* sim)
{
  return sim->simulationResult().nodeVectorIndex(node._nodeId);
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
  size_t resultIndex = (_ticks.size() - steps) * _map.size() + nodeIndex;
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
  size_t resultIndex = (_ticks.size() - steps) * _map.size() + devIndex;
  return _values[resultIndex];
}

static inline IntegrateMethod
chooseIntMethod(IntegrateMethod intMethod, const SimResult& prevData)
{
  if (intMethod == IntegrateMethod::BackwardEuler) {
    return IntegrateMethod::BackwardEuler;
  } else if (intMethod == IntegrateMethod::Gear2) {
    if (prevData._ticks.size() < 1) {
      return IntegrateMethod::BackwardEuler;
    } else {
      return IntegrateMethod::Gear2;
    }
/*} else if (intMethod == IntegrateMethod::RK4) {
    if (prevData._ticks.size() < 1) {
      return IntegrateMethod::BackwardEuler;
    } else if (prevData._ticks.size() < 3) {
      return IntegrateMethod::Gear2;
    } else {
      return IntegrateMethod::RK4;
    } */
  }
  return IntegrateMethod::Gear2;
}

std::vector<size_t>
branchDevices(const Circuit& ckt)
{
  std::vector<size_t> devIds;
  const std::vector<Device>& devs = ckt.devices();
  for (const Device& dev : devs) {
    if (needExtraDim(dev)) {
      devIds.push_back(dev._devId);
    }
    if (dev._type == DeviceType::CCCS ||
        dev._type == DeviceType::CCVS) {
      if (dev._sampleDevice != static_cast<size_t>(-1)) {
        devIds.push_back(dev._sampleDevice);
      }
    }
  }
  /// Sort and unique as dev._sampleDevice can introduce duplicates
  std::sort(devIds.begin(), devIds.end());
  devIds.erase(std::unique(devIds.begin(), devIds.end()), devIds.end());
  return devIds;
}

void
initResultMap(const Circuit& ckt, SimResult& result)
{
  const std::vector<Node>& nodes = ckt.nodes();
  size_t index = 0;
  for (const Node& node : nodes) {
    result._map._nodeVoltageMap.insert({node._nodeId, index});
    ++index;
  }
  const std::vector<size_t>& branchIds = branchDevices(ckt);
  for (size_t id : branchIds) {
    result._map._deviceCurrentMap.insert({id, index});
    ++index;
  }
}

void 
Simulator::updateEquation()
{
  if (_needUpdateA) {
    MNAStamper::updateA(_Alu, this);
  }
  MNAStamper::updateb(_b, this);
}

void 
Simulator::formulateEquation()
{
  initResultMap(_circuit, _result);
  _eqnDim = _result._map.size();
  Eigen::MatrixXd A;
  A.setZero(_eqnDim, _eqnDim);
  _b.setZero(_eqnDim);
  MNAStamper::stamp(A, _b, this);
  if (Debug::enabled()) {
    Debug::printEquation(A, _b);
  }
  _Alu = A.fullPivLu();
}

void 
Simulator::solveEquation()
{
  Eigen::VectorXd x(_eqnDim);
  x = _Alu.solve(_b);

  std::vector<double>& ticks = _result._ticks;
  std::deque<double>& values = _result._values;
  double prevTime = 0;
  if (ticks.empty() == false) {
    prevTime = ticks.back();
  }
  ticks.push_back(prevTime + _simTick);
  values.insert(values.end(), x.begin(), x.end());
  if (Debug::enabled()) {
    Debug::printIterationSolution(prevTime+_simTick, x);
  }
}

bool
Simulator::converged() const
{
  double simTime = 0;
  if (_result._ticks.empty() == false) {
    simTime = _result._ticks.back();
  }
  bool converge = true;
  return simTime >= _simEnd && converge;
}

void
Simulator::adjustSimTick()
{
  /// Implement LTE calculation and adjust _simTick based on it
  /// and set _needUpdateA to true
  return;
}

void 
Simulator::run()
{
  formulateEquation();
  solveEquation();
  while (!converged()) {
    adjustSimTick();
    updateEquation();
    solveEquation();
  }
}




}
