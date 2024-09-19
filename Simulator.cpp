#include <algorithm>
#include <iostream>
#include "Simulator.h"
#include "Circuit.h"
#include "MNAStamper.h"
#include "StepControl.h"
#include "Debug.h"

namespace Tran {

IntegrateMethod
Simulator::integrateMethod() const
{
  IntegrateMethod method = IntegrateMethod::None;
  if (_intMethod == IntegrateMethod::BackwardEuler) {
    method = IntegrateMethod::BackwardEuler;
  } else if (_intMethod == IntegrateMethod::Gear2) {
    if (_result._ticks.size() < 2) {
      method = IntegrateMethod::BackwardEuler;
    } else {
      method= IntegrateMethod::Gear2;
    }
  } else if (_intMethod == IntegrateMethod::Trapezoidal) {
    if (_result._ticks.size() < 2) {
      method = IntegrateMethod::BackwardEuler;
    } else {
      method= IntegrateMethod::Trapezoidal;
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
  if (method == IntegrateMethod::None) {
    method = IntegrateMethod::Gear2;
  }
  return method;
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
  result._map._nodeVoltageMap.assign(nodes.size(), SimResultMap::invalidValue());
  size_t index = 0;
  for (const Node& node : nodes) {
    if (node._isGround) {
      continue;
    }
    result._map._nodeVoltageMap[node._nodeId] = index;
    ++index;
  }
  result._map._deviceCurrentMap.assign(ckt.deviceNumber(), SimResultMap::invalidValue());
  const std::vector<size_t>& branchIds = branchDevices(ckt);
  for (size_t id : branchIds) {
    result._map._deviceCurrentMap[id] = index;
    ++index;
  }
  result._map.setDimention(index);
}

void 
Simulator::updateEquation()
{
  //if (_needUpdateA) {
  //  MNAStamper::updateA(_Alu, this);
  //}
  if (_needRebuild) {
    formulateEquation();
  } else {
    MNAStamper::updateb(_b, this);
    if (Debug::enabled()) {
      double prevTime = _result._ticks.back();
      Debug::printVector(prevTime+_simTick, "b", _b);
    }
  }
}

void 
Simulator::formulateEquation()
{
  Eigen::MatrixXd A;
  A.setZero(_eqnDim, _eqnDim);
  _b.setZero(_eqnDim);
  MNAStamper::stamp(A, _b, this);
  
  if (Debug::enabled()) {
    Debug::printEquation(A, _b);
    /*Eigen::EigenSolver<Eigen::MatrixXd> es(A);
    printf("Eigenvalues of A: \n");
    for (int i=0; i<A.rows(); ++i) {
      std::complex<double> value = es.eigenvalues().col(0)[i];
      printf("  %f+%fi\n", std::real(value), std::imag(value));
    }
    */
  }
  _Alu = A.fullPivLu();
}

void 
Simulator::initData()
{
  initResultMap(_circuit, _result);
  _eqnDim = _result._map.size();
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
    Debug::printSolution(prevTime+_simTick, "x", x, this);
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
  return simTime > _simEnd && converge;
}

void
Simulator::adjustSimTick()
{
  double stepSizeLimit = StepControl::stepLimit(this, _relTol);
  if (_simTick > stepSizeLimit) {
    printf("WARNING: Current step size %G is larger the LTE limit %G\n", _simTick, stepSizeLimit);
  }
  /// Implement LTE calculation and adjust _simTick based on it
  /// and set _needUpdateA to true
  return;
}

void 
Simulator::run()
{
  initData();
  formulateEquation();
  solveEquation();
  while (!converged()) {
    checkNeedRebuild();
    adjustSimTick();
    updateEquation();
    solveEquation();
  }
}

void
Simulator::setIntegrateMethod(IntegrateMethod intMethod)
{
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      printf("Backward Euler is chosen as the numerical integration method\n");
      _intMethod = intMethod;
      break;
    case IntegrateMethod::Gear2:
      printf("Gear2 (BDF2) is chosen as the numerical integration method\n");
      _intMethod = intMethod;
      break;
    case IntegrateMethod::Trapezoidal:
      printf("Trapezoidal method is chosen as the numerical integration method\n");
      _intMethod = intMethod;
      break;
    default:
      printf("Unsupported numerical integration method, using default Gear2\n");
      _intMethod = IntegrateMethod::Gear2;
  }
}

void
Simulator::setSimTick(double simTick) 
{
  printf("Simulation time step set to %g second\n", simTick);
  _simTick = simTick;
}

void
Simulator::setSimulationEndTime(double simTime)
{
  printf("Simulation end time set to %g second\n", simTime);
  _simEnd = simTime;
}

double
Simulator::nodeVoltage(size_t nodeId, size_t timeStep) const
{
  if (circuit().isGroundNode(nodeId)) {
    return .0f;
  }
  double voltage = std::numeric_limits<double>::lowest();
  const Node& node = _circuit.node(nodeId);
  for (size_t devId : node._connection) {
    const Device& dev = _circuit.device(devId);
    if (dev._type == DeviceType::VoltageSource && dev._posNode == nodeId) {
      voltage = std::max(voltage, dev._value);
    }
  }
  if (voltage != std::numeric_limits<double>::lowest()) {
    return voltage;
  } 
  return _result.nodeVoltage(nodeId, timeStep);
}

double 
Simulator::deviceCurrent(size_t deviceId, size_t timeStep) const
{
  const Device& dev = _circuit.device(deviceId);
  if (dev._type == DeviceType::CurrentSource) {
    return dev._value;
  }
  return _result.deviceCurrent(deviceId, timeStep);
}

double
Simulator::nodeVoltageBackstep(size_t nodeId, size_t steps) const
{
  if (circuit().isGroundNode(nodeId)) {
    return .0f;
  }
  double voltage = std::numeric_limits<double>::lowest();
  const Node& node = _circuit.node(nodeId);
  for (size_t devId : node._connection) {
    const Device& dev = _circuit.device(devId);
    if (dev._type == DeviceType::VoltageSource && dev._posNode == nodeId) {
      voltage = std::max(voltage, dev._value);
    }
  }
  if (voltage != std::numeric_limits<double>::lowest()) {
    return voltage;
  } 
  return _result.nodeVoltageBackstep(nodeId, steps);
}

double 
Simulator::deviceCurrentBackstep(size_t deviceId, size_t steps) const
{
  const Device& dev = _circuit.device(deviceId);
  if (dev._type == DeviceType::CurrentSource) {
    return dev._value;
  }
  return _result.deviceCurrentBackstep(deviceId, steps);
}

double 
calcDerivative(const std::vector<double>& y, 
               const std::vector<double>& x)
{
  std::vector<double> derivative(y.begin(), y.end());
  size_t xIndexOffset = 0;
  while (derivative.size() > 1) {
    for (size_t i=1; i<derivative.size(); ++i) {
      double deltaY = derivative[i] - derivative[i-1];
      size_t xIndex = xIndexOffset + i;
      double deltaX = x[xIndex] - x[xIndex-1];
      double derivativeValue = deltaY / deltaX;
      derivative[i-1] = derivativeValue;
    }
    derivative.pop_back();
    xIndexOffset += 1;
  }
  return derivative.back();
}

double 
Simulator::nodeVoltageDerivative(size_t nodeId, size_t order, size_t steps) const
{
  if (steps == 0) {
    return .0f;
  }
  size_t resultSize = _result._ticks.size();
  if (resultSize <= steps+order) {
    return .0f;
  }

  std::vector<double> voltage;
  std::vector<double> time;
  /// Iterate backward to make sure voltage and time are in correct order
  for (size_t i=steps+order; i>=steps; --i) {
    double vol = nodeVoltageBackstep(nodeId, i);
    voltage.push_back(vol);
  }
  size_t timeEndIndex = resultSize - steps;
  size_t timeStartIndex = timeEndIndex - order;
  for (size_t i=timeStartIndex; i<=timeEndIndex; ++i) {
    time.push_back(_result._ticks[i]);
  }
  return calcDerivative(voltage, time);
}

double 
Simulator::deviceVoltageDerivative(const Device& device, 
                                   size_t order, size_t steps) const
{
  if (steps == 0) {
    return .0f;
  }
  size_t resultSize = _result._ticks.size();
  if (resultSize <= steps+order) {
    return .0f;
  }

  size_t posNodeId = device._posNode;
  size_t negNodeId = device._negNode;

  std::vector<double> voltage;
  std::vector<double> time;
  /// Iterate backward to make sure voltage and time are in correct order
  for (size_t i=steps+order; i>=steps; --i) {
    double volDiff = nodeVoltageBackstep(posNodeId, i) - nodeVoltageBackstep(negNodeId, i);
    voltage.push_back(volDiff);
  }
  size_t timeEndIndex = resultSize - steps;
  size_t timeStartIndex = timeEndIndex - order;
  for (size_t i=timeStartIndex; i<=timeEndIndex; ++i) {
    time.push_back(_result._ticks[i]);
  }
  return calcDerivative(voltage, time);
}

double 
Simulator::deviceCurrentDerivative(const Device& device, size_t order, size_t steps) const
{
  if (steps == 0) {
    return .0f;
  }
  size_t resultSize = _result._ticks.size();
  if (resultSize <= steps+order) {
    return .0f;
  }

  std::vector<double> current;
  std::vector<double> time;
  /// Iterate backward to make sure voltage and time are in correct order
  for (size_t i=steps+order; i>=steps; --i) {
    current.push_back(deviceCurrentBackstep(device._devId, i));
  }
  size_t timeEndIndex = resultSize - steps;
  size_t timeStartIndex = timeEndIndex - order;
  for (size_t i=timeStartIndex; i<=timeEndIndex; ++i) {
    time.push_back(_result._ticks[i]);
  }
  return calcDerivative(current, time);
}

void
Simulator::checkNeedRebuild() 
{
  _needRebuild = false;
  if (_prevMethod != integrateMethod()) {
    _prevMethod = integrateMethod();
    _needRebuild = true;
  }
}

}
