#include "SimResult.h"
#include "Circuit.h"
#include <cmath>
#include <cassert>
#include <limits>
#include <algorithm>
#include <vector>

namespace NA {

Waveform::Waveform(const PWLValue& pwlValue)
{
  _points.reserve(pwlValue._value.size());
  for (size_t i=0; i<pwlValue._value.size(); ++i) {
    _points.push_back({pwlValue._time[i], pwlValue._value[i]});
  }
}

void
Waveform::range(double& max, double& min) const
{
  for (const WaveformPoint& p : _points) {
    max = std::max(max, p._value);
    min = std::min(min, p._value);
  }
}

double
Waveform::measure(double targetValue) const
{
  bool isRise = this->isRise();
  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  for (size_t i=1; i<_points.size(); ++i) {
    if ((isRise && _points[i-1]._value <= targetValue && _points[i]._value >= targetValue) || 
       (!isRise && _points[i-1]._value >= targetValue && _points[i]._value <= targetValue)) {
      x1 = _points[i-1]._time;
      y1 = _points[i-1]._value;
      x2 = _points[i]._time;
      y2 = _points[i]._value;
      break;
    }
  }
  if (x1 == 0 && x2 == 0) {
    return 1e99;
  }
  double k = (y2 - y1) / (x2 - x1);
  double b = y1 - k * x1;
  return (targetValue - b) / k;
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
branchDevices(const Circuit* ckt)
{
  std::vector<size_t> devIds;
  const std::vector<Device>& devs = ckt->devicesToSimulate();
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
SimResult::init(const Circuit* ckt)
{
  const std::vector<Node>& nodes = ckt->nodesToSimulate();
  _map._nodeVoltageMap.assign(ckt->nodeNumber(), SimResultMap::invalidValue());
  size_t index = 0;
  for (const Node& node : nodes) {
    if (node._isGround) {
      continue;
    }
    _map._nodeVoltageMap[node._nodeId] = index;
    ++index;
  }
  _map._deviceCurrentMap.assign(ckt->deviceNumber(), SimResultMap::invalidValue());
  const std::vector<size_t>& branchIds = branchDevices(ckt);
  for (size_t id : branchIds) {
    _map._deviceCurrentMap[id] = index;
    ++index;
  }
  _map.setDimention(index);
}

SimResult::SimResult(const Circuit* ckt, const std::string& name)
: _ckt(ckt), _name(name)
{
  init(ckt);
}

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
SimResult::nodeVoltageImp(size_t nodeId, size_t timeStep) const
{
  assert(_ticks.size() > timeStep);
  size_t nodeIndex = nodeVectorIndex(nodeId);
  assert(nodeIndex != SimResultMap::invalidValue() && "Incorrect nodeId");
  size_t resultIndex = timeStep * _map.size() + nodeIndex;
  return _values[resultIndex];
}

double
SimResult::nodeVoltage(size_t nodeId, size_t timeStep) const
{
  if (_ckt->isGroundNode(nodeId)) {
    return .0f;
  }
  double voltage = std::numeric_limits<double>::lowest();
  const Node& node = _ckt->node(nodeId);
  for (size_t devId : node._connection) {
    const Device& dev = _ckt->device(devId);
    if (dev._type == DeviceType::VoltageSource && dev._posNode == nodeId) {
      if (dev._isPWLValue) {
        double simTime = stepTime(timeStep);
        const PWLValue& pwlData = _ckt->PWLData(dev);
        voltage = pwlData.valueAtTime(simTime);
      } else {
        voltage = dev._value;
      }
      voltage = std::max(voltage, dev._value);
    }
  }
  if (voltage != std::numeric_limits<double>::lowest()) {
    return voltage;
  } 
  return nodeVoltageImp(nodeId, timeStep);
}

double 
SimResult::deviceCurrentImp(size_t deviceId, size_t timeStep) const
{
  assert(_ticks.size() > timeStep);
  size_t devIndex = deviceVectorIndex(deviceId);
  assert(devIndex != SimResultMap::invalidValue() && "Incorrect deviceId");
  size_t resultIndex = timeStep * _map.size() + devIndex;
  return _values[resultIndex];
}

double 
SimResult::deviceCurrent(size_t deviceId, size_t timeStep) const
{
  const Device& dev = _ckt->device(deviceId);
  if (dev._type == DeviceType::CurrentSource) {
    if (dev._isPWLValue) {
      double simTime = stepTime(timeStep);
      const PWLValue& pwlData = _ckt->PWLData(dev);
      return pwlData.valueAtTime(simTime);
    } else {
      return dev._value;
    }
  }
  return deviceCurrentImp(deviceId, timeStep);
}

double
SimResult::nodeVoltageBackstepImp(size_t nodeId, size_t steps) const
{
  assert(steps > 0 && "Incorrect input parameter");
  if (_ticks.size() < steps) {
    /// Initial condition. Normally this is computed with a DC OP analysis
    /// For now we use 0
    return 0;
  }
  size_t nodeIndex = nodeVectorIndex(nodeId);
  assert(nodeIndex != SimResultMap::invalidValue() && "Incorrect nodeId");
  steps -= 1;
  size_t resultIndex = (_ticks.size() - steps - 1) * _map.size() + nodeIndex;
  return _values[resultIndex];
}

double
SimResult::nodeVoltageBackstep(size_t nodeId, size_t steps) const
{
  if (_ckt->isGroundNode(nodeId)) {
    return .0f;
  }
  double voltage = std::numeric_limits<double>::lowest();
  const Node& node = _ckt->node(nodeId);
  for (size_t devId : node._connection) {
    const Device& dev = _ckt->device(devId);
    if (dev._type == DeviceType::VoltageSource && dev._posNode == nodeId) {
      voltage = std::max(voltage, dev._value);
    }
  }
  if (voltage != std::numeric_limits<double>::lowest()) {
    return voltage;
  } 
  return nodeVoltageBackstepImp(nodeId, steps);
}

double 
SimResult::deviceCurrentBackstepImp(size_t deviceId, size_t steps) const
{
  assert(steps > 0 && "Incorrect input parameter");
  if (_ticks.size() < steps) {
    /// Initial condition. Normally this is computed with a DC OP analysis
    /// For now we use 0
    return 0;
  }
  size_t devIndex = deviceVectorIndex(deviceId);
  assert(devIndex != SimResultMap::invalidValue() && "Incorrect deviceId");
  steps -= 1;
  size_t resultIndex = (_ticks.size() - steps - 1) * _map.size() + devIndex;
  return _values[resultIndex];
}

double 
SimResult::deviceCurrentBackstep(size_t deviceId, size_t steps) const
{
  const Device& dev = _ckt->device(deviceId);
  if (dev._type == DeviceType::CurrentSource) {
    return dev._value;
  }
  return deviceCurrentBackstepImp(deviceId, steps);
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

double
SimResult::stepTime(size_t step) const
{
  if (_ticks.size() <= step) {
    return std::numeric_limits<double>::max();
  }
  return _ticks[step];
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
SimResult::nodeVoltageDerivative(size_t nodeId, size_t order, size_t steps) const
{
  if (steps == 0) {
    return .0f;
  }
  size_t resultSize = _ticks.size();
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
    time.push_back(_ticks[i]);
  }
  return calcDerivative(voltage, time);
}

double 
SimResult::deviceVoltageDerivative(const Device& device, 
                                   size_t order, size_t steps) const
{
  if (steps == 0) {
    return .0f;
  }
  size_t resultSize = _ticks.size();
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
    time.push_back(_ticks[i]);
  }
  return calcDerivative(voltage, time);
}

double 
SimResult::deviceCurrentDerivative(const Device& device, size_t order, size_t steps) const
{
  if (steps == 0) {
    return .0f;
  }
  size_t resultSize = _ticks.size();
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
    time.push_back(_ticks[i]);
  }
  return calcDerivative(current, time);
}

Waveform
SimResult::waveformData(size_t rowIndex, double* max, double* min) const
{
  std::vector<WaveformPoint> data;
  if (max != nullptr) *max = std::numeric_limits<double>::lowest();
  if (min != nullptr) *min = std::numeric_limits<double>::max();
  size_t resultVectorSize = indexMap().size();
  for (size_t tIndex=0; tIndex<ticks().size(); ++tIndex) {
    size_t valueIndex = tIndex*resultVectorSize+rowIndex;
    double value = this->value(valueIndex);
    if (!std::isnan(value) && !std::isinf(value)) {
      if (max != nullptr) *max = std::max(*max, value);
      if (min != nullptr) *min = std::min(*min, value);
      data.push_back({tick(tIndex), value});
    }
  }
  return Waveform(data);
}

size_t
tryFindPWLVoltageDev(const Circuit* ckt, const Node& node) 
{
  for (size_t devId : node._connection) {
    const Device& dev = ckt->device(devId);
    if (dev._type == DeviceType::VoltageSource && dev._isPWLValue) {
      return devId;
    }
  }
  return SimResultMap::invalidValue();
}
    
Waveform 
SimResult::nodeVoltageWaveform(size_t nodeId) const
{
  size_t rowIndex = nodeVectorIndex(nodeId);
  if (rowIndex == SimResultMap::invalidValue()) {
    const Node& node = _ckt->node(nodeId);
    size_t devId = tryFindPWLVoltageDev(_ckt, node);
    if (devId != SimResultMap::invalidValue()) {
      return Waveform(_ckt->PWLData(_ckt->device(devId)));
    }
    return std::vector<WaveformPoint>();
  }
  return waveformData(rowIndex, nullptr, nullptr); 
}

Waveform
SimResult::deviceCurrentWaveform(size_t devId) const
{
  size_t rowIndex = deviceVectorIndex(devId);
  if (rowIndex == SimResultMap::invalidValue()) {
    const Device& device = _ckt->device(devId);
    if (device._isPWLValue) {
      return Waveform(_ckt->PWLData(device));
    }
    return std::vector<WaveformPoint>();
  }
  return waveformData(rowIndex, nullptr, nullptr); 
}

Waveform 
SimResult::nodeVoltageWaveform(const std::string& nodeName, 
                               double& max, double& min) const
{
  const Node& node = _ckt->findNodeByName(nodeName);
  if (node._nodeId == SimResultMap::invalidValue()) {
    printf("Node %s not found\n", nodeName.data());
    return std::vector<WaveformPoint>();
  }
  size_t rowIndex = nodeVectorIndex(node._nodeId);
  if (rowIndex == SimResultMap::invalidValue()) {
    size_t devId = tryFindPWLVoltageDev(_ckt, node);
    if (devId != SimResultMap::invalidValue()) {
      Waveform waveform(_ckt->PWLData(_ckt->device(devId)));
      waveform.range(max, min);
      return waveform;
    }
    return std::vector<WaveformPoint>();
  }
  return waveformData(rowIndex, &max, &min); 
}

Waveform 
SimResult::deviceCurrentWaveform(const std::string& devName, 
                                 double& max, double& min) const
{
  const Device& device = _ckt->findDeviceByName(devName);
  if (device._devId == static_cast<size_t>(-1)) {
    printf("Device %s not found\n", devName.data());
    return std::vector<WaveformPoint>();
  }
  size_t rowIndex = deviceVectorIndex(device._devId);
  if (rowIndex == SimResultMap::invalidValue()) {
    if (device._isPWLValue) {
      Waveform waveform(_ckt->PWLData(device));
      waveform.range(max, min);
      return waveform;
    }
    return std::vector<WaveformPoint>();
  }
  return waveformData(rowIndex, &max, &min); 
}

static inline double
chargeInTimeInterval(double I0, double I1, double timeInterval)
{
  return (I0 + I1) * timeInterval / 2;
}

static double
totalCharge(const std::vector<WaveformPoint>& waveform)
{
  double charge = 0;
  double prevT = 0;
  double prevI = 0;
  for (const WaveformPoint& p : waveform) {
    charge += chargeInTimeInterval(prevI, p._value, p._time - prevT);
    prevT = p._time;
    prevI = p._value;
  }
  return charge;
}

double
SimResult::totalCharge(const Device& device) const
{
  if (device._type != DeviceType::Resistor && device._type != DeviceType::VoltageSource) {
    printf("ERROR: total charge calculation is only supported on resistors and voltage sources\n");
    return 0;
  }
  if (device._type == DeviceType::Resistor) {
    const std::vector<WaveformPoint>& posWaveform = nodeVoltageWaveform(device._posNode).data();
    const std::vector<WaveformPoint>& negWaveform = nodeVoltageWaveform(device._negNode).data();
    std::vector<WaveformPoint> currentWaveform;
    currentWaveform.reserve(posWaveform.size());
    for (size_t i = 0; i < posWaveform.size(); ++i) {
      const WaveformPoint& posData = posWaveform[i];
      const WaveformPoint& negData = negWaveform[i];
      currentWaveform.push_back({posData._time, (posData._value - negData._value)/device._value});
      return NA::totalCharge(currentWaveform);
    }
  } else {
    const std::vector<WaveformPoint>& currentWaveform = deviceCurrentWaveform(device._devId).data();
    return NA::totalCharge(currentWaveform);
  }
  return 0;
}


}
