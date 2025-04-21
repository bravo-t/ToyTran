#include "Measure.h"
#include "Circuit.h"

namespace NA {

static inline double 
calcMeasureTime(double x1, double y1, double x2, double y2, double value)
{
  double k = (y2 - y1) / (x2 - x1);
  double b = y1 - k * x1;
  return (value - b) / k;
}

static inline bool
getSimData(const SimResultType& type, const std::string& point, 
           const SimResult& result, double step, 
           double& value, double& nextValue)
{
  const Circuit& ckt = result.circuit();
  if (type == SimResultType::Voltage) {
    const Node& node = ckt.findNodeByName(point);
    if (node._nodeId == static_cast<size_t>(-1)) {
      printf("Measure error: Node %s not found\n", point.data());
      return false;
    }
    value = result.nodeVoltage(node._nodeId, step);
    nextValue = result.nodeVoltage(node._nodeId, step+1);
  } else if (type == SimResultType::Current) {
    const Device& dev = ckt.findDeviceByName(point);
    if (dev._devId == static_cast<size_t>(-1)) {
      printf("Measure error: device %s not found\n", point.data());
      return false;
    }
    value = result.deviceCurrent(dev._devId, step);
    nextValue = result.deviceCurrent(dev._devId, step+1);
  }
  return true;
}

double
measure(const SimResult& simData, const MeasurePoint& mp)
{
  size_t simSteps = simData.size();
  bool startMeasure = false;
  bool triggerTimeFound = false;
  bool targetTimeFound = false;
  double measureStart = .0f;
  double measureEnd = .0f;
  double measureStep = 0;
  for (size_t step=0; step<simSteps-1; ++step) {
    if (startMeasure == false &&
        simData.stepTime(step) >= mp._timeDelay) {
      startMeasure = true;
    } 
    if (startMeasure) {
      if (triggerTimeFound == false) {
        double triggerValue = 0;
        double triggerValueNext = 0;
        if (getSimData(mp._triggerType, mp._trigger, simData, step, triggerValue, triggerValueNext) == false) {
          return 0;
        }
        if ((triggerValue <= mp._triggerValue && triggerValueNext >= mp._triggerValue) || 
            (triggerValue >= mp._triggerValue && triggerValueNext <= mp._triggerValue)) {
          double currentTime = simData.stepTime(step);
          double nextTime = simData.stepTime(step+1);
          measureStart = calcMeasureTime(currentTime, triggerValue, nextTime, triggerValueNext, mp._triggerValue);
          triggerTimeFound = true;
        } else if (measureStep == 0 && triggerValue > mp._triggerValue) {
          measureStart = simData.stepTime(step);
          triggerTimeFound = true;
        }
      }
      if (targetTimeFound == false) {
        double targetValue = 0;
        double targetValueNext = 0;
        if (getSimData(mp._targetType, mp._target, simData, step, targetValue, targetValueNext) == false) {
          return 0;
        }
        if ((targetValue <= mp._targetValue && targetValueNext >= mp._targetValue) || 
            (targetValue >= mp._targetValue && targetValueNext <= mp._targetValue)) {
          double currentTime = simData.stepTime(step);
          double nextTime = simData.stepTime(step+1);
          measureEnd = calcMeasureTime(currentTime, targetValue, nextTime, targetValueNext, mp._targetValue);
          targetTimeFound = true;
        }
      }
      if (triggerTimeFound && targetTimeFound) {
        break;
      }
    }
    ++measureStep;
  }
  if (triggerTimeFound == false) {
    printf("Measure error: %s trigger condition never meet the required value\n", mp._variableName.data());
    return 0;
  }
  if (targetTimeFound == false) {
    printf("Measure error: %s target value never meet\n", mp._variableName.data());
    return 0;
  }
  return measureEnd - measureStart;
}

void
Measure::run() const
{
  for (const MeasurePoint& mp : _measurePoints) {
    double result = measure(_simResult, mp);
    printf("Measurement %s: %E second(s)\n", mp._variableName.data(), result);
  }
}

}