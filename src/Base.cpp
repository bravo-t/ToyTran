#include "Base.h"
#include "LibData.h"

namespace NA {


double 
PWLValue::valueAtTime(double time) const
{
  if (time < _time[0]) {
    return 0;
  }
  for (size_t i=1; i<_time.size(); ++i) {
    if (time < _time[i]) {
      double v1 = _value[i-1];
      double v2 = _value[i];
      double t1 = _time[i-1];
      double t2 = _time[i];
      //printf("DEBUG: time %g goes into [%g, %g] interval, voltage: [%g, %g], interpolated voltage: %g\n", 
      //  time, t1, t2, v1, v2, v1 + (v2-v1)/(t2-t1)*(time-t1));
      return v1 + (v2-v1)/(t2-t1)*(time-t1);
    }
  }
  return _value.back();
}

double 
PWLValue::measure(double targetValue) const
{
  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  for (size_t i=1; i<_value.size(); ++i) {
    if ((_value[i-1] <= targetValue && _value[i] >= targetValue) ||
        (_value[i-1] >= targetValue && _value[i] <= targetValue)) {
      x1 = _time[i-1];
      y1 = _value[i-1];
      x2 = _time[i];
      y2 = _value[i];
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

Waveform::Waveform(const PWLValue& pwlValue)
{
  _points.reserve(pwlValue._value.size());
  for (size_t i=0; i<pwlValue._value.size(); ++i) {
    _points.push_back({pwlValue._time[i], pwlValue._value[i]});
  }
}

Waveform::Waveform(bool isRise, double startTime, 
                   double rampTime, double voltage) 
{
  double initVoltage = 0;
  double endVoltage = voltage;
  if (isRise == false) {
    initVoltage = voltage;
    endVoltage = 0;
  }
  _points.push_back({0, initVoltage});
  if (startTime > 0) {
    _points.push_back({startTime, initVoltage});
  }
  _points.push_back({startTime+rampTime, endVoltage});
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

void 
Waveform::range(double& max, double& min) const
{
  for (const WaveformPoint& p : _points) {
    max = std::max(max, p._value);
    min = std::min(min, p._value);
  }
}

size_t 
Waveform::indexTime(double time) const 
{
  if (_points.size() <= 2) {
    return 0;
  }
  size_t lower = 1;
  size_t upper = _points.size()-2;
  if (time <= _points[lower]._time) {
    return 0;
  }
  if (time >= _points[upper]._time) {
    return upper;
  }
  size_t idx = 0;
  while (upper - lower > 1) {
    idx = (lower + upper) >> 1;
    if (_points[idx]._time > time) {
      upper = idx;
    } else {
      lower = idx;
    }
  }
  if (_points[idx]._time > time) {
    --idx;
  }
  return idx;
}

double 
Waveform::value(double time) const 
{
  if (_points.size() < 2) {
    return _points[0]._value;
  }
  size_t idx1 = indexTime(time);
  size_t idx2 = idx1 + 1;
  double t1 = _points[idx1]._time;
  double v1 = _points[idx1]._value;
  double t2 = _points[idx2]._time;
  double v2 = _points[idx2]._value;
  
  double k = (v2 - v1) / (t2 - t1);
  double b = v1 - k * t1;
  return k * time + b;
}

double 
Waveform::valueNoExtrapolation(double time) const 
{
  if (time <= _points[0]._time) {
    return _points[0]._value;
  }
  if (time >= _points.back()._time) {
    return _points.back()._value;
  }
  size_t idx1 = indexTime(time);
  size_t idx2 = idx1 + 1;
  double t1 = _points[idx1]._time;
  double v1 = _points[idx1]._value;
  double t2 = _points[idx2]._time;
  double v2 = _points[idx2]._value;
  
  double k = (v2 - v1) / (t2 - t1);
  double b = v1 - k * t1;
  return k * time + b;
}

double 
Waveform::valueAtBackStep(size_t backStep) const {
  if (backStep >= _points.size()) {
    return 0;
  }
  return _points[_points.size()-1-backStep]._value;
}

double 
Waveform::timeAtBackStep(size_t backStep) const {
  if (backStep >= _points.size()) {
    return 0;
  }
  return _points[_points.size()-1-backStep]._time;
}

double 
Waveform::transitionTime(const LibData* libData) const 
{
  double vol = libData->voltage();
  double v1, v2;
  if (isRise()) {
    v1 = libData->riseTransitionLowThres() / 100 * vol;
    v2 = libData->riseTransitionHighThres() / 100 * vol;
  } else {
    v1 = libData->fallTransitionHighThres() / 100 * vol;
    v2 = libData->fallTransitionLowThres() / 100 * vol;
  }
  double t1 = measure(v1);
  double t2 = measure(v2);
  if (t1 == 1e99 || t2 == 1e99) {
    return 1e99;
  } 
  return t2 - t1;
}

}