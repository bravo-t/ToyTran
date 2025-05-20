#include "EffCap.h"
#include "SimResult.h"

namespace NA {

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
    charge += chargeInTimeInterval(prevI, p.value, p.time - prevT);
    prevT = p.time;
    prevI = p.value;
  }
  return charge;
}

double
EffCap::charge(const Circuit& ckt, const SimResult& result, const Device& device)
{
  if (device._type != DeviceType::Resistor && device._type != DeviceType::VoltageSource) {
    printf("ERROR: total charge calculation is only supported on resistors and voltage sources\n");
    return 0;
  }
  if (device._type == DeviceType::Resistor) {
    const std::vector<WaveformPoint>& posWaveform = result.nodeVoltageWaveform(device._posNode);
    const std::vector<WaveformPoint>& negWaveform = result.nodeVoltageWaveform(device._negNode);
    std::vector<WaveformPoint>& currentWaveform;
    currentWaveform.reserve(posWaveform.size());
    for (size_t i = 0; i < posWaveform.size(); ++i) {
      const WaveformPoint& posData = posWaveform[i];
      const WaveformPoint& negData = negWaveform[i];
      currentWaveform.push_back({posData.time, (posData.value - negData.value)/device._value});
      return totalCharge(currentWaveform);
    }
  } else {
    const std::vector<WaveformPoint>& currentWaveform = result.deviceCurrentWaveform(device._devId);
    return totalCharge(currentWaveform);
  }
  return 0;
}



}
