#include "EffCap.h"

namespace NA {

static inline double
chargeInTimeInterval(double I0, double I1, double timeInterval)
{
  return (I0 + I1) * timeInterval / 2;
}

static double
totalCharge(const Circuit& ckt, const SimResult& result, const Device& device)
{

}

double
EffCap::charge(const Circuit& ckt, const SimResult& result, const Device& device)
{
  if (device._type != DeviceType::Resistor && device._type != DeviceType::VoltageSource) {
    printf("ERROR: total charge calculation is only supported on resistors and voltage sources\n");
    return 0;
  }

}

}
