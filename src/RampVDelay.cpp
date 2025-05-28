#include "RampVDelay.h"
#include "Circuit.h"

namespace NA {

double
totalLoadOnDriver(Circuit* ckt, size_t rdId)
{
  const std::vector<Device*> connDevs = ckt->traceDevice(rdId);
  double totalCap = 0;
  for (const Device* dev : connDevs) {
    if (dev->_type == DeviceType::Capacitor) {
      totalCap += dev->_value;
    }
  }
  return totalCap;
}

/// tranThres1 and tranThres2 are two threshold that defines transition time,
/// the output voltage will go through tranThres1 first, tranThres2 second.
/// for rise transition they correspond to low and high thresholds, 
/// for fall transition they correspond to high and low thresholds.
/// For fall transitions, targetThres should be (100-expected value), 
/// for example, if you want a t80 for fall transitions, set the targetThres to (100-80)=20
static inline double 
extrapolateDelayTime(double tDelay, double delayThres, 
                     double trans, double targetThres, 
                     double tranThres1, double tranThres2)
{
  double trans100 = trans / (100/tranThres2 - 100/tranThres1);
  double zeroTime = tDelay - trans100/(100/delayThres);
  return zeroTime + targetThres*trans100/100;
}

void 
calcNLDMLUTDelayTrantion(const NLDMArc* nldmData, double inputTran, double outputLoad, bool isRise)
{
  
}

bool
RampVDelay::calculate() 
{
  double inputTran = _cellArc->inputTransition(_ckt);
  
}


}