#include <cstdio>
#include <vector>
#include <cmath>
#include "RampVDelay.h"

namespace NA {

static size_t invalidId = static_cast<size_t>(-1);
static double delayMatchPoint = 20;
static double rdMatchPoint = 90;

double
totalLoadOnDriver(const Circuit* ckt, size_t rdId)
{
  const std::vector<const Device*> connDevs = ckt->traceDevice(rdId);
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
calcNLDMLUTDelayTrantion(const NLDMArc* nldmData, double inputTran, 
                         double outputLoad, bool isRise, 
                         double& delay, double& trans)
{
  LUTType delayLUTType = LUTType::RiseDelay;
  LUTType transLUTType = LUTType::RiseTransition;
  if (isRise == false) {
    delayLUTType = LUTType::FallDelay;
    transLUTType = LUTType::FallTransition;
  }
  const NLDMLUT& delayLUT = nldmData->getLUT(delayLUTType);
  const NLDMLUT& transLUT = nldmData->getLUT(transLUTType);
  delay = delayLUT.value(inputTran, outputLoad);
  trans = transLUT.value(inputTran, outputLoad);
}

void
RampVDelay::initParameters()
{
  size_t vSrcId = _cellArc->inputSourceDevId(_ckt);
  if (vSrcId == invalidId) {
    printf("Cannot find input source device on driver model\n");
    return;
  }
  const Device& vSrc = _ckt->device(vSrcId);
  const PWLValue& data = _ckt->PWLData(vSrc);

  _isRiseOnDriverPin = (data.isRiseTransition() != _cellArc->isInvertedArc());
  _effCap =  totalLoadOnDriver(_ckt, _cellArc->driverResistorId());
  double inputTran = _cellArc->inputTransition(_ckt);
  double t50 = 0;
  double trans = 0;
  calcNLDMLUTDelayTrantion(_cellArc->nldmData(), inputTran, _effCap, 
                           _isRiseOnDriverPin, t50, trans);
  _delayThres = _libData->riseDelayThres();
  _tranThres1 = _libData->riseTransitionLowThres();
  _tranThres2 = _libData->riseTransitionHighThres();
  if (_isRiseOnDriverPin == false) {
    _delayThres = _libData->fallDelayThres();
    _tranThres1 = _libData->fallTransitionHighThres();
    _tranThres2 = _libData->fallTransitionLowThres();
  }
  double t20 = extrapolateDelayTime(t50, trans, delayMatchPoint);
  double t90 = extrapolateDelayTime(t50, trans, rdMatchPoint);
  _tDelta = (t50-t20)*10/3;
  _rd = (t90 - t50) / (_effCap * log(5));
  _tZero = t50 - 0.69*_rd*_effCap - _tDelta/2;
}

double 
RampVDelay::extrapolateDelayTime(double t50, double trans, double targetThres) const
{
  return ::NA::extrapolateDelayTime(t50, _delayThres, trans, targetThres, _tranThres1, _tranThres2);
}

bool
RampVDelay::calculate() 
{
  
  return true;
}


}