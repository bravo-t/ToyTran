#include <cstdio>
#include <vector>
#include <cmath>
#include "RampVCellDelay.h"
#include "RootSolver.h"
#include "Simulator.h"
#include "Debug.h"

namespace NA {

static size_t invalidId = static_cast<size_t>(-1);
static double delayMatchPoint = 20;
static double rdMatchPoint = 90;

double
totalLoadOnDriver(const Circuit* ckt, size_t rdId)
{
  const std::vector<const Device*>& connDevs = ckt->traceDevice(rdId);
  double totalCap = 0;
  for (const Device* dev : connDevs) {
    if (dev->_type == DeviceType::Capacitor) {
      totalCap += dev->_value;
    }
  }
  return totalCap;
}

void
markSimulationScope(size_t rdId, Circuit* ckt)
{
  const std::vector<const Device*>& connDevs = ckt->traceDevice(rdId);
  ckt->markSimulationScope(connDevs);
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
  double trans100 = trans / (tranThres2-tranThres1) * 100;
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
  //printf("DEBUG: rise : %s, inTran = %G, outLoad = %G, delay = %G, trans = %G\n", isRise ? "T" : "F", inputTran, outputLoad, delay, trans);
}

void 
RampVCellDelay::updateTParams() 
{
  calcNLDMLUTDelayTrantion(_cellArc->nldmData(), _inputTran, _effCap, 
                           _isRiseOnDriverPin, _t50, _driverPinTran);
  _t20 = extrapolateDelayTime(_t50, _driverPinTran, delayMatchPoint);
}

void 
RampVCellDelay::updateRd()
{
  double t90 = extrapolateDelayTime(_t50, _driverPinTran, rdMatchPoint);
  _rd = (t90 - _t50) / (_effCap * log(5));
}

void
RampVCellDelay::updateLoadCaps()
{
  size_t rdId = _cellArc->driverResistorId();
  const std::vector<const Device*>& connDevs = _ckt->traceDevice(rdId);
  for (const Device* dev : connDevs) {
    if (dev->_isInternal && dev->_type == DeviceType::Capacitor) {
      const std::vector<CellArc*>& arcs = _ckt->cellArcsOfDevice(dev);
      assert(arcs.empty() == false);
      const CellArc* loadArc = arcs[0];
      Device& mDev = _ckt->device(dev->_devId);
      /// This is the load device, so they should follow the same transition direction of driver pin
      mDev._value = loadArc->fixedLoadCap(_isRiseOnDriverPin);
      if (Debug::enabled(DebugModule::NLDM)) {
        printf("DEBUG: Load cap %s value updated to %G\n", dev->_name.data(), mDev._value);
      }
    }
  }
}

void
RampVCellDelay::initParameters()
{
  size_t vSrcId = _cellArc->inputSourceDevId(_ckt);
  if (vSrcId == invalidId) {
    printf("Cannot find input source device on driver model\n");
    return;
  }
  const Device& vSrc = _ckt->device(vSrcId);
  const PWLValue& data = _ckt->PWLData(vSrc);

  _isRiseOnInputPin = data.isRiseTransition();

  _isRiseOnDriverPin = (_isRiseOnInputPin != _cellArc->isInvertedArc());
  if (_isRiseOnDriverPin) {
    _delayThres = _libData->riseDelayThres();
    _tranThres1 = _libData->riseTransitionLowThres();
    _tranThres2 = _libData->riseTransitionHighThres();
  } else {
    _delayThres = _libData->fallDelayThres();
    _tranThres1 = _libData->fallTransitionHighThres();
    _tranThres2 = _libData->fallTransitionLowThres();
  }
  updateLoadCaps();
  _effCap =  totalLoadOnDriver(_ckt, _cellArc->driverResistorId());
  _inputTran = _cellArc->inputTransition(_ckt);
  markSimulationScope(_cellArc->driverResistorId(), _ckt);
  updateTParams();
  updateRd();
  _tDelta = (_t50-_t20)*10/3;
  _tZero = _t50 - 0.69*_rd*_effCap - _tDelta/2;
  //if (_tZero < 0) _tZero = 1e-15;
  if (Debug::enabled(DebugModule::NLDM)) {
    printf("DEBUG: Init params: inTran: %G, Rd: %G, effCap: %G, T50: %G, outTran: %G. T20: %G, dT: %G, Tz: %G\n", 
           _inputTran, _rd, _effCap, _t50, _driverPinTran, _t20, _tDelta, _tZero);
  }
}

double 
RampVCellDelay::extrapolateDelayTime(double t50, double trans, double targetThres) const
{
  if (_isRiseOnDriverPin) {
    return ::NA::extrapolateDelayTime(t50, _delayThres, trans, targetThres, 
                                      _tranThres1, _tranThres2);
  } else {
    return ::NA::extrapolateDelayTime(t50, _delayThres, trans, targetThres, 
                                      _tranThres2, _tranThres1);
  }
}

static inline double
y0(double t, double tZero, double rd, double effCap)
{
  double tShift = t - tZero;
  double tConstant = rd * effCap;
  double inParen = 1 - exp(-tShift/tConstant);
  double a = tConstant * inParen;
  double b = tShift - a;
  return b;
}

static inline double
y(double t, double tZero, double tDelta, double rd, double effCap)
{
  double tShift = t - tZero;
  if (tShift <= 0) {
    return 0;
  } else if (tShift < tDelta) {
    return y0(t, tZero, rd, effCap) / tDelta;
  } else {
    double y0a = y0(t, tZero, rd, effCap);
    double y0b = y0(t-tDelta, tZero, rd, effCap);
    return (y0a - y0b) / tDelta;
  }
}

static inline double 
dy0dtz(double t, double tZero, double rd, double effCap)
{
  double tShift = t - tZero;
  double tConstant = rd * effCap;
  return std::exp(-tShift/tConstant) - 1;
}

static inline double
dydtz(double t, double tZero, double tDelta, double rd, double effCap)
{
  double tShift = t - tZero;
  if (tShift <= 0) {
    return 0;
  } else if (tShift < tDelta) {
    return dy0dtz(t, tZero, rd, effCap) / tDelta;
  } else {
    double dy0a = dy0dtz(t, tZero, rd, effCap);
    double dy0b = dy0dtz(t-tDelta, tZero, rd, effCap);
    return dy0a - dy0b;
  }
}

static inline double
dydtD(double t, double tZero, double tDelta, double rd, double effCap)
{
  double tShift = t - tZero;
  if (tShift <= 0) {
    return 0;
  } else if (tShift < tDelta) {
    return -y0(t, tZero, rd, effCap) / (tDelta*tDelta);
  } else {
    double dy0a = -y0(t, tZero, rd, effCap) / (tDelta * tDelta);
    double tmtD = t - tDelta;
    double dy0b = -y0(tmtD, tZero, rd, effCap) / (tmtD * tmtD);
    return dy0a + dy0b;
  }
}

static void
populatePWLData(double tDelta, double vdd, 
                bool isRise, PWLValue& pwlData)
{
  pwlData._time.clear();
  pwlData._value.clear();
  double v1 = 0;
  double v2 = vdd;
  if (isRise == false) {
    v2 = -vdd;
  }
  pwlData._time.push_back(0);
  pwlData._value.push_back(v1);
  pwlData._time.push_back(tDelta);
  pwlData._value.push_back(v2);
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

static double
simCapCharge(const SimResult& result, const Device& device)
{
  if (device._type != DeviceType::Resistor && device._type != DeviceType::VoltageSource) {
    printf("ERROR: total charge calculation is only supported on resistors and voltage sources\n");
    return 0;
  }
  if (device._type == DeviceType::Resistor) {
    const std::vector<WaveformPoint>& posWaveform = result.nodeVoltageWaveform(device._posNode);
    const std::vector<WaveformPoint>& negWaveform = result.nodeVoltageWaveform(device._negNode);
    std::vector<WaveformPoint> currentWaveform;
    currentWaveform.reserve(posWaveform.size());
    for (size_t i = 0; i < posWaveform.size(); ++i) {
      const WaveformPoint& posData = posWaveform[i];
      const WaveformPoint& negData = negWaveform[i];
      currentWaveform.push_back({posData._time, (posData._value - negData._value)/device._value});
      return totalCharge(currentWaveform);
    }
  } else {
    const std::vector<WaveformPoint>& currentWaveform = result.deviceCurrentWaveform(device._devId);
    return totalCharge(currentWaveform);
  }
  return 0;
}

static inline double
effCapCharge(double tDelta, double effCap, double rd, double vdd)
{
  double tConstant = effCap * rd;
  double parenA = tConstant * tDelta;
  double parenB = tConstant * tConstant * (1-exp(-tDelta/tConstant));
  return vdd * (parenA - parenB) / (rd * tDelta);
}

void
RampVCellDelay::updateDriverParameter()
{
  Device& driverResistor = _ckt->device(_cellArc->driverResistorId());
  driverResistor._value = _rd;
  const Device& driverSource = _ckt->device(_cellArc->driverSourceId());
  PWLValue& driverData = _ckt->PWLData(driverSource);
  double vdd = _cellArc->nldmData()->owner()->voltage();
  //populatePWLData(_tZero, _tDelta, vdd, _isRiseOnDriverPin, driverData);
  populatePWLData(_tDelta, vdd, _isRiseOnDriverPin, driverData);
}

bool
RampVCellDelay::calcIteration()
{
  RootSolver::Function f1 = [this](const Eigen::VectorXd& x)->double {
    return y(this->_t50, x(0), x(1), _rd, _effCap) - 0.5;
  };
  RootSolver::Function f2 = [this](const Eigen::VectorXd& x)->double {
    double b = delayMatchPoint / 100;
    return y(this->_t20, x(0), x(1), _rd, _effCap) - b;
  };
  RootSolver::Function df1dtz = [this](const Eigen::VectorXd& x)->double {
    return dydtz(this->_t50, x(0), x(1), _rd, _effCap);
  };
  RootSolver::Function df2dtz = [this](const Eigen::VectorXd& x)->double {
    return dydtz(this->_t20, x(0), x(1), _rd, _effCap);
  };
  RootSolver::Function df1dtD = [this](const Eigen::VectorXd& x)->double {
    return dydtD(this->_t50, x(0), x(1), _rd, _effCap);
  };
  RootSolver::Function df2dtD = [this](const Eigen::VectorXd& x)->double {
    return dydtD(this->_t20, x(0), x(1), _rd, _effCap);
  };
  RootSolver tSolver;
  tSolver.addFunction(f1);
  tSolver.addFunction(f2);
  tSolver.addDerivativeFunction(df1dtz);
  tSolver.addDerivativeFunction(df1dtD);
  tSolver.addDerivativeFunction(df2dtz);
  tSolver.addDerivativeFunction(df2dtD);
  tSolver.setInitX({_tZero, _tDelta});
  tSolver.run();
  const std::vector<double>& sol = tSolver.solution();
  _tZero = sol[0];
  _tDelta = sol[1];
  if (Debug::enabled(DebugModule::NLDM)) {
    printf("DEBUG: new tZero = %G, tDelta = %G solved after %lu iterations\n", _tZero, _tDelta, tSolver.iterCount());
  }
  if (std::isnan(_tZero) || std::isnan(_tDelta)) {
    return false;
  }
  updateDriverParameter();
  AnalysisParameter simParam;
  simParam._type = AnalysisType::Tran;
  simParam._simTime = _tDelta * 1.2;
  simParam._simTick = simParam._simTime / 1000;
  simParam._intMethod = IntegrateMethod::Trapezoidal;
  Simulator sim(*_ckt, simParam);
  if (Debug::enabled(DebugModule::NLDM)) {
    printf("DEBUG: start transient simualtion\n");
  }
  sim.run();
  const SimResult& simResult = sim.simulationResult();
  const Device& driverSource = _ckt->device(_cellArc->driverSourceId());
  double totalCharge = std::abs(simCapCharge(simResult, driverSource));
  double vdd = _cellArc->nldmData()->owner()->voltage();
  RootSolver::Function fEffCap = [this, totalCharge, vdd](const Eigen::VectorXd& x)->double {
    return effCapCharge(this->_tDelta, x(0), _rd, vdd) - totalCharge;
  };
  RootSolver cSolver;
  cSolver.addFunction(fEffCap);
  cSolver.setInitX({_effCap});
  cSolver.run();
  const std::vector<double>& solvedCap = cSolver.solution();
  double newEffCap = solvedCap[0];
  if (Debug::enabled(DebugModule::NLDM)) {
    printf("DEBUG: new effCap calculated to be %G with total charge of %G in %lu iterations\n", newEffCap, totalCharge, cSolver.iterCount());
  }
  double absDiff = std::abs((newEffCap - _effCap)/_effCap);
  if (absDiff < 0.001) {
    _finalResult.copy(simResult);
    return false;
  } else {
    _effCap = newEffCap;
    return true;
  }
}

bool
RampVCellDelay::calculate() 
{
  if (Debug::enabled(DebugModule::NLDM)) {
    printf("DEBUG: Start calculate delay of cell arc %s : %s->%s\n", 
      _cellArc->instance().data(), _cellArc->fromPin().data(), _cellArc->toPin().data());
  }
  initParameters();
  while (calcIteration()) {
    updateDriverParameter();
    updateTParams();
    updateRd();
    if (Debug::enabled(DebugModule::NLDM)) {
      printf("DEBUG: T50 updated to %G, output transition to %G, T20 to %G, Rd to %G\n", 
             _t50, _driverPinTran, _t20, _rd);
    }
  }
  return true;
}


}
