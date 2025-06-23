#include <cassert>
#include "RampVDelay.h"
#include "RampVCellDelay.h"
#include "Simulator.h"
#include "Debug.h"

namespace NA {

RampVDelay::RampVDelay(const AnalysisParameter& param, const NetlistParser& parser)
: _ckt(parser, param)
{
  const std::vector<std::string>& pinsToCalc = parser.cellOutPinsToCalcDelay();
  for (const std::string& outPin : pinsToCalc) {
    const std::vector<std::string>& cellInPins = _ckt.cellArcFromPins(outPin);
    for (const std::string& frPin : cellInPins) {
      const CellArc* driverArc = _ckt.cellArc(frPin, outPin);
      if (driverArc == nullptr) {
        printf("ERROR: Cannot find cell arc connected on pin %s\n", frPin.data());
        continue;
      }
      _cellArcs.push_back(driverArc);
    }
  }
}

std::vector<const CellArc*>
setTerminationCondition(const Circuit* ckt, const CellArc* driverArc, 
                        bool isRiseOnDriverPin, Simulator& sim)
{
  size_t rdId = driverArc->driverResistorId();
  const std::vector<const Device*>& connDevs = ckt->traceDevice(rdId);
  std::vector<const CellArc*> retval;
  for (const Device* dev : connDevs) {
    if (dev->_type == DeviceType::Capacitor && dev->_isInternal) {
      const std::vector<CellArc*>& loadArcs = ckt->cellArcsOfDevice(dev);
      double termVoltage = 0;
      const CellArc* loadArc = nullptr;
      for (const CellArc* cellArc : loadArcs) {
        const LibData* libData = cellArc->libData();
        double libVoltage = libData->voltage();
        double termPoint = libData->riseTransitionHighThres();
        if (isRiseOnDriverPin == false) {
          termPoint = 100 - libData->fallTransitionLowThres();
        }
        double v = libVoltage * termPoint / 100;
        if (isRiseOnDriverPin) {
          if (v > termVoltage) {
            termVoltage = v;
            loadArc = cellArc;
          }
        } else {
          if (v < termVoltage) {
            termVoltage = v;
            loadArc = cellArc;
          }
        }
      }
      retval.push_back(loadArc);
      const Node& posNode = ckt->node(dev->_posNode);
      const Node& negNode = ckt->node(dev->_negNode);
      assert(posNode._isGround != negNode._isGround);
      if (posNode._isGround) {
        sim.setTerminationVoltage(negNode._nodeId, termVoltage);
      } else {
        sim.setTerminationVoltage(posNode._nodeId, termVoltage);
      }
    }
  }
  return retval;
}

void
RampVDelay::calculate()
{
  for (const CellArc* driverArc : _cellArcs) {
    calculateArc(driverArc);
  }
}

void
measureVoltage(const SimResult& result, size_t nodeId, const LibData* libData,  
               double& delay, double& trans)
{
  const Waveform& nodeVoltage = result.nodeVoltageWaveform(nodeId);
  bool isRise = nodeVoltage.isRise();
  double delayThres = libData->riseDelayThres();
  double lowerThres = libData->riseTransitionLowThres();
  double upperThres = libData->riseTransitionHighThres();
  if (isRise == false) {
    delayThres = libData->fallDelayThres();
    lowerThres = libData->fallTransitionLowThres();
    upperThres = libData->fallTransitionHighThres();
  }
  double libVoltage = libData->voltage();
   
  delay = nodeVoltage.measure(delayThres / 100 * libVoltage);
  if (nodeVoltage.isRise()) {
    double transLower = nodeVoltage.measure(lowerThres / 100 * libVoltage);
    double transUpper = nodeVoltage.measure(upperThres / 100 * libVoltage);
    trans = transUpper - transLower;
  } else {
    double upperVoltage = (upperThres - 100) / 100 * libVoltage;
    double lowerVoltage = (lowerThres - 100) / 100 * libVoltage;
    double transLower = nodeVoltage.measure(lowerVoltage);
    double transUpper = nodeVoltage.measure(upperVoltage);
    trans = transLower - transUpper;
  }
}

void
RampVDelay::calculateArc(const CellArc* driverArc)
{
  RampVCellDelay cellDelayCalc(driverArc, &_ckt);
  cellDelayCalc.calculate();
  if (Debug::enabled(DebugModule::NLDM)) {
    printf("DEBUG: Starting network simulation for net arc delay calculation\n");
  }
  double tOffset = cellDelayCalc.tZero();
  AnalysisParameter simParam;
  simParam._name = "Delay calculation";
  simParam._type = AnalysisType::Tran;
  simParam._simTime = 1e99;
  simParam._simTick = cellDelayCalc.tDelta() / 1000;
  simParam._intMethod = IntegrateMethod::Trapezoidal;
  Simulator sim(_ckt, simParam);
  const std::vector<const CellArc*>& loadArcs = setTerminationCondition(&_ckt, driverArc, cellDelayCalc.isRiseOnOutputPin(), sim);
  sim.run();
  const SimResult& simResult = sim.simulationResult();
  const LibData* libData = driverArc->libData();
  const Device& inputSrc = _ckt.device(driverArc->inputSourceDevId(&_ckt));
  size_t inputNodeId = driverArc->inputNode(&_ckt);
  double inputT50;
  double inputTran;
  measureVoltage(simResult, inputNodeId, libData, inputT50, inputTran);
  size_t outputNodeId = driverArc->outputNode(&_ckt);
  double outputT50;
  double outputTran;
  measureVoltage(simResult, outputNodeId, libData, outputT50, outputTran);
  double cellDelay = outputT50 - inputT50 + tOffset;
  printf("Cell delay of %s:%s->%s: %G, transition on output pin: %G\n", driverArc->instance().data(), driverArc->fromPin().data(), 
          driverArc->toPin().data(), cellDelay, outputTran);
  for (const CellArc* loadArc : loadArcs) {
    size_t loadNode = loadArc->inputNode(&_ckt);
    double loadT50;
    double loadTran;
    measureVoltage(simResult, loadNode, loadArc->libData(), loadT50, loadTran);
    double netDelay = loadT50 - outputT50;
    printf("Net delay of %s->%s: %G, transition on %s: %G\n", driverArc->toPinFullName().data(), 
           loadArc->fromPinFullName().data(), netDelay, loadArc->fromPinFullName().data(), loadTran);
  }
}


}
