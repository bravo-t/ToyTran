#include "RampVDelay.h"
#include "RampVCellDelay.h"

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

void
RampVDelay::calculate()
{
  for (const CellArc* driverArc : _cellArcs) {
    calculateArc(driverArc);
  }
}

void
RampVDelay::calculateArc(const CellArc* driverArc)
{
  RampVCellDelay cellDelayCalc(driverArc, &_ckt);
  cellDelayCalc.calculate();
}


}