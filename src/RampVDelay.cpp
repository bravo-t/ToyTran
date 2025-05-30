#include "RampVDelay.h"
#include "RampVCellDelay.h"

namespace NA {

RampVDelay::RampVDelay(const AnalysisParameter& param, const NetlistParser& parser)
: _ckt(parser, param)
{
  const std::vector<NetlistParser::StringPair>& delayArcsToCalculate = parser.delayArcs();
  for (const auto& pair : delayArcsToCalculate) {
    const std::string& fromPin = pair.first;
    const std::string& toPin = pair.second;
    const CellArc* driverArc = _ckt.cellArc(fromPin);
    const CellArc* loaderArc = _ckt.cellArc(toPin);
    if (driverArc == nullptr || loaderArc == nullptr) {
      if (driverArc == nullptr) {
        printf("ERROR: Cannot find cell arc connected on pin %s\n", fromPin.data());
      }
      if (loaderArc == nullptr) {
        printf("ERROR: Cannot find cell arc connected to pin %s\n", toPin.data());
      }
      continue;
    }
    _cellArcs.push_back({driverArc, loaderArc});
  }
}

void
RampVDelay::calculate()
{
  for (const auto& arcPair : _cellArcs) {
    calculateArc(arcPair.first, arcPair.second);
  }
}

void
RampVDelay::calculateArc(const CellArc* driverArc, const CellArc* loaderArc)
{
  RampVCellDelay cellDelayCalc(driverArc, &_ckt);
  cellDelayCalc.calculate();
}


}