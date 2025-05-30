#ifndef _NA_RAMPV_DLY_H_
#define _NA_RAMPV_DLY_H_

#include <tuple>
#include <vector>
#include "Base.h"
#include "NetlistParser.h"
#include "Circuit.h"

namespace NA {

typedef std::pair<const CellArc*, const CellArc*> ArcPair;

class RampVDelay {
  public:
    RampVDelay(const AnalysisParameter& param, const NetlistParser& parser);

    void calculate();

  private:
    void calculateArc(const CellArc* driverArc, const CellArc* loaderArc);

  private:
    Circuit _ckt;
    std::vector<ArcPair> _cellArcs;
};

}

#endif