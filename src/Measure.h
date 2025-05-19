#ifndef _TRAN_MEAS_H_
#define _TRAN_MEAS_H_

#include <vector>
#include "NetlistParser.h"
#include "SimResult.h"

namespace NA {

class Measure {
  public:
    Measure(const SimResult& result, const std::vector<MeasurePoint>& measurePoints)
    : _simResult(result), _measurePoints(measurePoints) {}
    ~Measure() {}

    void run() const;

  private:
    const SimResult& _simResult;
    std::vector<MeasurePoint> _measurePoints;
};

}

#endif