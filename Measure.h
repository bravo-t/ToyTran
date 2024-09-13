#ifndef _TRAN_MEAS_H_
#define _TRAN_MEAS_H_

#include <vector>
#include "NetlistParser.h"
#include "Simulator.h"

namespace Tran {

class Measure {
  public:
    Measure(const Simulator& simulator, const std::vector<MeasurePoint>& measurePoints)
    : _simulator(simulator), _measurePoints(measurePoints) {}
    ~Measure() {}

    void run() const;

  private:
    const Simulator& _simulator;
    const std::vector<MeasurePoint>& _measurePoints;
};

}

#endif