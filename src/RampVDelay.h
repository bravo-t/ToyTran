#ifndef _NA_RAMPV_DLY_H_
#define _NA_RAMPV_DLY_H_

namespace NA {

class CellArc;
class Circuit;

class RampVDelay {
  public: 
    RampVDelay(const CellArc* cellArc, const Circuit* ckt)
    : _cellArc(cellArc), _ckt(ckt) {}

    bool calculate();

  private:
    const CellArc* _cellArc;
    const Circuit* _ckt;
    double _tZero = 0;
    double _tDelta = 0;
    double _rd = 0;
};

}

#endif