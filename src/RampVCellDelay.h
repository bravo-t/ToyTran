#ifndef _NA_RAMPV_CDLY_H_
#define _NA_RAMPV_CDLY_H_

#include "Circuit.h"
#include "LibData.h"

namespace NA {

class CellArc;
class Circuit;

class RampVCellDelay {
  public: 
    RampVCellDelay(const CellArc* cellArc, Circuit* ckt)
    : _cellArc(cellArc), _ckt(ckt), _libData(cellArc->nldmData()->owner()) {}

    bool calculate();

  private:
    void initParameters();
    void updateParameters();
    double extrapolateDelayTime(double t50, double trans, double targetThres) const;
    void updateTParams();
    void updateRd();
    void updateLoadCaps();
    double calcIteration();
    void updateDriverParameter();

  private:
    const CellArc* _cellArc;
    Circuit* _ckt;
    const LibData* _libData;
    bool   _isRiseOnInputPin = true;
    bool   _isRiseOnDriverPin = true;
    double _delayThres = 50;
    double _tranThres1 = 10;
    double _tranThres2 = 90;
    double _inputTran = 0;
    double _driverPinTran = 0;
    double _effCap = 0;
    double _tZero = 0;
    double _tDelta = 0;
    double _rd = 0;
    double _t50 = 0;
    double _t20 = 0;
};

}

#endif
