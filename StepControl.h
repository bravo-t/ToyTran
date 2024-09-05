#ifndef _TRAN_STPCTL_H_
#define _TRAN_STPCTL_H_

namespace Tran {

class Simulator;

class LTE {
  public:
    static double maxLTE(const Simulator* sim);
};

class StepControl {
  public:
    static double stepLimit(const Simulator* sim, double relTol);
};

}

#endif