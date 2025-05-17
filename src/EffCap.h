#ifndef _NA_EFFCAP_H_
#define _NA_EFFCAP_H_

#include "Circuit.h"
#include "SimResult.h"

namespace NA {

class EffCap {
  public:
    static double value(const Circuit& ckt, const SimResult& result);

  private:
    static double charge(const Circuit& ckt, const SimResult& result, const Device& device);

};

}

#endif
