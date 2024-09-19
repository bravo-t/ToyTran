#ifndef _TRAN_PLTR_H_
#define _TRAN_PLTR_H_

#include <string>

namespace Tran {

class Circuit;
class NetlistParser;
struct SimResult;

class Plotter {
  public:
    Plotter(const NetlistParser& parser, const Circuit& ckt, const SimResult& result); 
    void plot() const;

  private:
    void plotNodeVoltage(const std::string& nodeName, const Circuit& ckt, const SimResult& result) const;
    void plotDeviceCurrent(const std::string& devName, const Circuit& ckt, const SimResult& result) const;

  private:
    const NetlistParser&   _parser;
    const Circuit&         _circuit;
    const SimResult& _result;
};


}


#endif