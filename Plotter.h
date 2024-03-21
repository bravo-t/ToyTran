#ifndef _TRAN_PLTR_H_
#define _TRAN_PLTR_H_

namespace Tran {

class Circuit;
class NetlistParser;
struct SimResult;

class Plotter {
  public:
    Plotter(const NetlistParser& parser, const Circuit& ckt, const SimResult& result); 
    void plot();

  private:
    const NetlistParser&   _parser;
    const Circuit&         _circuit;
    const SimResult& _result;
};


}


#endif