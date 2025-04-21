#ifndef _TRAN_WRITER_H_
#define _TRAN_WRITER_H_

#include <string>

namespace NA {

class Circuit;
struct SimResult;

class TR0Writer {
  public:
    TR0Writer(const Circuit& circuit, const std::string& outputFile)
    : _ckt(circuit), _outFile(outputFile) {}
    void adjustNumberWidth(double simTick, double simTime);
    void writeData(const SimResult& result) const;

  private:
    const Circuit& _ckt;
    std::string    _outFile;
    int            _significandWidth = 9;
    int            _exponentWidth = 3; // Plus the "+"/"-" sign
};

}

#endif