#ifndef _TRAN_PLTR_H_
#define _TRAN_PLTR_H_

#include "Base.h"
#include <string>
#include <vector>

namespace NA {

class Circuit;
class NetlistParser;
struct SimResult;
struct PlotData;

class Plotter {
  public:
    Plotter(const NetlistParser& parser, const std::vector<Circuit>& ckts, const std::vector<SimResult>& results); 
    void plot() const;
    static void plot(const PlotData& data, const std::vector<Circuit>& ckts, 
                     const std::vector<SimResult>& results, 
                     size_t width = -1, size_t height = -1);
    
    static void plotWaveforms(const std::vector<Waveform>& waveforms);
  private:
    void plotNodeVoltage(const std::string& nodeName, const std::string& simName, 
                         const std::vector<SimResult>& results) const;
    void plotDeviceCurrent(const std::string& devName, const std::string& simName, 
                           const std::vector<SimResult>& results) const;
    void plot(const PlotData& data) const;

  private:
    const NetlistParser&          _parser;
    const std::vector<Circuit>&   _circuits;
    const std::vector<SimResult>& _results;
};


}


#endif
