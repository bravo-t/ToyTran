#ifndef _TRAN_NLPS_H_
#define _TRAN_NLPS_H_

#include <vector>
#include <string>
#include <unordered_map>
#include "Base.h"

namespace Tran {

class NetlistParser {
  public:
    NetlistParser(const char* fileName);
    void parseLine(const std::string& line, 
                   std::unordered_map<std::string, size_t>& nodeMap);
  
    std::vector<std::string> nodes() const { return _nodes; }
    std::vector<Device> devices() const { return _devices; }
    std::vector<PWLValue> PWLData() const { return _PWLData; }

    double simulationTime() const { return _simTime; }
    double simulationTick() const { return _simTick; }
    IntegrateMethod integrateMethod() const { return _intMethod; }

    const std::vector<std::string>& nodesToPlot() const  { return _nodeToPlot; }
    const std::vector<std::string>& devicesToPlot() const  { return _deviceToPlot; }

  private:
    void processCommands(const std::string& line);
    void processOption(const std::string& line);


  private:
    std::vector<std::string>     _nodes;
    std::vector<Device>          _devices;
    std::vector<PWLValue>        _PWLData;
    double                       _simTick = 1e-15;
    double                       _simTime = 2;
    IntegrateMethod              _intMethod = IntegrateMethod::Gear2;
    std::vector<std::string>     _nodeToPlot;
    std::vector<std::string>     _deviceToPlot;

};

}

#endif
