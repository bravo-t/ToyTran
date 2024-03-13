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

  private:
    void processCommands(const std::string& line);


  private:
    std::vector<std::string>     _nodes;
    std::vector<Device>          _devices;
    std::vector<PWLValue>        _PWLData;
    double                       _simTick = 1e-15;
    double                       _simTime = 2;

};

}

#endif
