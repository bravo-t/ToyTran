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

  private:
    std::vector<std::string>     _nodes;
    std::vector<Device>          _devices;
    std::vector<PWLValue>        _PWLData;

};

}

#endif
