#ifndef _TRANS_NLPS_H_
#define _TRANS_NLPS_H_

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
  private:


  private:
    std::vector<std::string> _nodes;
    std::vector<Device>      _devices;
    std::vector<PWLValue>    _PWLData;

};

}

#endif
