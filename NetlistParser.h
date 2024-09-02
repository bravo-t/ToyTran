#ifndef _TRAN_NLPS_H_
#define _TRAN_NLPS_H_

#include <vector>
#include <string>
#include <unordered_map>
#include "Base.h"

namespace Tran {

struct ParserDevice {
  std::string _name;
  std::string _posNode;
  std::string _negNode;
  std::string _posSampleNode;
  std::string _negSampleNode;
  DeviceType  _type;
  bool        _isPWLValue = false;
  union {
    double    _value;
    size_t    _PWLData = 0;
  };
};

class NetlistParser {
  public:
    NetlistParser(const char* fileName);
    void parseLine(const std::string& line);
  
    std::vector<ParserDevice> devices() const { return _devices; }
    std::vector<PWLValue> PWLData() const { return _PWLData; }

    double simulationTime() const { return _simTime; }
    double simulationTick() const { return _simTick; }
    IntegrateMethod integrateMethod() const { return _intMethod; }

    const std::vector<std::string>& nodesToPlot() const  { return _nodeToPlot; }
    const std::vector<std::string>& devicesToPlot() const  { return _deviceToPlot; }

    bool needPlot() const { return _nodeToPlot.empty() == false || _deviceToPlot.empty() == false; }

    bool dumpData() const { return _saveData; }

  private:
    void processCommands(const std::string& line);
    void processOption(const std::string& line);


  private:
    std::vector<ParserDevice>    _devices;
    std::vector<PWLValue>        _PWLData;
    double                       _simTick = 1e-15;
    double                       _simTime = 2;
    double                       _saveData = false;
    IntegrateMethod              _intMethod = IntegrateMethod::Gear2;
    std::vector<std::string>     _nodeToPlot;
    std::vector<std::string>     _deviceToPlot;

};

}

#endif
