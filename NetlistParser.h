#ifndef _TRANS_NLPS_H_
#define _TRANS_NLPS_H_

#include <vector>
#include <string>

namespace Tran {

enum class DeviceType : unsigned char {
  Resistor,
  Capacitor,
  Inductor,
  VoltageSource,
  CurrentSource,
  VCCS, /// Voltage controlled current source, etc
  VCVS,
  CCCS,
  CCVS
};

struct Device {
  std::string _name;
  size_t _posNode = 0;
  size_t _negNode = 0;
  DeviceType _type;
  bool _isPWLValue = false;
  union {
    double _value;
    size_t _PWLData = 0;
  };
};

struct PWLValue {
  std::vector<double> _time;
  std::vector<double> _value;
};

class NetlistParser {
  public:
    NetlistParser(const char* fileName);
    void parseLine(const std::string& line);
  private:


  private:
    std::vector<std::string> _nodes;
    std::vector<Device>      _devices;
    std::vector<PWLValue>    _PWLData;

};

}

#endif
