#ifndef _TRANS_BASE_H_
#define _TRANS_BASE_H_

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

}

#endif
