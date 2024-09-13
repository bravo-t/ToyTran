#ifndef _TRAN_BASE_H_
#define _TRAN_BASE_H_

#include <vector>
#include <string>

namespace Tran {

enum class SimResultType : unsigned char {
  Voltage,
  Current,
};

enum class IntegrateMethod : unsigned char {
  None,
  BackwardEuler,
  Trapezoidal,
  Gear2,
};

enum class DeviceType : unsigned char {
  Resistor,
  Capacitor,
  Inductor,
  VoltageSource,
  CurrentSource,
  VCCS, /// Voltage controlled current source, etc
  VCVS,
  CCCS,
  CCVS,

  Total
};

struct Device {
  std::string _name;
  size_t      _devId = static_cast<size_t>(-1);
  size_t      _posNode = static_cast<size_t>(-1);
  size_t      _negNode = static_cast<size_t>(-1);
  size_t      _posSampleNode = static_cast<size_t>(-1);
  size_t      _negSampleNode = static_cast<size_t>(-1);
  size_t      _sampleDevice = static_cast<size_t>(-1);
  DeviceType  _type;
  bool        _isPWLValue = false;
  union {
    double    _value;
    size_t    _PWLData = 0;
  };
};

struct Node {
  size_t              _nodeId = static_cast<size_t>(-1);
  bool                _isGround = false;
  std::string         _name;
  std::vector<size_t> _connection;
};

struct PWLValue {
  std::vector<double> _time;
  std::vector<double> _value;
};

inline bool
isAnySource(const Device& dev) {
  return dev._type >= DeviceType::VoltageSource;
}

}

#endif
