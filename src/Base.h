#ifndef _TRAN_BASE_H_
#define _TRAN_BASE_H_

#include <vector>
#include <string>

namespace NA {

enum class AnalysisType : unsigned char {
  Tran, /// Transient analysis
  PZ,   /// Pole-Zero anlaysis
  TF    /// Transfer function analysis
};

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

struct AnalysisParameter {
  AnalysisType _type;
  union {
    /// Parameters for transient analysis
    struct {
      double          _relTotal;
      double          _simTime;
      double          _simTick;
      IntegrateMethod _intMethod;
    };
    /// Parameters for pole-zero analysis
    struct {
      unsigned int  _order;
      std::string*  _inDev;
      std::string*  _outNode;
    };
  };
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
  double valueAtTime(double time) const
  {
    if (time < _time[0]) {
      return 0;
    }
    for (size_t i=1; i<_time.size(); ++i) {
      if (time < _time[i]) {
        double v1 = _value[i-1];
        double v2 = _value[i];
        double t1 = _time[i-1];
        double t2 = _time[i];
        //printf("DEBUG: time %g goes into [%g, %g] interval, voltage: [%g, %g], interpolated voltage: %g\n", 
        //  time, t1, t2, v1, v2, v1 + (v2-v1)/(t2-t1)*(time-t1));
        return v1 + (v2-v1)/(t2-t1)*(time-t1);
      }
    }
    return _value.back();
  }

  std::vector<double> _time;
  std::vector<double> _value;
};

inline bool
isAnySource(const Device& dev) {
  return dev._type >= DeviceType::VoltageSource;
}

}

#endif
