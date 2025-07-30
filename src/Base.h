#ifndef _TRAN_BASE_H_
#define _TRAN_BASE_H_

#include <vector>
#include <string>

namespace NA {

enum class AnalysisType : unsigned char {
  None,
  Tran, /// Transient analysis
  PZ,   /// Pole-Zero anlaysis
  TF,   /// Transfer function analysis
  FD,   /// Full-stage delay analysis
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

enum class NetworkModel : unsigned char {
  Tran, /// transient simulation is used for network delay calculation, WIP
  PZ,   /// Pole-Zero analysis is used for net delay calculation, to-be-implemented
};

enum class DriverModel : unsigned char {
  None,
  /// The driver is characterized with a ramp voltage source and a resistor 
  /// connected in series to the voltage source, as stated in 
  /// F. Dartu, N. Menezes and L. T. Pileggi, 
  /// "Performance computation for precharacterized CMOS gates with RC loads," 
  /// in IEEE Transactions on Computer-Aided Design of Integrated Circuits and Systems, 
  /// vol. 15, no. 5, pp. 544-553, May 1996, doi: 10.1109/43.506141.
  RampVoltage, 
  /// Current source model, details to be implemented.
  PWLCurrent,
};

enum class LoaderModel : unsigned char {
  None,
  /// Capacitors with fixed values,
  Fixed,
  /// Capacitors with values depend on input transition and output load
  Varied,
};

struct AnalysisParameter {
  ~AnalysisParameter() 
  {
    if (_type == AnalysisType::PZ) {
      delete _inDev;
      delete _outNode;
    }
  }
  AnalysisType _type = AnalysisType::None;
  bool         _hasMeasurePoints = false;
  std::string  _name;
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
      unsigned int  _order = 0;
      std::string*  _inDev;
      std::string*  _outNode;
    };
    /// Parameters for full-stage delay calculation
    struct {
      DriverModel  _driverModel;
      LoaderModel    _loaderModel;
      NetworkModel _netModel;
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
  Cell, /// Special kind of device represent lib cells

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
  bool        _isInternal = false; /// this is true if the device is elaborated from Cell devices
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
  double measure(double targetValue) const
  {
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    for (size_t i=1; i<_value.size(); ++i) {
      if ((_value[i-1] <= targetValue && _value[i] >= targetValue) ||
          (_value[i-1] >= targetValue && _value[i] <= targetValue)) {
        x1 = _time[i-1];
        y1 = _value[i-1];
        x2 = _time[i];
        y2 = _value[i];
        break;
      }
    }
    if (x1 == 0 && x2 == 0) {
      return 1e99;
    }
    double k = (y2 - y1) / (x2 - x1);
    double b = y1 - k * x1;
    return (targetValue - b) / k;
  }

  bool isRiseTransition() const 
  {
    return _value[0] < _value.back();
  }

  std::vector<double> _time;
  std::vector<double> _value;
};

struct WaveformPoint {
  double _time = 0;
  double _value = 0;
};

struct Waveform {
  Waveform() = default;
  Waveform(const std::vector<WaveformPoint>& points)
  : _points(points) {}

  Waveform(const PWLValue& pwlValue)
  {
    _points.reserve(pwlValue._value.size());
    for (size_t i=0; i<pwlValue._value.size(); ++i) {
      _points.push_back({pwlValue._time[i], pwlValue._value[i]});
    }
  }

  Waveform(double startTime, double rampTime, double voltage) 
  {
    _points.push_back({0, 0});
    if (startTime > 0) {
      _points.push_back({startTime, 0});
    }
    _points.push_back({startTime+rampTime, voltage});
  }

  void addPoint(double time, double value) 
  {
    _points.push_back({time, value});
  }

  double measure(double targetValue) const
  {
    bool isRise = this->isRise();
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    for (size_t i=1; i<_points.size(); ++i) {
      if ((isRise && _points[i-1]._value <= targetValue && _points[i]._value >= targetValue) || 
         (!isRise && _points[i-1]._value >= targetValue && _points[i]._value <= targetValue)) {
        x1 = _points[i-1]._time;
        y1 = _points[i-1]._value;
        x2 = _points[i]._time;
        y2 = _points[i]._value;
        break;
      }
    }
    if (x1 == 0 && x2 == 0) {
      return 1e99;
    }
    double k = (y2 - y1) / (x2 - x1);
    double b = y1 - k * x1;
    return (targetValue - b) / k;
  }
  
  bool isRise() const { return _points[0]._value < _points.back()._value; }
  
  std::vector<WaveformPoint> data() const { return _points; }
  
  void range(double& max, double& min) const
  {
    for (const WaveformPoint& p : _points) {
      max = std::max(max, p._value);
      min = std::min(min, p._value);
    }
  }

  size_t indexTime(double time) const 
  {
    size_t lower = 1;
    size_t upper = _points.size()-2;
    if (time <= _points[lower]._time) {
      return 0;
    }
    if (time >= _points[upper]._time) {
      return upper;
    }
    size_t idx = 0;
    while (upper - lower > 1) {
      idx = (lower + upper) >> 1;
      if (_points[idx]._time > time) {
        upper = idx;
      } else {
        lower = idx;
      }
    }
    if (_points[idx]._time > time) {
      --idx;
    }
    return idx;
  }

  double value(double time) const 
  {
    size_t idx1 = indexTime(time);
    size_t idx2 = idx1 + 1;
    double t1 = _points[idx1]._time;
    double v1 = _points[idx1]._value;
    double t2 = _points[idx2]._time;
    double v2 = _points[idx2]._value;
    
    double k = (v2 - v1) / (t2 - t1);
    double b = v1 - k * t1;
    return k * time + b;
  }
  size_t size() const { return _points.size(); }
  WaveformPoint operator[](size_t index) const { return _points[index]; }
  WaveformPoint& operator[](size_t index) { return _points[index]; }
  std::vector<WaveformPoint> _points;
};

inline bool
isAnySource(const Device& dev) {
  return dev._type >= DeviceType::VoltageSource && dev._type <= DeviceType::CCVS;
}

/// v11 = f(x1, y1)
/// v12 = f(x1, y2)
/// v21 = f(x2, y1)
/// v22 = f(x2, y2)
inline double
bilinearInterpolate(double x1, double y1, double x2, double y2, 
                    double v11, double v12, double v21, double v22, 
                    double x, double y)
{
  double dx2 = x2 - x;
  double dx1 = x - x1;
  double dx21 = x2 - x1;
  double dy2 = y2 - y;
  double dy1 = y - y1;
  double dy21 = y2 - y1;
  double div = 1 / (dx21 * dy21);
  double w11 = dx2 * dy2 * div;
  double w12 = dx2 * dy1 * div;
  double w21 = dx1 * dy2 * div;
  double w22 = dx1 * dy1 * div;
  return w11*v11 + w12*v12 + w21*v21 + w22*v22;
}

}

#endif
