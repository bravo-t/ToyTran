#ifndef _TRAN_BASE_H_
#define _TRAN_BASE_H_

#include <vector>
#include <string>

namespace NA {

class LibData;

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
  double valueAtTime(double time) const;
  double measure(double targetValue) const;

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

  Waveform(const PWLValue& pwlValue);
  Waveform(bool isRise, double startTime, double rampTime, double voltage);
  
  void addPoint(double time, double value) 
  {
    _points.push_back({time, value});
  }
  bool isRise() const { return _points[0]._value < _points.back()._value; }
  std::vector<WaveformPoint> data() const { return _points; }
  size_t size() const { return _points.size(); }
  bool empty() const { return _points.empty(); }
  void clear() { _points.clear(); }

  double measure(double targetValue) const;
  void range(double& max, double& min) const;
  size_t indexTime(double time) const;
  double value(double time) const;
  double valueNoExtrapolation(double time) const;
  double valueAtBackStep(size_t backStep) const;
  double timeAtBackStep(size_t backStep) const;
  double transitionTime(const LibData* libData) const;
  std::vector<WaveformPoint>& data() { return _points; }

  WaveformPoint operator[](size_t index) const { return _points[index]; }
  WaveformPoint& operator[](size_t index) { return _points[index]; }
  std::vector<WaveformPoint> _points;
};

inline bool
isAnySource(const Device& dev) {
  return dev._type >= DeviceType::VoltageSource && dev._type <= DeviceType::CCVS;
}

inline double
linearInterpolate(double x1, double x2, double v1, double v2, double x)
{
  double k = (v1-v2)/(x1-x2);
  double b = v1 - x1*k;
  return k*x + b;
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
  if (dx1 == 0 && dx2 == 0) {
    return linearInterpolate(y1, y2, v11, v12, y);
  } 
  if (dy1 == 0 && dy2 == 0) {
    return linearInterpolate(x1, x2, v11, v21, x);
  }
  double div = 1 / (dx21 * dy21);
  double w11 = dx2 * dy2 * div;
  double w12 = dx2 * dy1 * div;
  double w21 = dx1 * dy2 * div;
  double w22 = dx1 * dy1 * div;
  return w11*v11 + w12*v12 + w21*v21 + w22*v22;
}

}

#endif
