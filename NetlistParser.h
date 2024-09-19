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

struct MeasurePoint {
  std::string _variableName;
  double _timeDelay = .0f;
  std::string _trigger;
  SimResultType _triggerType;
  double _triggerValue;
  std::string _target;
  SimResultType _targetType;
  double _targetValue;
};

struct PlotData {
  std::string              _canvasName;
  std::vector<std::string> _nodeToPlot;
  std::vector<std::string> _deviceToPlot;
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

    const std::vector<PlotData>& plotData() const { return _plotData; }
    std::vector<PlotData>& plotData() { return _plotData; }

    bool needPlot() const { return _plotData.empty() == false; }
    int plotWidth() const { return _plotWidth; }
    int plotHeight() const { return _plotHeight; }

    bool dumpData() const { return _saveData; }

    double relTol() const { return _relTol; }

    bool haveMeasurePoints() const { return !_measurePoints.empty(); }
    const std::vector<MeasurePoint>& measurePoints() const { return _measurePoints; }

  private:
    void processCommands(const std::string& line);
    void processOption(const std::string& line);


  private:
    std::vector<ParserDevice>    _devices;
    std::vector<PWLValue>        _PWLData;
    std::vector<MeasurePoint>     _measurePoints;
    double                       _simTick = 1e-15;
    double                       _simTime = 2;
    double                       _relTol = 1e-6;
    IntegrateMethod              _intMethod = IntegrateMethod::Gear2;
    bool                         _saveData = false;
    size_t                       _plotWidth = static_cast<size_t>(-1);
    size_t                       _plotHeight = static_cast<size_t>(-1);
    std::vector<PlotData>        _plotData;
};

}

#endif
