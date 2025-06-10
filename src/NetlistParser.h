#ifndef _TRAN_NLPS_H_
#define _TRAN_NLPS_H_

#include <vector>
#include <string>
#include <unordered_map>
#include "Base.h"

namespace NA {

struct ParserDevice {
  std::string _name;
  std::string _posNode;
  std::string _negNode;
  std::string _posSampleNode;
  std::string _negSampleNode;
  DeviceType  _type;
  bool        _isPWLValue = false;
  bool        _isInternal = false;
  union {
    double    _value;
    size_t    _PWLData = 0;
  };
  /// For cell type devices
  std::string _libCellName;
  std::unordered_map<std::string, std::string> _pinMap;
};

struct MeasurePoint {
  std::string _simName;
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
  std::vector<std::string> _nodeSimName;
  std::vector<std::string> _deviceToPlot;
  std::vector<std::string> _devSimName;
};

class NetlistParser {
  public:
    NetlistParser(const char* fileName);

    /// Circuit information
    std::vector<ParserDevice> devices() const { return _devices; }
    std::vector<PWLValue> PWLData() const { return _PWLData; }
    const std::string& userGroundNet() const { return _groundNet; }
    std::vector<std::string> libDataFiles() const { return _libDataFiles; }

    /// Plot information
    const std::vector<PlotData>& plotData() const { return _plotData; }
    std::vector<PlotData>& plotData() { return _plotData; }
    bool needPlot() const { return _plotData.empty() == false; }
    int plotWidth() const { return _plotWidth; }
    int plotHeight() const { return _plotHeight; }

    bool dumpData() const { return _saveData; }

    /// Measure information
    bool haveMeasurePoints(const std::string& simName) const;
    std::vector<MeasurePoint> measurePoints(const std::string& simName) const;

    std::vector<AnalysisParameter> analysisParameters() const { return _analysisParams; }

    std::vector<std::string> cellOutPinsToCalcDelay() const { return _cellOutPinsToCalc; }

  private:
    void parseLine(const std::string& line);
    void processCommands(const std::string& line);
    void processOption(const std::string& line);


  private:
    std::vector<ParserDevice>         _devices;
    std::vector<PWLValue>             _PWLData;
    std::vector<std::string>          _libDataFiles;
    std::vector<MeasurePoint>         _measurePoints;
    std::vector<AnalysisParameter>    _analysisParams;
    std::vector<std::string>          _cellOutPinsToCalc;
    bool                              _saveData = false;
    size_t                            _plotWidth = static_cast<size_t>(-1);
    size_t                            _plotHeight = static_cast<size_t>(-1);
    std::string                       _groundNet;
    std::vector<PlotData>             _plotData;
};

}

#endif
