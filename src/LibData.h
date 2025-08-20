#ifndef _NA_LIBDAT_H_
#define _NA_LIBDAT_H_

#include <cstddef>
#include <unordered_map>
#include <vector>
#include <string>

namespace NA {

class LibData;
class CCSArc;

enum class LUTType {
  RiseDelay,
  FallDelay,
  RiseTransition,
  FallTransition,
  RiseCurrent,
  FallCurrent,
  RiseRecvCap,
  FallRecvCap,

};

class NLDMLUT {
  public:
    NLDMLUT() = default;
    NLDMLUT(const NLDMLUT& other) = default;
    /*
    : _index1(other._index1), 
      _index2(other._index2), 
      _values(other._values) {}
      */

    void reset()
    {
      _index1.clear();
      _index2.clear();
      _values.clear();
    }
    void setIndex1(const std::vector<double>& index1) { _index1.assign(index1.begin(), index1.end()); }
    void setIndex2(const std::vector<double>& index2) { _index2.assign(index2.begin(), index2.end()); }
    void setValues(const std::vector<double>& values) { _values.assign(values.begin(), values.end()); }

    size_t axis1Index(double value) const;
    size_t axis2Index(double value) const;
    
    double value(double inputTran, double outputLoad) const;

    bool empty() const { return _values.empty(); }

    void indexValues(const std::vector<double>& values, size_t X, size_t Y, 
                     double& Z1, double& Z2, double& Z3, double& Z4) const;

    std::vector<double> index1Values() const { return _index1; }
    std::vector<double> index2Values() const { return _index2; }
    std::vector<double> values() const { return _values; }

  private:
    std::vector<double> _index1;
    std::vector<double> _index2;
    std::vector<double> _values;
};

class NLDMArc {
  public:
    NLDMArc(LibData* owner)
    : _owner(owner) {}

    void reset() 
    {
      _fromPin.clear();
      _toPin.clear();
      _isInverted = false;
      _riseDelay.reset();
      _fallDelay.reset();
      _riseTransition.reset();
      _fallTransition.reset();
    }
    void setFromToPin(const std::string& fromPin, const std::string& toPin, bool isInverted)
    {
      _fromPin = fromPin; 
      _toPin = toPin; 
      _isInverted = isInverted;
    }   
    NLDMLUT& getLUT(LUTType dataType);
    const NLDMLUT& getLUT(LUTType dataType) const;

    const char* fromPin() const { return _fromPin.data(); }
    const char* toPin() const { return _toPin.data(); }
    bool empty() const { return _riseDelay.empty() && _fallDelay.empty() &&
                                _riseTransition.empty() && _fallTransition.empty(); }

    const LibData* owner() const { return _owner; }
    bool isInverted() const { return _isInverted; }

  private:
    const LibData* _owner = nullptr;
    std::string    _fromPin;
    std::string    _toPin;
    bool           _isInverted; /// negative_unate -> true
    NLDMLUT        _riseDelay;
    NLDMLUT        _fallDelay;
    NLDMLUT        _riseTransition;
    NLDMLUT        _fallTransition;
};

class CCSLUT {
  public:
    CCSLUT() = default;
    
    void init(double refTime, double index1, double index2,
              const std::vector<double>& index3, 
              const std::vector<double>& values)
    {
      _referenceTime = refTime;
      _index1 = index1;
      _index2 = index2;
      _index3.assign(index3.begin(), index3.end());
      _values.assign(values.begin(), values.end());
    }

    double inputTransition() const { return _index1; }
    double outputLoad() const { return _index2; }
    double referenceTime() const { return _referenceTime; }
    std::vector<double> times() const { return _index3; }
    std::vector<double> values() const { return _values; }

    void reset()
    {
      _referenceTime = 0;
      _index1 = 0;
      _index2 = 0;
      _index3.clear();
      _values.clear();
    }

    bool empty() const { return _values.empty(); }

  private:
    double              _referenceTime;
    double              _index1;
    double              _index2;
    std::vector<double> _index3;
    std::vector<double> _values;
};

class CCSGroup {
  public:
    CCSGroup() = default;
    void addLUT(const CCSLUT& data) { _ccsluts.push_back(data); }
    CCSLUT value(double inputTran, double outputLoad) const;
    void sortTable();
    std::vector<size_t> searchSteps() const { return _transDiv; }

    bool empty() const { return _ccsluts.empty(); }
    std::vector<CCSLUT> tables() const { return _ccsluts; }
    
    void reset() 
    { 
      _ccsluts.clear(); 
      _transDiv.clear();
    }

  private:
    std::vector<CCSLUT> _ccsluts;
    std::vector<size_t> _transDiv;
};

class CCBOutputVoltageLUT {
  public:
    CCBOutputVoltageLUT() = default;
    void init(double inputTran, double outputLoad, 
              const std::vector<double>& time,
              const std::vector<double>& voltage)
    {
      _inputTran = inputTran;
      _outputLoad = outputLoad;
      _time.assign(time.begin(), time.end());
      _voltage.assign(voltage.begin(), voltage.end());
    }
    
    double inputTransition() const { return _inputTran; }
    double outputLoad() const { return _outputLoad; }
    std::vector<double> times() const { return _time; }
    std::vector<double> values() const { return _voltage; }

  private:  
    double _inputTran = 0;
    double _outputLoad = 0;
    std::vector<double> _time;
    std::vector<double> _voltage;
};

class CCBOutputVoltage {
  public:
    CCBOutputVoltage() = default;
    void addLUT(const CCBOutputVoltageLUT& lut) { _lutData.push_back(lut); }

    void sortTable();
    void reset() 
    {
      _lutData.clear();
      _transDiv.clear();
    }

    std::vector<CCBOutputVoltageLUT> tables() const { return _lutData; }
    std::vector<size_t> searchSteps() const { return _transDiv; }
  private:  
    std::vector<CCBOutputVoltageLUT> _lutData;
    std::vector<size_t> _transDiv;
};

class CCBData {
  public: 
    CCBData() = default;
    void setIsInverting(bool val) { _isInverting = val; }
    void setMillerCaps(double rise, double fall) { _millerCapRise = rise; _millerCapFall = fall; }
    NLDMLUT& getDcCurrent() { return _dcCurrent; }
    CCBOutputVoltage& getRiseOutputVoltage() { return _riseVoltage; }
    CCBOutputVoltage& getFallOutputVoltage() { return _fallVoltage; }
    const CCBOutputVoltage& getRiseOutputVoltage() const { return _riseVoltage; }
    const CCBOutputVoltage& getFallOutputVoltage() const { return _fallVoltage; }
    void setRiseOutputVoltage(const CCBOutputVoltage& val) { _riseVoltage = val; }
    void setFallOutputVoltage(const CCBOutputVoltage& val) { _fallVoltage = val; }

    double dcCurrent(double Vin, double Vout) const { return _dcCurrent.value(Vin, Vout); }
    double millerCap(bool isRise) const
    {
      if (isRise) {
        return _millerCapRise;
      } else {
        return _millerCapFall;
      }
    }
    
    bool isInverting() const { return _isInverting; }
    void reset() 
    {
      _millerCapRise = 0;
      _millerCapFall = 0;
      _dcCurrent.reset();
      _riseVoltage.reset();
      _fallVoltage.reset();
    }

  private:
    bool _isInverting = true;
    double _millerCapRise = 0;
    double _millerCapFall = 0;
    NLDMLUT _dcCurrent;
    CCBOutputVoltage _riseVoltage;
    CCBOutputVoltage _fallVoltage;
};

class CCSArc {
  public:
    CCSArc(const LibData* owner)
    : _owner(owner) {}

    void reset()
    {
      _fromPin.clear();
      _toPin.clear();
      _isInverted = false;
      _riseCurrent.reset();
      _fallCurrent.reset();
      _riseRecvCaps.clear();
      _fallRecvCaps.clear();
      _firstStageCCBData.reset();
      _lastStageCCBData.reset();
    }
    void setFromToPin(const std::string& fromPin, const std::string& toPin, bool isInverted)
    {
      _fromPin = fromPin; 
      _toPin = toPin;
      _isInverted = isInverted;
    }   

    void setMillerCaps(double riseCap, double fallCap)
    {
      _riseMillerCap = riseCap; _fallMillerCap = fallCap;
    }

    std::vector<NLDMLUT>& getRecvCap(LUTType dataType);
    CCSGroup& getCurrent(LUTType dataType);
    NLDMLUT& getDCCurrent();
    CCBData& ccbFirstStageData() { return _firstStageCCBData; }
    CCBData& ccbLastStageData() { return _lastStageCCBData; }
    const CCBData* ccbFirstStageData() const { return &_firstStageCCBData; }
    const CCBData* ccbLastStageData() const { return &_lastStageCCBData; }
    const std::vector<NLDMLUT>& getRecvCap(LUTType dataType) const;
    const CCSGroup& getCurrent(LUTType dataType) const;
    const NLDMLUT& getDCCurrent() const; 
    
    const char* fromPin() const { return _fromPin.data(); }
    const char* toPin() const { return _toPin.data(); }
    bool isInverted() const { return _isInverted; }

    bool empty() const { return _riseCurrent.empty() && _fallCurrent.empty() &&
                                _riseRecvCaps.empty() && _fallRecvCaps.empty(); }

    const LibData* owner() const { return _owner; }

  private:
    const LibData*        _owner = nullptr;
    std::string           _fromPin;
    std::string           _toPin;
    double                _riseMillerCap;
    double                _fallMillerCap;
    bool                  _isInverted; /// negative_unate -> true
    CCSGroup              _riseCurrent;
    CCSGroup              _fallCurrent;
    std::vector<NLDMLUT>  _riseRecvCaps;
    std::vector<NLDMLUT>  _riseRecvCaps;
    CCBData               _firstStageCCBData;
    CCBData               _lastStageCCBData;
};

class FixedLoadCap {
  public:
    FixedLoadCap() = default;
    void setPinName(const std::string& pin) { _pin = pin; }
    void setCaps(double rise, double fall) { _rise = rise; _fall = fall; }

    std::string pinName() const { return _pin; }
    double value(bool isRise) const 
    {
      return isRise ? _rise : _fall;
    }
  private:
    std::string _pin;
    double      _rise;
    double      _fall;
};

class LibData {
  public:
    LibData() = default;
    LibData(const std::vector<const char*>& datFiles);

    void read(const std::vector<std::string>& datFiles);

    const NLDMArc* findNLDMArc(const char* cell, const char* fromPin, 
                               const char* toPin) const;

    const CCSArc* findCCSArc(const char* cell, const char* fromPin, 
                             const char* toPin) const;

    bool isOutputPin(const char* cell, const char* pin) const;
    
    const NLDMArc* findNLDMArc(const std::string& cell, const std::string& fromPin, 
                               const std::string& toPin) const;

    const CCSArc* findCCSArc(const std::string& cell, const std::string& fromPin, 
                             const std::string& toPin) const;

    bool isOutputPin(const std::string& cell, const std::string& pin) const;
    double fixedLoadCap(const std::string& cell, const std::string& pin, bool isRise) const;

    NLDMLUT riseDriverWaveform() const { return _riseDriverWaveform; }
    NLDMLUT fallDriverWaveform() const { return _fallDriverWaveform; }

    size_t cellCount() const { return _nldmData.size(); }

    std::vector<std::string> cellArcInputPins(const std::string& cell, const std::string& outPin) const;
    std::vector<std::string> cellArcOutputPins(const std::string& cell, const std::string& inPin) const;

    double voltage() const { return _voltage; }
    double riseTransitionLowThres() const { return _transitionRiseLowThres; }
    double riseTransitionHighThres() const { return _transitionRiseHighThres; }
    double fallTransitionLowThres() const { return _transitionFallLowThres; }
    double fallTransitionHighThres() const { return _transitionFallHighThres; }
    double riseDelayThres() const { return _delayRiseThres; }
    double fallDelayThres() const { return _delayFallThres; }
    
  private:
    double    _delayRiseThres = 50;
    double    _delayFallThres = 50;
    double    _transitionRiseLowThres = 10;
    double    _transitionRiseHighThres = 90;
    double    _transitionFallHighThres = 90;
    double    _transitionFallLowThres = 10;
    double    _voltage = 0;
    NLDMLUT   _riseDriverWaveform;
    NLDMLUT   _fallDriverWaveform;
    std::unordered_map<std::string, std::vector<NLDMArc>>      _nldmData;
    std::unordered_map<std::string, std::vector<CCSArc>>       _ccsData;
    std::unordered_map<std::string, std::vector<FixedLoadCap>> _loadCaps;

  friend class LibReader;
};

}

#endif
