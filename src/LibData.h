#ifndef _NA_LIBDAT_H_
#define _NA_LIBDAT_H_

#include <unordered_map>
#include <vector>

namespace NA {

class LibData;

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

    void reset()
    {
      _index1.clear();
      _index2.clear();
      _values.clear();
    }
    void setIndex1(const std::vector<double>& index1) { _index1.assign(index1.begin(), index1.end()); }
    void setIndex2(const std::vector<double>& index2) { _index2.assign(index2.begin(), index2.end()); }
    void setValues(const std::vector<double>& values) { _values.assign(values.begin(), values.end()); }
    
    double value(double inputTran, double outputLoad) const;

    bool empty() const { return _values.empty(); }

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
    void reset() { _ccsluts.clear(); }
    CCSLUT value(double inputTran, double outputLoad) const;
    void sortTable();
    std::vector<size_t> searchSteps() const { return _transDiv; }

    bool empty() const { return _ccsluts.empty(); }
    std::vector<CCSLUT> tables() const { return _ccsluts; }

  private:
    std::vector<CCSLUT> _ccsluts;
    std::vector<size_t> _transDiv;
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
      _riseRecvCap.reset();
      _fallRecvCap.reset();
      _dcCurrent.reset();
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

    NLDMLUT& getRecvCap(LUTType dataType);
    CCSGroup& getCurrent(LUTType dataType);
    NLDMLUT& getDCCurrent();
    const NLDMLUT& getRecvCap(LUTType dataType) const;
    const CCSGroup& getCurrent(LUTType dataType) const;
    const NLDMLUT& getDCCurrent() const; 
    
    const char* fromPin() const { return _fromPin.data(); }
    const char* toPin() const { return _toPin.data(); }
    bool isInverted() const { return _isInverted; }

    bool empty() const { return _riseCurrent.empty() && _fallCurrent.empty() &&
                                _riseRecvCap.empty() && _fallRecvCap.empty() &&
                                _dcCurrent.empty(); }

    const LibData* owner() const { return _owner; }

  private:
    const LibData* _owner = nullptr;
    std::string    _fromPin;
    std::string    _toPin;
    double         _riseMillerCap;
    double         _fallMillerCap;
    bool           _isInverted; /// negative_unate -> true
    CCSGroup       _riseCurrent;
    CCSGroup       _fallCurrent;
    NLDMLUT        _riseRecvCap;
    NLDMLUT        _fallRecvCap;
    NLDMLUT        _dcCurrent;
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