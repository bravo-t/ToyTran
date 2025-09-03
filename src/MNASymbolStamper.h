#ifndef _TRAN_MNASYMSTM_H_
#define _TRAN_MNASYMSTM_H_

#include <string>
#include "Base.h"
#include "Circuit.h"

namespace NA {

class AnalysisParameter;
class Circuit;
class SimResult;

class MyString {
  public:
    MyString(const std::string& str) : _str(str) {}
    MyString(size_t n, char c) : _str(n, c) {}
    MyString() = default;
    std::string& str() { return _str; }
    size_t length() const { return _str.length(); }
    size_t size() const { return _str.size(); }
    bool empty() const { return _str.empty(); }
    const char* data() const { return _str.data(); }
    MyString operator=(const char* str) const { return MyString(str); }
    MyString& operator=(const char* str) { _str = str; return *this; }
    MyString& operator=(const std::string& str) { _str = str; return *this; }
    MyString& operator=(const MyString& rhs) { _str = rhs._str; return *this; }
    MyString operator+(const char* str) const { return MyString(_str+str); }
    MyString operator+(const std::string& str) const { return MyString(_str+str); }
    MyString operator+(const MyString& rhs) const { return MyString(_str+rhs._str); }
    MyString& operator+=(const char* rhs) 
    { 
      if (_str.empty()) {
        _str = rhs;
        return *this;
      } 
      _str += std::string(" + ") + rhs;
      return *this;
    }
    MyString& operator+=(const std::string& rhs) 
    {
      if (_str.empty()) {
        _str = rhs;
        return *this;
      } 
      _str += std::string(" + ") + rhs;
      return *this;
    }
    MyString& operator+=(const MyString& rhs) 
    { 
      if (_str.empty()) {
        _str = rhs._str;
        return *this;
      } 
      _str += std::string(" + ") + rhs._str;
      return *this;
    }
    MyString& operator-=(const MyString& rhs) 
    { 
      if (_str.empty()) {
        _str = rhs._str;
        return *this;
      } 
      _str += std::string(" - ") + rhs._str;
      return *this;
    }
    MyString& operator-=(const std::string& rhs) 
    { 
      if (_str.empty()) {
        _str = rhs;
        return *this;
      } 
      _str += std::string(" - ") + rhs;
      return *this;
    }

  private:
    std::string _str;
};

class StringMatrix {
  public:
    StringMatrix(size_t row, size_t col);
    MyString& operator()(size_t row, size_t col)
    {
      return _data[row][col];
    } 
    void set(size_t row, size_t col, const MyString& val)
    {
      _data[row][col] = val;
    }
    void print() const;
    size_t row() const { return _data.size(); }
    size_t col() const 
    { 
      if (_data.empty()) return 0;
      return _data[0].size(); 
    }

  private:

  private:
    std::vector<std::vector<MyString>> _data;
};

class MNASymbolStamper {
  public:
    MNASymbolStamper(const AnalysisParameter& param, const Circuit& ckt, const SimResult& simResult)
    : _analysisParam(param), _circuit(ckt), _simResult(simResult) {}
    void stamp(StringMatrix& G, StringMatrix& C, StringMatrix& b, 
               IntegrateMethod intMethod = IntegrateMethod::Gear2);

  private:
    inline double simTick() const { return _analysisParam._simTick; }
    inline bool isSDomain() const 
    { 
      return _analysisParam._type == AnalysisType::PZ || 
             _analysisParam._type == AnalysisType::TF; 
    }
    inline bool isNodeOmitted(size_t nodeId) const 
    {
      return _circuit.isGroundNode(nodeId);
    }
    /// Stamp functions for G and C
    void stampCCVS(StringMatrix& G, StringMatrix& /*C*/, 
                   StringMatrix& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampVCVS(StringMatrix& G, StringMatrix& /*C*/, 
                   StringMatrix& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampCCCS(StringMatrix& G, StringMatrix& /*C*/, 
                   StringMatrix& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampVCCS(StringMatrix& G, StringMatrix& /*C*/, 
                   StringMatrix& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampVoltageSource(StringMatrix& G, StringMatrix& /*C*/,
                            StringMatrix& b, const Device& dev,
                            IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampCurrentSource(StringMatrix& /*G*/, StringMatrix& /*C*/, 
                            StringMatrix& b, const Device& dev, 
                            IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampCapacitor(StringMatrix& G, StringMatrix& C, StringMatrix& b, const Device& cap, 
                        IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampResistor(StringMatrix& G, StringMatrix& C, StringMatrix& b, const Device& dev,
                       IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampInductor(StringMatrix& G, StringMatrix& C, StringMatrix& b, const Device& ind, 
                       IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    /// update functions for b
    void updatebCapacitor(StringMatrix& b, const Device& cap, 
                          IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebInductor(StringMatrix& b, const Device& ind, 
                         IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebVoltageSource(StringMatrix& b, const Device& dev,
                              IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebCurrentSource(StringMatrix& b, const Device& dev,
                              IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebNoop(StringMatrix& b, const Device& dev,
                     IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    
    /// stamp and update functions for specific integration methods
    void updatebCapacitorBE(StringMatrix& b, const Device& cap) const;
    void stampCapacitorBE(StringMatrix& G, StringMatrix& C, StringMatrix& b, const Device& cap) const;
    void updatebCapacitorGear2(StringMatrix& b, const Device& cap) const;
    void stampCapacitorGear2(StringMatrix& /*G*/, StringMatrix& C, StringMatrix& b, const Device& cap) const;
    void updatebCapacitorTrap(StringMatrix& b, const Device& cap) const;
    void stampCapacitorTrap(StringMatrix& /*G*/, StringMatrix& C, StringMatrix& b, const Device& cap) const;
    void updatebInductorBE(StringMatrix& b, const Device& ind) const;
    void stampInductorBE(StringMatrix& /*G*/, StringMatrix& C, StringMatrix& b, const Device& ind) const;
    void updatebInductorGear2(StringMatrix& b, const Device& ind) const;
    void stampInductorGear2(StringMatrix& /*G*/, StringMatrix& C, StringMatrix& b, const Device& ind) const;
    void updatebInductorTrap(StringMatrix& b, const Device& ind) const;
    void stampInductorTrap(StringMatrix& /*G*/, StringMatrix& C, StringMatrix& b, const Device& ind) const;

  private:
    AnalysisParameter _analysisParam;
    const Circuit& _circuit;
    const SimResult& _simResult;
};

}

#endif