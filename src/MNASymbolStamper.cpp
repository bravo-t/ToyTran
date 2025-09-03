#include <algorithm>
#include "Base.h"
#include "Circuit.h"
#include "Simulator.h"
#include "MNASymbolStamper.h"
#include "Debug.h"

namespace NA {

StringMatrix::StringMatrix(size_t row, size_t col)
{
  for (size_t i=0; i<row; ++i) {
    std::vector<MyString> s(col, MyString());
    _data.push_back(s);
  }
}

std::vector<size_t>
colWidth(const std::vector<std::vector<MyString>>& data) 
{
  std::vector<size_t> width;
  for (size_t i=0; i<data[0].size(); ++i) {
    size_t w = 1;
    for (size_t j=0; j<data.size(); ++j) {
      w = std::max(w, data[j][i].size());
    }
    width.push_back(w);
  }
  return width;
}

void
printPrefixPad(const MyString& s, size_t width)
{
  size_t padSize = width - s.size();
  if (s.empty()) padSize -= 1;
  for (size_t i=0; i<padSize; ++i) {
    printf(" ");
  }
  if (s.empty()) {
    printf("0");
  } else {
    printf("%s", s.data());
  }
}

void
StringMatrix::print() const
{
  const std::vector<size_t>& widths = colWidth(_data);
  size_t totalWidth = 0;
  for (size_t w : widths) totalWidth += w;
  totalWidth += (_data[0].size() - 1);
  MyString line("--");
  line.str() += std::string(totalWidth, ' ');
  line.str() += "--";
  printf("%s\n", line.data());

  for (size_t i=0; i<_data.size(); ++i) {
    printf("| ");
    for (size_t j=0; j<_data[i].size(); ++j) {
      printPrefixPad(_data[i][j], widths[j]);
      printf(" ");
    }
    printf("|\n");
  }
  
  printf("%s\n", line.data());
}

std::string
stampSymbol(const std::string& s, double v) 
{
  return s + "(" + std::to_string(v) + ")";
  //MyString str = "+ " + s + "(" + std::to_string(v) + ")";
  //return str;
}

template <typename T>
std::string
stampSymbolDev(const T& t, double v) 
{
  return stampSymbol(t._name, v);
}

void
MNASymbolStamper::stampResistor(StringMatrix& G, 
                                StringMatrix& /*C*/, 
                                StringMatrix& /*b*/, 
                                const Device& dev, 
                                IntegrateMethod /*intMethod*/) const
{
  double stampValue = 1.0 / dev._value;
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._negNode);
  if (isNodeOmitted(dev._posNode) == false) {
    G(posNodeIndex, posNodeIndex) += stampSymbolDev(dev, stampValue);
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(negNodeIndex, negNodeIndex) += stampSymbolDev(dev, stampValue);
  }
  if (isNodeOmitted(dev._posNode) == false && 
      isNodeOmitted(dev._negNode) == false) {
    G(posNodeIndex, negNodeIndex) -= stampSymbolDev(dev, stampValue);
    G(negNodeIndex, posNodeIndex) -= stampSymbolDev(dev, stampValue);
  }
}

inline void
MNASymbolStamper::updatebCapacitorBE(StringMatrix& b, 
                               const Device& cap) const
{
  double stampValue = cap._value / simTick();
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  double posVoltage = _simResult.nodeVoltageBackstep(cap._posNode, 1);
  double negVoltage = _simResult.nodeVoltageBackstep(cap._negNode, 1);
  double voltageDiff = posVoltage - negVoltage;
  double bValue = stampValue * voltageDiff; 
  if (Debug::enabled(DebugModule::Sim, 9)) {
    printf("DEBUG: T@%G BE %s posNode: %lu, negNode: %lu, bPosRow: %lu, bNegRow: %lu, diff: %G-%G=%G current: %G\n", 
      _simResult.currentTime(), cap._name.data(), cap._posNode, cap._negNode, posNodeIndex, negNodeIndex, posVoltage, 
      negVoltage, voltageDiff, bValue);
  }
  std::string symbol = "dV("+cap._name+")[t]*Cap/Tick";
  if (isNodeOmitted(cap._posNode) == false) {
    b(posNodeIndex, 0) += stampSymbol(symbol, bValue);
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    b(negNodeIndex, 0) += stampSymbol(symbol, -bValue);
  }
}

inline void
MNASymbolStamper::stampCapacitorBE(StringMatrix& /*G*/, 
                             StringMatrix& C, 
                             StringMatrix& b, 
                             const Device& cap) const
{
  double stampValue;
  if (isSDomain()) {
    stampValue = cap._value * _circuit.scalingFactor();
  } else {
    stampValue = cap._value / simTick();
  }
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(cap._posNode) == false) {
    C(posNodeIndex, posNodeIndex) += stampSymbolDev(cap, stampValue);
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    C(negNodeIndex, negNodeIndex) += stampSymbolDev(cap, stampValue);
  }
  if (isNodeOmitted(cap._posNode) == false && isNodeOmitted(cap._negNode) == false) {
    C(posNodeIndex, negNodeIndex) -= stampSymbolDev(cap, stampValue);
    C(negNodeIndex, posNodeIndex) -= stampSymbolDev(cap, stampValue);
  }
  if (isSDomain() == false) {
    updatebCapacitorBE(b, cap);
  }
}

inline void
MNASymbolStamper::updatebCapacitorGear2(StringMatrix& b,
                                  const Device& cap) const
{
  double baseValue =  cap._value / simTick();
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  double posVoltage1 = _simResult.nodeVoltageBackstep(cap._posNode, 1);
  double negVoltage1 = _simResult.nodeVoltageBackstep(cap._negNode, 1);
  double posVoltage2 = _simResult.nodeVoltageBackstep(cap._posNode, 2);
  double negVoltage2 = _simResult.nodeVoltageBackstep(cap._negNode, 2);
  double voltageDiff1 = posVoltage1 - negVoltage1;
  double voltageDiff2 = posVoltage2 - negVoltage2;
  double stampValue = baseValue * (2 * voltageDiff1 - 0.5 * voltageDiff2);
  if (Debug::enabled(DebugModule::Sim, 9)) {
    printf("DEBUG: T@%G BDF %s posNode: %lu, negNode: %lu, bPosRow: %lu, bNegRow: %lu, diff1: %G-%G=%G, diff2: %G-%G=%G, current: %G\n", 
      _simResult.currentTime(), cap._name.data(), cap._posNode, cap._negNode, posNodeIndex, negNodeIndex,
      posVoltage1, negVoltage1, voltageDiff1, 
      posVoltage2, negVoltage2, voltageDiff2, stampValue);
  }
  std::string symbol = "(dV("+cap._name+")[t]*2-dV("+cap._name+")[t-1]*0.5))*Cap/Tick";
  if (isNodeOmitted(cap._posNode) == false) {
    b(posNodeIndex, 0) += stampSymbol(symbol, stampValue);
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    b(negNodeIndex, 0) -= stampSymbol(symbol, stampValue);
  }
}

inline void
MNASymbolStamper::stampCapacitorGear2(StringMatrix& /*G*/, 
                                StringMatrix& C,
                                StringMatrix& b, 
                                const Device& cap) const
{
  double baseValue = 1.5 * cap._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(cap._posNode) == false) {
    C(posNodeIndex, posNodeIndex) += stampSymbolDev(cap, stampValue);
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    C(negNodeIndex, negNodeIndex) += stampSymbolDev(cap, stampValue);
  }
  if (isNodeOmitted(cap._posNode) == false && isNodeOmitted(cap._negNode) == false) {
    C(posNodeIndex, negNodeIndex) -= stampSymbolDev(cap, stampValue);
    C(negNodeIndex, posNodeIndex) -= stampSymbolDev(cap, stampValue);
  }
  updatebCapacitorGear2(b, cap);
}

inline void
MNASymbolStamper::updatebCapacitorTrap(StringMatrix& b,
                                 const Device& cap) const
{
  double baseValue =  cap._value / simTick();
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  double posVoltage1 = _simResult.nodeVoltageBackstep(cap._posNode, 1);
  double negVoltage1 = _simResult.nodeVoltageBackstep(cap._negNode, 1);
  double dV1dt = _simResult.deviceVoltageDerivative(cap, 1, 1);
  double voltageDiff1 = posVoltage1 - negVoltage1;
  double stampValue = 2 * baseValue * voltageDiff1 + cap._value * dV1dt;
  if (Debug::enabled(DebugModule::Sim, 9)) {
    printf("DEBUG: T@%G BDF %s posNode: %lu, negNode: %lu, bPosRow: %lu, bNegRow: %lu, diff1: %G-%G=%G, dV1dt: %G, current: %G\n", 
      _simResult.currentTime(), cap._name.data(), cap._posNode, cap._negNode, posNodeIndex, negNodeIndex,
      posVoltage1, negVoltage1, voltageDiff1, dV1dt, stampValue);
  }
  std::string symbol = "(dV("+cap._name+")[t]*2-ddV("+cap._name+")[t]*Tick))*Cap/Tick";
  if (isNodeOmitted(cap._posNode) == false) {
    b(posNodeIndex, 0) += stampSymbol(symbol, stampValue);
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    b(negNodeIndex, 0) -= stampSymbol(symbol, stampValue);
  }
}

inline void
MNASymbolStamper::stampCapacitorTrap(StringMatrix& /*G*/,
                               StringMatrix& C,
                               StringMatrix& b, 
                               const Device& cap) const
{
  double baseValue = 2 * cap._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(cap._posNode) == false) {
    C(posNodeIndex, posNodeIndex) += stampSymbolDev(cap, stampValue);
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    C(negNodeIndex, negNodeIndex) += stampSymbolDev(cap, stampValue);
  }
  if (isNodeOmitted(cap._posNode) == false && isNodeOmitted(cap._negNode) == false) {
    C(posNodeIndex, negNodeIndex) -= stampSymbolDev(cap, stampValue);
    C(negNodeIndex, posNodeIndex) -= stampSymbolDev(cap, stampValue);
  }
  updatebCapacitorTrap(b, cap);
}

inline void
MNASymbolStamper::stampCapacitor(StringMatrix& G, StringMatrix& C, 
                           StringMatrix& b, const Device& cap,
                           IntegrateMethod intMethod) const
{
  if (isSDomain()) {
    stampCapacitorBE(G, C, b, cap);
    return;
  }
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      stampCapacitorBE(G, C, b, cap);
      break;
    case IntegrateMethod::Gear2:
      stampCapacitorGear2(G, C, b, cap);
      break;
    case IntegrateMethod::Trapezoidal:
      stampCapacitorTrap(G, C, b, cap);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

inline void
MNASymbolStamper::updatebCapacitor(StringMatrix& b, const Device& cap, 
                             IntegrateMethod intMethod) const
{
  if (isSDomain()) {
    return;
  }
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      updatebCapacitorBE(b, cap);
      break;
    case IntegrateMethod::Gear2:
      updatebCapacitorGear2(b, cap);
      break;
    case IntegrateMethod::Trapezoidal:
      updatebCapacitorTrap(b, cap);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

inline void
MNASymbolStamper::updatebInductorBE(StringMatrix& b,
                              const Device& ind) const
{
  double stampValue = ind._value / simTick();
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  double indCurrent = _simResult.deviceCurrentBackstep(ind._devId, 1);
  double bValue = -stampValue * indCurrent;
  std::string symbol = "-I("+ind._name+")[t]*Ind/Tick";
  b(deviceIndex, 0) += stampSymbol(symbol, bValue);
}

inline void
MNASymbolStamper::stampInductorBE(StringMatrix& G, 
                            StringMatrix& C, 
                            StringMatrix& b, 
                            const Device& ind) const
{
  double stampValue;
  if (isSDomain()) {
    stampValue = ind._value * _circuit.scalingFactor();
  } else {
    stampValue = ind._value / simTick();
  }
  size_t posNodeIndex = _simResult.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(ind._posNode) == false) {
    G(posNodeIndex, deviceIndex) += stampSymbolDev(ind, 1);
    G(deviceIndex, posNodeIndex) += stampSymbolDev(ind, 1);
  }
  if (isNodeOmitted(ind._negNode) == false) {
    G(negNodeIndex, deviceIndex) -= stampSymbolDev(ind, 1);
    G(deviceIndex, negNodeIndex) -= stampSymbolDev(ind, 1);
  }
  C(deviceIndex, deviceIndex) -= stampSymbolDev(ind, stampValue);
  if (isSDomain() == false) {
    updatebInductorBE(b, ind);
  }
}

inline void
MNASymbolStamper::updatebInductorGear2(StringMatrix& b,
                                 const Device& ind) const
{
  double baseValue = ind._value / simTick();
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  double indCurrent1 = _simResult.deviceCurrentBackstep(ind._devId, 1);
  double indCurrent2 = _simResult.deviceCurrentBackstep(ind._devId, 2);
  double stampValue = -baseValue * (2 * indCurrent1 - 0.5 * indCurrent2);
  std::string symbol = "-(I("+ind._name+")[t]*2-I("+ind._name+")[t-1]*0.5))*Ind/Tick";
  b(deviceIndex, 0) += stampSymbol(symbol, stampValue);
}

inline void
MNASymbolStamper::stampInductorGear2(StringMatrix& /*G*/,
                               StringMatrix& C, 
                               StringMatrix& b, 
                               const Device& ind) const
{
  double baseValue = 1.5 * ind._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(ind._posNode) == false) {
    C(posNodeIndex, deviceIndex) += stampSymbolDev(ind, 1);
    C(deviceIndex, posNodeIndex) += stampSymbolDev(ind, 1);
  }
  if (isNodeOmitted(ind._negNode) == false) {
    C(negNodeIndex, deviceIndex) -= stampSymbolDev(ind, 1);
    C(deviceIndex, negNodeIndex) -= stampSymbolDev(ind, 1);
  }
  C(deviceIndex, deviceIndex) -= stampSymbolDev(ind, stampValue);
  updatebInductorGear2(b, ind);
}

inline void
MNASymbolStamper::updatebInductorTrap(StringMatrix& b,
                                const Device& ind) const
{
  double baseValue = ind._value / simTick();
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  double indCurrent1 = _simResult.deviceCurrentBackstep(ind._devId, 1);
  double dI1dt = _simResult.deviceCurrentDerivative(ind, 1, 1);
  double stampValue = -2 * baseValue * indCurrent1 - ind._value * dI1dt;
  std::string symbol = "-(I("+ind._name+")[t]*2-dI("+ind._name+")[t]*Tick))*Ind/Tick";
  b(deviceIndex, 0) += stampSymbol(symbol, stampValue);
}

inline void
MNASymbolStamper::stampInductorTrap(StringMatrix& /*G*/,
                              StringMatrix& C,
                              StringMatrix& b, 
                              const Device& ind) const
{
  double baseValue = 2 * ind._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(ind._posNode) == false) {
    C(posNodeIndex, deviceIndex) += stampSymbolDev(ind, 1);
    C(deviceIndex, posNodeIndex) += stampSymbolDev(ind, 1);
  }
  if (isNodeOmitted(ind._negNode) == false) {
    C(negNodeIndex, deviceIndex) -= stampSymbolDev(ind, 1);
    C(deviceIndex, negNodeIndex) -= stampSymbolDev(ind, 1);
  }
  C(deviceIndex, deviceIndex) -= stampSymbolDev(ind, stampValue);
  updatebInductorTrap(b, ind);
}

inline void
MNASymbolStamper::stampInductor(StringMatrix& G, StringMatrix& C,
                          StringMatrix& b, const Device& ind, 
                          IntegrateMethod intMethod) const
{
  if (isSDomain()) {
    stampInductorBE(G, C, b, ind);
    return;
  }
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      stampInductorBE(G, C, b, ind);
      break;
    case IntegrateMethod::Gear2:
      stampInductorGear2(G, C, b, ind);
      break;
    case IntegrateMethod::Trapezoidal:
      stampInductorTrap(G, C, b, ind);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

inline void
MNASymbolStamper::updatebInductor(StringMatrix& b, const Device& ind, 
                            IntegrateMethod intMethod) const
{
  if (isSDomain()) {
    return;
  }
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      updatebInductorBE(b, ind);
      break;
    case IntegrateMethod::Gear2:
      updatebInductorGear2(b, ind);
      break;
    case IntegrateMethod::Trapezoidal:
      updatebInductorTrap(b, ind);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

inline void
MNASymbolStamper::updatebVoltageSource(StringMatrix& b,
                                 const Device& dev,
                                 IntegrateMethod /*intMethod*/) const
{
  double value;
  if (isSDomain()) {
    value = 1 * _circuit.scalingFactor();
  } else {
    if (dev._isPWLValue) {
      const PWLValue& pwlData = _circuit.PWLData(dev);
      value = pwlData.valueAtTime(_simResult.currentTime());
    } else {
      value = dev._value;
    }
  }
  size_t deviceIndex = _simResult.deviceVectorIndex(dev._devId);
  b(deviceIndex, 0) += stampSymbolDev(dev, value);
}

inline void
MNASymbolStamper::stampVoltageSource(StringMatrix& G, 
                               StringMatrix& /*C*/,
                               StringMatrix& b, 
                               const Device& dev,
                               IntegrateMethod /*intMethod*/) const
{
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(dev._devId);
  if (isNodeOmitted(dev._posNode) == false) {
    G(posNodeIndex, deviceIndex) += stampSymbolDev(dev, 1);
    G(deviceIndex, posNodeIndex) += stampSymbolDev(dev, 1);
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(negNodeIndex, deviceIndex) -= stampSymbolDev(dev, 1);
    G(deviceIndex, negNodeIndex) -= stampSymbolDev(dev, 1);
  }
  updatebVoltageSource(b, dev);
}

inline void
MNASymbolStamper::updatebCurrentSource(StringMatrix& b,
                                 const Device& dev,
                                 IntegrateMethod /*intMethod*/) const
{
  double value;
  if (dev._isPWLValue) {
    const PWLValue& pwlData = _circuit.PWLData(dev);
    value = pwlData.valueAtTime(_simResult.currentTime());
  } else {
    value = dev._value;
  }
  if (isSDomain()) {
    value = 1 * _circuit.scalingFactor();
  }
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._negNode);
  if (isNodeOmitted(dev._posNode) == false) {
    b(posNodeIndex, 0) -= stampSymbolDev(dev, value);
  }
  if (isNodeOmitted(dev._negNode) == false) {
    b(negNodeIndex, 0) += stampSymbolDev(dev, value);
  }
}

inline void
MNASymbolStamper::stampCurrentSource(StringMatrix& /*G*/, 
                               StringMatrix& /*C*/, 
                               StringMatrix& b, 
                               const Device& dev,
                               IntegrateMethod /*intMethod*/) const
{
  updatebCurrentSource(b, dev);
}

inline void
MNASymbolStamper::stampCCVS(StringMatrix& G, 
                      StringMatrix& /*C*/, 
                      StringMatrix& /*b*/, 
                      const Device& dev, 
                      IntegrateMethod /*intMethod*/) const
{
  assert(isSDomain() == false);
  const Device& sampleDevice = _circuit.device(dev._sampleDevice);
  double value = dev._value;
  if (sampleDevice._posNode == dev._negSampleNode) {
    value = -value;
  }
  size_t deviceIndex = _simResult.deviceVectorIndex(dev._devId);
  size_t sampleDeviceIndex = _simResult.deviceVectorIndex(dev._sampleDevice);
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = _simResult.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = _simResult.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(dev._posNode) == false) {
    G(sampleDeviceIndex, posSampleNodeIndex) += stampSymbolDev(dev, 1);
    G(posSampleNodeIndex, sampleDeviceIndex) += stampSymbolDev(dev, 1);
    G(deviceIndex, posNodeIndex) += stampSymbolDev(dev, 1);
    G(posNodeIndex, deviceIndex) += stampSymbolDev(dev, 1);
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(negSampleNodeIndex, sampleDeviceIndex) -= stampSymbolDev(dev, 1);
    G(sampleDeviceIndex, negSampleNodeIndex) -= stampSymbolDev(dev, 1);
    G(deviceIndex, negNodeIndex) -= stampSymbolDev(dev, 1);
    G(negNodeIndex, deviceIndex) -= stampSymbolDev(dev, 1);
  }
  G(deviceIndex, sampleDeviceIndex) += stampSymbolDev(dev, value);
}

inline void
MNASymbolStamper::stampVCVS(StringMatrix& G, 
                      StringMatrix& /*C*/, 
                      StringMatrix& /*b*/, 
                      const Device& dev,
                      IntegrateMethod /*intMethod*/) const
{
  assert(isSDomain() == false);
  double value = dev._value;
  size_t deviceIndex = _simResult.deviceVectorIndex(dev._devId);
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = _simResult.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = _simResult.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(dev._posNode) == false) {
    G(deviceIndex, posSampleNodeIndex) -= stampSymbolDev(dev, value);
    G(deviceIndex, posNodeIndex) += stampSymbolDev(dev, 1);
    G(posNodeIndex, deviceIndex) += stampSymbolDev(dev, 1);
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(deviceIndex, negSampleNodeIndex) += stampSymbolDev(dev, value);
    G(deviceIndex, negNodeIndex) += stampSymbolDev(dev, 1);
    G(negNodeIndex, deviceIndex) += stampSymbolDev(dev, 1);
  }
}

inline void
MNASymbolStamper::stampCCCS(StringMatrix& G, 
                      StringMatrix& /*C*/,
                      StringMatrix& /*b*/, 
                      const Device& dev,
                      IntegrateMethod /*intMethod*/) const
{
  assert(isSDomain() == false);
  const Device& sampleDevice = _circuit.device(dev._sampleDevice);
  double value = dev._value;
  if (sampleDevice._posNode == dev._negSampleNode) {
    value = -value;
  }
  size_t sampleDeviceIndex = _simResult.deviceVectorIndex(dev._sampleDevice);
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = _simResult.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = _simResult.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(dev._posNode) == false) {
    G(sampleDeviceIndex, posSampleNodeIndex) += stampSymbolDev(dev, 1);
    G(posSampleNodeIndex, sampleDeviceIndex) += stampSymbolDev(dev, 1);
    G(posNodeIndex, sampleDeviceIndex) += stampSymbolDev(dev, value);
  }
  if (isNodeOmitted(dev._posNode) == false) {
    G(negSampleNodeIndex, sampleDeviceIndex) -= stampSymbolDev(dev, 1);
    G(sampleDeviceIndex, negSampleNodeIndex) -= stampSymbolDev(dev, 1);
    G(negNodeIndex, sampleDeviceIndex) -= stampSymbolDev(dev, value);
  }
}

inline void
MNASymbolStamper::stampVCCS(StringMatrix& G, 
                      StringMatrix& /*C*/, 
                      StringMatrix& /*b*/, 
                      const Device& dev,
                      IntegrateMethod /*intMethod*/) const
{
  assert(isSDomain() == false);
  double value = dev._value;
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = _simResult.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = _simResult.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(dev._posNode) == false && 
      isNodeOmitted(dev._posSampleNode) == false) {
    G(posNodeIndex, posSampleNodeIndex) += stampSymbolDev(dev, value);
   }
  if (isNodeOmitted(dev._posNode) == false && 
      isNodeOmitted(dev._negSampleNode) == false) {
    G(posNodeIndex, negSampleNodeIndex) -= stampSymbolDev(dev, value);
  }
  if (isNodeOmitted(dev._negNode) == false && 
      isNodeOmitted(dev._posSampleNode) == false) {
    G(negNodeIndex, posSampleNodeIndex) -= stampSymbolDev(dev, value);
  }
  if (isNodeOmitted(dev._negNode) == false && 
      isNodeOmitted(dev._negSampleNode) == false) {
    G(negNodeIndex, negSampleNodeIndex) += stampSymbolDev(dev, value);
  }
}

void
MNASymbolStamper::stamp(StringMatrix& G, 
                  StringMatrix& C,
                  StringMatrix& b, 
                  IntegrateMethod intMethod)
{
  static void (MNASymbolStamper::*stampFunc[static_cast<size_t>(DeviceType::Total)])(
      StringMatrix& G, StringMatrix& C, StringMatrix& b, 
      const Device& dev, IntegrateMethod intMethod) const;

  stampFunc[static_cast<size_t>(DeviceType::Resistor)] = &NA::MNASymbolStamper::stampResistor;
  stampFunc[static_cast<size_t>(DeviceType::Capacitor)] = &NA::MNASymbolStamper::stampCapacitor;
  stampFunc[static_cast<size_t>(DeviceType::Inductor)] = &NA::MNASymbolStamper::stampInductor;
  stampFunc[static_cast<size_t>(DeviceType::VoltageSource)] = &NA::MNASymbolStamper::stampVoltageSource;
  stampFunc[static_cast<size_t>(DeviceType::CurrentSource)] = &NA::MNASymbolStamper::stampCurrentSource;
  stampFunc[static_cast<size_t>(DeviceType::VCVS)] = &NA::MNASymbolStamper::stampVCVS;
  stampFunc[static_cast<size_t>(DeviceType::VCCS)] = &NA::MNASymbolStamper::stampVCCS;
  stampFunc[static_cast<size_t>(DeviceType::CCVS)] = &NA::MNASymbolStamper::stampCCVS;
  stampFunc[static_cast<size_t>(DeviceType::CCCS)] = &NA::MNASymbolStamper::stampCCCS;

  const std::vector<Device>& devices = _circuit.devicesToSimulate();
  for (const Device& device : devices) {
    (this->*stampFunc[static_cast<size_t>(device._type)])(G, C, b, device, intMethod);
  }
}

void
MNASymbolStamper::updatebNoop(StringMatrix& /*b*/, 
                        const Device& /*dev*/, 
                        IntegrateMethod /*intMethod*/) const
{
  return;
}

}
