#include <algorithm>
#include "Base.h"
#include "Circuit.h"
#include "Simulator.h"
#include "MNAStamper.h"

namespace NA {

void
MNAStamper::stampResistor(Eigen::MatrixXd& G, 
                          Eigen::MatrixXd& /*C*/, 
                          Eigen::VectorXd& /*b*/, 
                          const Device& dev, 
                          IntegrateMethod /*intMethod*/) const
{
  double stampValue = 1.0 / dev._value;
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._negNode);
  if (isNodeOmitted(dev._posNode) == false) {
    G(posNodeIndex, posNodeIndex) += stampValue;
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(dev._posNode) == false && 
      isNodeOmitted(dev._negNode) == false) {
    G(posNodeIndex, negNodeIndex) += -stampValue;
    G(negNodeIndex, posNodeIndex) += -stampValue;
  }
}

inline void
MNAStamper::updatebCapacitorBE(Eigen::VectorXd& b, 
                               const Device& cap) const
{
  double stampValue = cap._value / simTick();
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  double posVoltage = _simResult.nodeVoltageBackstep(cap._posNode, 1);
  double negVoltage = _simResult.nodeVoltageBackstep(cap._negNode, 1);
  double voltageDiff = posVoltage - negVoltage;
  double bValue = stampValue * voltageDiff; 
  //printf("DEBUG: T@%G BE posNode: %lu, negNode: %lu, diff: %G-%G=%G current: %G\n", 
  //  sim->simulationResult().currentTime(), cap._posNode, cap._negNode, posVoltage, 
  //  negVoltage, voltageDiff, bValue);
  if (isNodeOmitted(cap._posNode) == false) {
    b(posNodeIndex) += bValue;
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    b(negNodeIndex) += -bValue;
  }
}

inline void
MNAStamper::stampCapacitorBE(Eigen::MatrixXd& /*G*/, 
                             Eigen::MatrixXd& C, 
                             Eigen::VectorXd& b, 
                             const Device& cap) const
{
  double stampValue;
  if (isSDomain()) {
    stampValue = cap._value;
  }
  stampValue = cap._value / simTick();
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(cap._posNode) == false) {
    C(posNodeIndex, posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    C(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(cap._posNode) == false && isNodeOmitted(cap._negNode) == false) {
    C(posNodeIndex, negNodeIndex) -= stampValue;
    C(negNodeIndex, posNodeIndex) -= stampValue;
  }
  if (isSDomain() == false) {
    updatebCapacitorBE(b, cap);
  }
}

inline void
MNAStamper::updatebCapacitorGear2(Eigen::VectorXd& b,
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
  //printf("DEBUG: T@%G BDF posNode: %lu, negNode: %lu, diff1: %G-%G=%G, diff2: %G-%G=%G\n", 
  //  sim->simulationResult().currentTime(), cap._posNode, cap._negNode, 
  //    posVoltage1, negVoltage1, voltageDiff1, 
  //    posVoltage2, negVoltage2, voltageDiff2);
  if (isNodeOmitted(cap._posNode) == false) {
    b(posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    b(negNodeIndex) += -stampValue;
  }
}

inline void
MNAStamper::stampCapacitorGear2(Eigen::MatrixXd& /*G*/, 
                                Eigen::MatrixXd& C,
                                Eigen::VectorXd& b, 
                                const Device& cap) const
{
  double baseValue = 1.5 * cap._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(cap._posNode) == false) {
    C(posNodeIndex, posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    C(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(cap._posNode) == false && isNodeOmitted(cap._negNode) == false) {
    C(posNodeIndex, negNodeIndex) -= stampValue;
    C(negNodeIndex, posNodeIndex) -= stampValue;
  }
  updatebCapacitorGear2(b, cap);
}

inline void
MNAStamper::updatebCapacitorTrap(Eigen::VectorXd& b,
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
  if (isNodeOmitted(cap._posNode) == false) {
    b(posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    b(negNodeIndex) += -stampValue;
  }
}

inline void
MNAStamper::stampCapacitorTrap(Eigen::MatrixXd& /*G*/,
                               Eigen::MatrixXd& C,
                               Eigen::VectorXd& b, 
                               const Device& cap) const
{
  double baseValue = 2 * cap._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(cap._posNode) == false) {
    C(posNodeIndex, posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(cap._negNode) == false) {
    C(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(cap._posNode) == false && isNodeOmitted(cap._negNode) == false) {
    C(posNodeIndex, negNodeIndex) -= stampValue;
    C(negNodeIndex, posNodeIndex) -= stampValue;
  }
  updatebCapacitorTrap(b, cap);
}

inline void
MNAStamper::stampCapacitor(Eigen::MatrixXd& G, Eigen::MatrixXd& C, 
                           Eigen::VectorXd& b, const Device& cap,
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
MNAStamper::updatebCapacitor(Eigen::VectorXd& b, const Device& cap, 
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
MNAStamper::updatebInductorBE(Eigen::VectorXd& b,
                              const Device& ind) const
{
  double stampValue = ind._value / simTick();
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  double indCurrent = _simResult.deviceCurrentBackstep(ind._devId, 1);
  double bValue = -stampValue * indCurrent;
  b(deviceIndex) += bValue;
}

inline void
MNAStamper::stampInductorBE(Eigen::MatrixXd& /*G*/, 
                            Eigen::MatrixXd& C, 
                            Eigen::VectorXd& b, 
                            const Device& ind) const
{
  double stampValue = ind._value / simTick();
  if (isSDomain()) {
    stampValue = ind._value;
  }
  size_t posNodeIndex = _simResult.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(ind._posNode) == false) {
    C(posNodeIndex, deviceIndex) += 1;
    C(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(ind._negNode) == false) {
    C(negNodeIndex, deviceIndex) += -1;
    C(deviceIndex, negNodeIndex) += -1;
  }
  C(deviceIndex, deviceIndex) += -stampValue;
  if (isSDomain() == false) {
    updatebInductorBE(b, ind);
  }
}

inline void
MNAStamper::updatebInductorGear2(Eigen::VectorXd& b,
                                 const Device& ind) const
{
  double baseValue = ind._value / simTick();
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  double indCurrent1 = _simResult.deviceCurrentBackstep(ind._devId, 1);
  double indCurrent2 = _simResult.deviceCurrentBackstep(ind._devId, 2);
  double stampValue = -baseValue * (2 * indCurrent1 - 0.5 * indCurrent2);
  b(deviceIndex) += stampValue;
}

inline void
MNAStamper::stampInductorGear2(Eigen::MatrixXd& /*G*/,
                               Eigen::MatrixXd& C, 
                               Eigen::VectorXd& b, 
                               const Device& ind) const
{
  double baseValue = 1.5 * ind._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(ind._posNode) == false) {
    C(posNodeIndex, deviceIndex) += 1;
    C(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(ind._negNode) == false) {
    C(negNodeIndex, deviceIndex) += -1;
    C(deviceIndex, negNodeIndex) += -1;
  }
  C(deviceIndex, deviceIndex) += -stampValue;
  updatebInductorGear2(b, ind);
}

inline void
MNAStamper::updatebInductorTrap(Eigen::VectorXd& b,
                                const Device& ind) const
{
  double baseValue = ind._value / simTick();
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  double indCurrent1 = _simResult.deviceCurrentBackstep(ind._devId, 1);
  double dI1dt = _simResult.deviceCurrentDerivative(ind, 1, 1);
  double stampValue = -2 * baseValue * indCurrent1 - ind._value * dI1dt;
  b(deviceIndex) += stampValue;
}

inline void
MNAStamper::stampInductorTrap(Eigen::MatrixXd& /*G*/,
                              Eigen::MatrixXd& C,
                              Eigen::VectorXd& b, 
                              const Device& ind) const
{
  double baseValue = 2 * ind._value / simTick();
  double stampValue = baseValue;
  size_t posNodeIndex = _simResult.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(ind._posNode) == false) {
    C(posNodeIndex, deviceIndex) += 1;
    C(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(ind._negNode) == false) {
    C(negNodeIndex, deviceIndex) += -1;
    C(deviceIndex, negNodeIndex) += -1;
  }
  C(deviceIndex, deviceIndex) += -stampValue;
  updatebInductorTrap(b, ind);
}

inline void
MNAStamper::stampInductor(Eigen::MatrixXd& G, Eigen::MatrixXd& C,
                          Eigen::VectorXd& b, const Device& ind, 
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
MNAStamper::updatebInductor(Eigen::VectorXd& b, const Device& ind, 
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
MNAStamper::updatebVoltageSource(Eigen::VectorXd& b,
                                 const Device& dev,
                                 IntegrateMethod /*intMethod*/) const
{
  double value;
  if (isSDomain()) {
    value = 1;
  } else {
    if (dev._isPWLValue) {
      const PWLValue& pwlData = _circuit.PWLData(dev);
      value = pwlData.valueAtTime(_simResult.currentTime());
    } else {
      value = dev._value;
    }
  }
  size_t deviceIndex = _simResult.deviceVectorIndex(dev._devId);
  b(deviceIndex) += value;
}

inline void
MNAStamper::stampVoltageSource(Eigen::MatrixXd& G, 
                               Eigen::MatrixXd& /*C*/,
                               Eigen::VectorXd& b, 
                               const Device& dev,
                               IntegrateMethod /*intMethod*/) const
{
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._negNode);
  size_t deviceIndex = _simResult.deviceVectorIndex(dev._devId);
  if (isNodeOmitted(dev._posNode) == false) {
    G(posNodeIndex, deviceIndex) += 1;
    G(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(negNodeIndex, deviceIndex) += -1;
    G(deviceIndex, negNodeIndex) += -1;
  }
  updatebVoltageSource(b, dev);
}

inline void
MNAStamper::updatebCurrentSource(Eigen::VectorXd& b,
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
    value = 1;
  }
  size_t posNodeIndex = _simResult.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = _simResult.nodeVectorIndex(dev._negNode);
  if (isNodeOmitted(dev._posNode) == false) {
    b(posNodeIndex) = -value;
  }
  if (isNodeOmitted(dev._negNode) == false) {
    b(negNodeIndex) = value;
  }
}

inline void
MNAStamper::stampCurrentSource(Eigen::MatrixXd& /*G*/, 
                               Eigen::MatrixXd& /*C*/, 
                               Eigen::VectorXd& b, 
                               const Device& dev,
                               IntegrateMethod /*intMethod*/) const
{
  updatebCurrentSource(b, dev);
}

inline void
MNAStamper::stampCCVS(Eigen::MatrixXd& G, 
                      Eigen::MatrixXd& /*C*/, 
                      Eigen::VectorXd& /*b*/, 
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
    G(sampleDeviceIndex, posSampleNodeIndex) += 1;
    G(posSampleNodeIndex, sampleDeviceIndex) += 1;
    G(deviceIndex, posNodeIndex) += 1;
    G(posNodeIndex, deviceIndex) += 1;
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(negSampleNodeIndex, sampleDeviceIndex) += -1;
    G(sampleDeviceIndex, negSampleNodeIndex) += -1;
    G(deviceIndex, negNodeIndex) += -1;
    G(negNodeIndex, deviceIndex) += -1;
  }
  G(deviceIndex, sampleDeviceIndex) += value;
}

inline void
MNAStamper::stampVCVS(Eigen::MatrixXd& G, 
                      Eigen::MatrixXd& /*C*/, 
                      Eigen::VectorXd& /*b*/, 
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
    G(deviceIndex, posSampleNodeIndex) += -value;
    G(deviceIndex, posNodeIndex) += 1;
    G(posNodeIndex, deviceIndex) += 1;
  }
  if (isNodeOmitted(dev._negNode) == false) {
    G(deviceIndex, negSampleNodeIndex) += value;
    G(deviceIndex, negNodeIndex) += 1;
    G(negNodeIndex, deviceIndex) += 1;
  }
}

inline void
MNAStamper::stampCCCS(Eigen::MatrixXd& G, 
                      Eigen::MatrixXd& /*C*/,
                      Eigen::VectorXd& /*b*/, 
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
    G(sampleDeviceIndex, posSampleNodeIndex) += 1;
    G(posSampleNodeIndex, sampleDeviceIndex) += 1;
    G(posNodeIndex, sampleDeviceIndex) += value;
  }
  if (isNodeOmitted(dev._posNode) == false) {
    G(negSampleNodeIndex, sampleDeviceIndex) += -1;
    G(sampleDeviceIndex, negSampleNodeIndex) += -1;
    G(negNodeIndex, sampleDeviceIndex) += -value;
  }
}

inline void
MNAStamper::stampVCCS(Eigen::MatrixXd& G, 
                      Eigen::MatrixXd& /*C*/, 
                      Eigen::VectorXd& /*b*/, 
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
    G(posNodeIndex, posSampleNodeIndex) += value;
   }
  if (isNodeOmitted(dev._posNode) == false && 
      isNodeOmitted(dev._negSampleNode) == false) {
    G(posNodeIndex, negSampleNodeIndex) += -value;
  }
  if (isNodeOmitted(dev._negNode) == false && 
      isNodeOmitted(dev._posSampleNode) == false) {
    G(negNodeIndex, posSampleNodeIndex) += -value;
  }
  if (isNodeOmitted(dev._negNode) == false && 
      isNodeOmitted(dev._negSampleNode) == false) {
    G(negNodeIndex, negSampleNodeIndex) += value;
  }
}

void
MNAStamper::stamp(Eigen::MatrixXd& G, 
                  Eigen::MatrixXd& C,
                  Eigen::VectorXd& b, 
                  IntegrateMethod intMethod)
{
  void (MNAStamper::*stampFunc[static_cast<size_t>(DeviceType::Total)])(
      Eigen::MatrixXd& G, Eigen::MatrixXd& C, Eigen::VectorXd& b, 
      const Device& dev, IntegrateMethod intMethod) const;

  stampFunc[static_cast<size_t>(DeviceType::Resistor)] = &NA::MNAStamper::stampResistor;
  stampFunc[static_cast<size_t>(DeviceType::Capacitor)] = &NA::MNAStamper::stampCapacitor;
  stampFunc[static_cast<size_t>(DeviceType::Inductor)] = &NA::MNAStamper::stampInductor;
  stampFunc[static_cast<size_t>(DeviceType::VoltageSource)] = &NA::MNAStamper::stampVoltageSource;
  stampFunc[static_cast<size_t>(DeviceType::CurrentSource)] = &NA::MNAStamper::stampCurrentSource;
  stampFunc[static_cast<size_t>(DeviceType::VCVS)] = &NA::MNAStamper::stampVCVS;
  stampFunc[static_cast<size_t>(DeviceType::VCCS)] = &NA::MNAStamper::stampVCCS;
  stampFunc[static_cast<size_t>(DeviceType::CCVS)] = &NA::MNAStamper::stampCCVS;
  stampFunc[static_cast<size_t>(DeviceType::CCCS)] = &NA::MNAStamper::stampCCCS;

  const std::vector<Device>& devices = _circuit.devices();
  for (const Device& device : devices) {
    (this->*stampFunc[static_cast<size_t>(device._type)])(G, C, b, device, intMethod);
  }
}

void
MNAStamper::updatebNoop(Eigen::VectorXd& /*b*/, 
                        const Device& /*dev*/, 
                        IntegrateMethod /*intMethod*/) const
{
  return;
}

void 
MNAStamper::updateb(Eigen::VectorXd& b, IntegrateMethod intMethod)
{
  void (MNAStamper::*updatebFunc[static_cast<size_t>(DeviceType::Total)])(
      Eigen::VectorXd& b, const Device& dev, IntegrateMethod intMethod) const;

  updatebFunc[static_cast<size_t>(DeviceType::Resistor)] = &NA::MNAStamper::updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::Capacitor)] = &NA::MNAStamper::updatebCapacitor;
  updatebFunc[static_cast<size_t>(DeviceType::Inductor)] = &NA::MNAStamper::updatebInductor;
  updatebFunc[static_cast<size_t>(DeviceType::VoltageSource)] = &NA::MNAStamper::updatebVoltageSource;
  updatebFunc[static_cast<size_t>(DeviceType::CurrentSource)] = &NA::MNAStamper::updatebCurrentSource;
  updatebFunc[static_cast<size_t>(DeviceType::VCVS)] = &NA::MNAStamper::updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::VCCS)] = &NA::MNAStamper::updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::CCVS)] = &NA::MNAStamper::updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::CCCS)] = &NA::MNAStamper::updatebNoop;

  b.setZero();
  const std::vector<Device>& devices = _circuit.devices();
  for (const Device& device : devices) {
    (this->*updatebFunc[static_cast<size_t>(device._type)])(b, device, intMethod);
  }

}

}
