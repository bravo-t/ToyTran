#include <algorithm>
#include "Base.h"
#include "Circuit.h"
#include "Simulator.h"
#include "MNAStamper.h"

namespace Tran {

static inline bool 
isNodeOmitted(const Simulator* sim, size_t nodeId)
{
  return sim->circuit().isGroundNode(nodeId);
}

static inline void
stampResistor(Eigen::MatrixXd& A, Eigen::VectorXd& /*b*/, 
              const Device& dev, const Simulator* sim)
{
  double stampValue = 1.0 / dev._value;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(dev._negNode);
  if (isNodeOmitted(sim, dev._posNode) == false) {
    A(posNodeIndex, posNodeIndex) += stampValue;
  }
  if (isNodeOmitted(sim, dev._negNode) == false) {
    A(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(sim, dev._posNode) == false && isNodeOmitted(sim, dev._negNode) == false) {
    A(posNodeIndex, negNodeIndex) += -stampValue;
    A(negNodeIndex, posNodeIndex) += -stampValue;
  }
}

static inline void
updatebCapacitorBE(Eigen::VectorXd& b, 
                   const Device& cap, 
                   const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double stampValue = cap._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(cap._negNode);
  double posVoltage = sim->nodeVoltageBackstep(cap._posNode, 1);
  double negVoltage = sim->nodeVoltageBackstep(cap._negNode, 1);
  double voltageDiff = posVoltage - negVoltage;
  double bValue = stampValue * voltageDiff; 
  //printf("DEBUG: T@%G BE posNode: %lu, negNode: %lu, diff: %G-%G=%G current: %G\n", 
  //  sim->simulationResult().currentTime(), cap._posNode, cap._negNode, posVoltage, 
  //  negVoltage, voltageDiff, bValue);
  if (isNodeOmitted(sim, cap._posNode) == false) {
    b(posNodeIndex) += bValue;
  } 
  if (isNodeOmitted(sim, cap._negNode) == false) {
    b(negNodeIndex) += -bValue;
  }
}

static inline void
stampCapacitorBE(Eigen::MatrixXd& A, Eigen::VectorXd& b, 
                 const Device& cap, 
                 const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double stampValue = cap._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(sim, cap._posNode) == false) {
    A(posNodeIndex, posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(sim, cap._negNode) == false) {
    A(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(sim, cap._posNode) == false && isNodeOmitted(sim, cap._negNode) == false) {
    A(posNodeIndex, negNodeIndex) -= stampValue;
    A(negNodeIndex, posNodeIndex) -= stampValue;
  }
  updatebCapacitorBE(b, cap, sim);
}

static inline void
updatebCapacitorGear2(Eigen::VectorXd& b,
                      const Device& cap, 
                      const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue =  cap._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(cap._negNode);
  double posVoltage1 = sim->nodeVoltageBackstep(cap._posNode, 1);
  double negVoltage1 = sim->nodeVoltageBackstep(cap._negNode, 1);
  double posVoltage2 = sim->nodeVoltageBackstep(cap._posNode, 2);
  double negVoltage2 = sim->nodeVoltageBackstep(cap._negNode, 2);
  double voltageDiff1 = posVoltage1 - negVoltage1;
  double voltageDiff2 = posVoltage2 - negVoltage2;
  double stampValue = baseValue * (2 * voltageDiff1 - 0.5 * voltageDiff2);
  //printf("DEBUG: T@%G BDF posNode: %lu, negNode: %lu, diff1: %G-%G=%G, diff2: %G-%G=%G\n", 
  //  sim->simulationResult().currentTime(), cap._posNode, cap._negNode, 
  //    posVoltage1, negVoltage1, voltageDiff1, 
  //    posVoltage2, negVoltage2, voltageDiff2);
  if (isNodeOmitted(sim, cap._posNode) == false) {
    b(posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(sim, cap._negNode) == false) {
    b(negNodeIndex) += -stampValue;
  }
}

static inline void
stampCapacitorGear2(Eigen::MatrixXd& A, Eigen::VectorXd& b, 
                   const Device& cap, 
                   const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue = 1.5 * cap._value / simTick;
  double stampValue = baseValue;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(sim, cap._posNode) == false) {
    A(posNodeIndex, posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(sim, cap._negNode) == false) {
    A(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(sim, cap._posNode) == false && isNodeOmitted(sim, cap._negNode) == false) {
    A(posNodeIndex, negNodeIndex) -= stampValue;
    A(negNodeIndex, posNodeIndex) -= stampValue;
  }
  updatebCapacitorGear2(b, cap, sim);
}

static inline void
updatebCapacitorTrap(Eigen::VectorXd& b,
                     const Device& cap, 
                     const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue =  cap._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(cap._negNode);
  double posVoltage1 = sim->nodeVoltageBackstep(cap._posNode, 1);
  double negVoltage1 = sim->nodeVoltageBackstep(cap._negNode, 1);
  double dV1dt = sim->deviceVoltageDerivative(cap, 1, 1);
  double voltageDiff1 = posVoltage1 - negVoltage1;
  double stampValue = 2 * baseValue * voltageDiff1 + cap._value * dV1dt;
  if (isNodeOmitted(sim, cap._posNode) == false) {
    b(posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(sim, cap._negNode) == false) {
    b(negNodeIndex) += -stampValue;
  }
}

static inline void
stampCapacitorTrap(Eigen::MatrixXd& A, Eigen::VectorXd& b, 
                   const Device& cap, 
                   const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue = 2 * cap._value / simTick;
  double stampValue = baseValue;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(cap._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(cap._negNode);
  if (isNodeOmitted(sim, cap._posNode) == false) {
    A(posNodeIndex, posNodeIndex) += stampValue;
  } 
  if (isNodeOmitted(sim, cap._negNode) == false) {
    A(negNodeIndex, negNodeIndex) += stampValue;
  }
  if (isNodeOmitted(sim, cap._posNode) == false && isNodeOmitted(sim, cap._negNode) == false) {
    A(posNodeIndex, negNodeIndex) -= stampValue;
    A(negNodeIndex, posNodeIndex) -= stampValue;
  }
  updatebCapacitorTrap(b, cap, sim);
}

static inline void
stampCapacitor(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Device& cap, 
               const Simulator* simulator)
{
  IntegrateMethod intMethod = simulator->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      stampCapacitorBE(A, b, cap, simulator);
      break;
    case IntegrateMethod::Gear2:
      stampCapacitorGear2(A, b, cap, simulator);
      break;
    case IntegrateMethod::Trapezoidal:
      stampCapacitorTrap(A, b, cap, simulator);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline void
updatebCapacitor(Eigen::VectorXd& b, const Device& cap, 
                 const Simulator* simulator)
{
  IntegrateMethod intMethod = simulator->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      updatebCapacitorBE(b, cap, simulator);
      break;
    case IntegrateMethod::Gear2:
      updatebCapacitorGear2(b, cap, simulator);
      break;
    case IntegrateMethod::Trapezoidal:
      updatebCapacitorTrap(b, cap, simulator);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline void
updatebInductorBE(Eigen::VectorXd& b,
                  const Device& ind,
                  const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double stampValue = ind._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t deviceIndex = result.deviceVectorIndex(ind._devId);
  double indCurrent = sim->deviceCurrentBackstep(ind._devId, 1);
  double bValue = -stampValue * indCurrent;
  b(deviceIndex) += bValue;
}

static inline void
stampInductorBE(Eigen::MatrixXd& A, Eigen::VectorXd& b, 
                const Device& ind, 
                const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double stampValue = ind._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = result.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(sim, ind._posNode) == false) {
    A(posNodeIndex, deviceIndex) += 1;
    A(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(sim, ind._negNode) == false) {
    A(negNodeIndex, deviceIndex) += -1;
    A(deviceIndex, negNodeIndex) += -1;
  }
  A(deviceIndex, deviceIndex) += -stampValue;
  double indCurrent = sim->deviceCurrentBackstep(ind._devId, 1);
  double bValue = -stampValue * indCurrent;
  b(deviceIndex) += bValue;
}

static inline void
updatebInductorGear2(Eigen::VectorXd& b,
                  const Device& ind,
                  const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue = ind._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t deviceIndex = result.deviceVectorIndex(ind._devId);
  double indCurrent1 = sim->deviceCurrentBackstep(ind._devId, 1);
  double indCurrent2 = sim->deviceCurrentBackstep(ind._devId, 2);
  double stampValue = -baseValue * (2 * indCurrent1 - 0.5 * indCurrent2);
  b(deviceIndex) += stampValue;
}

static inline void
stampInductorGear2(Eigen::MatrixXd& A, Eigen::VectorXd& b, 
                   const Device& ind, 
                   const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue = 1.5 * ind._value / simTick;
  double stampValue = baseValue;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = result.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(sim, ind._posNode) == false) {
    A(posNodeIndex, deviceIndex) += 1;
    A(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(sim, ind._negNode) == false) {
    A(negNodeIndex, deviceIndex) += -1;
    A(deviceIndex, negNodeIndex) += -1;
  }
  A(deviceIndex, deviceIndex) += -stampValue;
  updatebInductorGear2(b, ind, sim);
}

static inline void
updatebInductorTrap(Eigen::VectorXd& b,
                    const Device& ind,
                    const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue = ind._value / simTick;
  const SimResult& result = sim->simulationResult();
  size_t deviceIndex = result.deviceVectorIndex(ind._devId);
  double indCurrent1 = sim->deviceCurrentBackstep(ind._devId, 1);
  double dI1dt = sim->deviceCurrentDerivative(ind, 1, 1);
  double stampValue = -2 * baseValue * indCurrent1 - ind._value * dI1dt;
  b(deviceIndex) += stampValue;
}

static inline void
stampInductorTrap(Eigen::MatrixXd& A, Eigen::VectorXd& b, 
                  const Device& ind, 
                  const Simulator* sim)
{
  double simTick = sim->simulationTick();
  double baseValue = 2 * ind._value / simTick;
  double stampValue = baseValue;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(ind._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(ind._negNode);
  size_t deviceIndex = result.deviceVectorIndex(ind._devId);
  if (isNodeOmitted(sim, ind._posNode) == false) {
    A(posNodeIndex, deviceIndex) += 1;
    A(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(sim, ind._negNode) == false) {
    A(negNodeIndex, deviceIndex) += -1;
    A(deviceIndex, negNodeIndex) += -1;
  }
  A(deviceIndex, deviceIndex) += -stampValue;
  updatebInductorTrap(b, ind, sim);
}

static inline void
stampInductor(Eigen::MatrixXd& A, Eigen::VectorXd& b, const Device& ind, 
              const Simulator* simulator)
{
  IntegrateMethod intMethod = simulator->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      stampInductorBE(A, b, ind, simulator);
      break;
    case IntegrateMethod::Gear2:
      stampInductorGear2(A, b, ind, simulator);
      break;
    case IntegrateMethod::Trapezoidal:
      stampInductorTrap(A, b, ind, simulator);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline void
updatebInductor(Eigen::VectorXd& b, const Device& ind, 
                const Simulator* simulator)
{
  IntegrateMethod intMethod = simulator->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      updatebInductorBE(b, ind, simulator);
      break;
    case IntegrateMethod::Gear2:
      updatebInductorGear2(b, ind, simulator);
      break;
    case IntegrateMethod::Trapezoidal:
      updatebInductorTrap(b, ind, simulator);
      break;
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline double
PWLDataAtTime(const PWLValue& pwlData, double time)
{
  if (time < pwlData._time[0]) {
    return 0;
  }
  for (size_t i=1; i<pwlData._time.size(); ++i) {
    if (time < pwlData._time[i]) {
      double v1 = pwlData._value[i-1];
      double v2 = pwlData._value[i];
      double t1 = pwlData._time[i-1];
      double t2 = pwlData._time[i];
      //printf("DEBUG: time %g goes into [%g, %g] interval, voltage: [%g, %g], interpolated voltage: %g\n", 
      //  time, t1, t2, v1, v2, v1 + (v2-v1)/(t2-t1)*(time-t1));
      return v1 + (v2-v1)/(t2-t1)*(time-t1);
    }
  }
  return pwlData._value.back();
}

static inline void
updatebVoltageSource(Eigen::VectorXd& b,
                     const Device& dev,
                     const Simulator* sim)
{
  double value;
  const SimResult& result = sim->simulationResult();
  if (dev._isPWLValue) {
    const PWLValue& pwlData = sim->circuit().PWLData(dev);
    value = PWLDataAtTime(pwlData, result.currentTime());
  } else {
    value = dev._value;
  }
  size_t deviceIndex = result.deviceVectorIndex(dev._devId);
  b(deviceIndex) += value;
}

static inline void
stampVoltageSource(Eigen::MatrixXd& A, Eigen::VectorXd& b, 
                   const Device& dev, 
                   const Simulator* sim)
{
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(dev._negNode);
  size_t deviceIndex = result.deviceVectorIndex(dev._devId);
  if (isNodeOmitted(sim, dev._posNode) == false) {
    A(posNodeIndex, deviceIndex) += 1;
    A(deviceIndex, posNodeIndex) += 1;
  }
  if (isNodeOmitted(sim, dev._negNode) == false) {
    A(negNodeIndex, deviceIndex) += -1;
    A(deviceIndex, negNodeIndex) += -1;
  }
  updatebVoltageSource(b, dev, sim);
}

static inline void
updatebCurrentSource(Eigen::VectorXd& b,
                     const Device& dev,
                     const Simulator* sim)
{
  double value;
  const SimResult& result = sim->simulationResult();
  if (dev._isPWLValue) {
    const PWLValue& pwlData = sim->circuit().PWLData(dev);
    value = PWLDataAtTime(pwlData, result.currentTime());
  } else {
    value = dev._value;
  }
  size_t posNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(dev._negNode);
  if (isNodeOmitted(sim, dev._posNode) == false) {
    b(posNodeIndex) = -value;
  }
  if (isNodeOmitted(sim, dev._negNode) == false) {
    b(negNodeIndex) = value;
  }
}

static inline void
stampCurrentSource(Eigen::MatrixXd& /*A*/, Eigen::VectorXd& b, 
                   const Device& dev, const Simulator* sim)
{
  updatebCurrentSource(b, dev, sim);
}

static inline void
stampCCVS(Eigen::MatrixXd& A, Eigen::VectorXd& /*b*/, 
          const Device& dev, const Simulator* sim)
{
  const Device& sampleDevice = sim->circuit().device(dev._sampleDevice);
  double value = dev._value;
  if (sampleDevice._posNode == dev._negSampleNode) {
    value = -value;
  }
  const SimResult& result = sim->simulationResult();
  size_t deviceIndex = result.deviceVectorIndex(dev._devId);
  size_t sampleDeviceIndex = result.deviceVectorIndex(dev._sampleDevice);
  size_t posNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = result.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = result.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(sim, dev._posNode) == false) {
    A(sampleDeviceIndex, posSampleNodeIndex) += 1;
    A(posSampleNodeIndex, sampleDeviceIndex) += 1;
    A(deviceIndex, posNodeIndex) += 1;
    A(posNodeIndex, deviceIndex) += 1;
  }
  if (isNodeOmitted(sim, dev._negNode) == false) {
    A(negSampleNodeIndex, sampleDeviceIndex) += -1;
    A(sampleDeviceIndex, negSampleNodeIndex) += -1;
    A(deviceIndex, negNodeIndex) += -1;
    A(negNodeIndex, deviceIndex) += -1;
  }
  A(deviceIndex, sampleDeviceIndex) += value;
}

static inline void
stampVCVS(Eigen::MatrixXd& A, Eigen::VectorXd& /*b*/, 
          const Device& dev, const Simulator* sim)
{
  double value = dev._value;
  const SimResult& result = sim->simulationResult();
  size_t deviceIndex = result.deviceVectorIndex(dev._devId);
  size_t posNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = result.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = result.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(sim, dev._posNode) == false) {
    A(deviceIndex, posSampleNodeIndex) += -value;
    A(deviceIndex, posNodeIndex) += 1;
    A(posNodeIndex, deviceIndex) += 1;
  }
  if (isNodeOmitted(sim, dev._negNode) == false) {
    A(deviceIndex, negSampleNodeIndex) += value;
    A(deviceIndex, negNodeIndex) += 1;
    A(negNodeIndex, deviceIndex) += 1;
  }
}

static inline void
stampCCCS(Eigen::MatrixXd& A, Eigen::VectorXd& /*b*/, 
          const Device& dev, const Simulator* sim)
{
  const Device& sampleDevice = sim->circuit().device(dev._sampleDevice);
  double value = dev._value;
  if (sampleDevice._posNode == dev._negSampleNode) {
    value = -value;
  }
  const SimResult& result = sim->simulationResult();
  size_t sampleDeviceIndex = result.deviceVectorIndex(dev._sampleDevice);
  size_t posNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = result.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = result.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(sim, dev._posNode) == false) {
    A(sampleDeviceIndex, posSampleNodeIndex) += 1;
    A(posSampleNodeIndex, sampleDeviceIndex) += 1;
    A(posNodeIndex, sampleDeviceIndex) += value;
  }
  if (isNodeOmitted(sim, dev._posNode) == false) {
    A(negSampleNodeIndex, sampleDeviceIndex) += -1;
    A(sampleDeviceIndex, negSampleNodeIndex) += -1;
    A(negNodeIndex, sampleDeviceIndex) += -value;
  }
}

static inline void
stampVCCS(Eigen::MatrixXd& A, Eigen::VectorXd& /*b*/, 
          const Device& dev, const Simulator* sim)
{
  double value = dev._value;
  const SimResult& result = sim->simulationResult();
  size_t posNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t negNodeIndex = result.nodeVectorIndex(dev._posNode);
  size_t posSampleNodeIndex = result.nodeVectorIndex(dev._posSampleNode);
  size_t negSampleNodeIndex = result.nodeVectorIndex(dev._negSampleNode);
  if (isNodeOmitted(sim, dev._posNode) == false && 
      isNodeOmitted(sim, dev._posSampleNode) == false) {
    A(posNodeIndex, posSampleNodeIndex) += value;
   }
  if (isNodeOmitted(sim, dev._posNode) == false && 
      isNodeOmitted(sim, dev._negSampleNode) == false) {
    A(posNodeIndex, negSampleNodeIndex) += -value;
  }
  if (isNodeOmitted(sim, dev._negNode) == false && 
      isNodeOmitted(sim, dev._posSampleNode) == false) {
    A(negNodeIndex, posSampleNodeIndex) += -value;
  }
  if (isNodeOmitted(sim, dev._negNode) == false && 
      isNodeOmitted(sim, dev._negSampleNode) == false) {
    A(negNodeIndex, negSampleNodeIndex) += value;
  }
}

void
MNAStamper::stamp(Eigen::MatrixXd& A, 
                  Eigen::VectorXd& b, 
                  const Simulator* sim)
{
  static void (*stampFunc[static_cast<size_t>(DeviceType::Total)])(
        Eigen::MatrixXd& A, Eigen::VectorXd& b, 
        const Device& dev, const Simulator* sim);

  stampFunc[static_cast<size_t>(DeviceType::Resistor)] = stampResistor;
  stampFunc[static_cast<size_t>(DeviceType::Capacitor)] = stampCapacitor;
  stampFunc[static_cast<size_t>(DeviceType::Inductor)] = stampInductor;
  stampFunc[static_cast<size_t>(DeviceType::VoltageSource)] = stampVoltageSource;
  stampFunc[static_cast<size_t>(DeviceType::CurrentSource)] = stampCurrentSource;
  stampFunc[static_cast<size_t>(DeviceType::VCVS)] = stampVCVS;
  stampFunc[static_cast<size_t>(DeviceType::VCCS)] = stampVCCS;
  stampFunc[static_cast<size_t>(DeviceType::CCVS)] = stampCCVS;
  stampFunc[static_cast<size_t>(DeviceType::CCCS)] = stampCCCS;

  const std::vector<Device>& devices = sim->circuit().devices();
  for (const Device& device : devices) {
    stampFunc[static_cast<size_t>(device._type)](A, b, device, sim);
  }
}

void
MNAStamper::updateA(Eigen::FullPivLU<Eigen::MatrixXd>& /*ALU*/, const Simulator* /*sim*/) {
  printf("incremental building and LU decomposing A is not supported right now\n");
  return;
}

static void
updatebNoop(Eigen::VectorXd& /*b*/, 
            const Device& /*dev*/, 
            const Simulator* /*sim*/)
{
  return;
}

void 
MNAStamper::updateb(Eigen::VectorXd& b, const Simulator* sim)
{
  static void (*updatebFunc[static_cast<size_t>(DeviceType::Total)])(
        Eigen::VectorXd& b, const Device& dev, const Simulator* sim);

  updatebFunc[static_cast<size_t>(DeviceType::Resistor)] = updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::Capacitor)] = updatebCapacitor;
  updatebFunc[static_cast<size_t>(DeviceType::Inductor)] = updatebInductor;
  updatebFunc[static_cast<size_t>(DeviceType::VoltageSource)] = updatebVoltageSource;
  updatebFunc[static_cast<size_t>(DeviceType::CurrentSource)] = updatebCurrentSource;
  updatebFunc[static_cast<size_t>(DeviceType::VCVS)] = updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::VCCS)] = updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::CCVS)] = updatebNoop;
  updatebFunc[static_cast<size_t>(DeviceType::CCCS)] = updatebNoop;

  b.setZero();
  const std::vector<Device>& devices = sim->circuit().devices();
  for (const Device& device : devices) {
    updatebFunc[static_cast<size_t>(device._type)](b, device, sim);
  }

}

}
