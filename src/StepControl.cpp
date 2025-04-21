#include "StepControl.h"
#include "Simulator.h"
#include "Circuit.h"

namespace NA {

static inline double
capacitorLTEBE(const Device& cap, const Simulator* sim)
{
  double volDeriv = sim->simulationResult().deviceVoltageDerivative(cap, 2, 1);
  double tick = sim->simulationTick();
  return -tick * tick * volDeriv / 2;
}

static inline double
capacitorLTEGear2(const Device& cap, const Simulator* sim)
{
  double volDeriv = sim->simulationResult().deviceVoltageDerivative(cap, 3, 1);
  double tick = sim->simulationTick();
  return tick * tick * tick * volDeriv / 3;
}

static inline double
capacitorLTETrap(const Device& cap, const Simulator* sim)
{
  double volDeriv = sim->simulationResult().deviceVoltageDerivative(cap, 3, 1);
  double tick = sim->simulationTick();
  return -tick * tick * tick * volDeriv / 12;
}

static inline double 
capacitorLTE(const Device& cap, const Simulator* sim)
{
  IntegrateMethod intMethod = sim->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      return capacitorLTEBE(cap, sim);
    case IntegrateMethod::Gear2:
      return capacitorLTEGear2(cap, sim);
    case IntegrateMethod::Trapezoidal:
      return capacitorLTETrap(cap, sim);
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline double
inductorLTEBE(const Device& ind, const Simulator* sim)
{
  double curDeriv = sim->simulationResult().deviceCurrentDerivative(ind, 2, 1);
  double tick = sim->simulationTick();
  return -tick * tick * curDeriv / 2;
}

static inline double
inductorLTEGear2(const Device& ind, const Simulator* sim)
{
  double curDeriv = sim->simulationResult().deviceCurrentDerivative(ind, 3, 1);
  double tick = sim->simulationTick();
  return tick * tick * tick * curDeriv / 3;
}

static inline double
inductorLTETrap(const Device& ind, const Simulator* sim)
{
  double curDeriv = sim->simulationResult().deviceCurrentDerivative(ind, 3, 1);
  double tick = sim->simulationTick();
  return -tick * tick * tick * curDeriv / 12;
}

static inline double 
inductorLTE(const Device& ind, const Simulator* sim)
{
  IntegrateMethod intMethod = sim->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      return capacitorLTEBE(ind, sim);
    case IntegrateMethod::Gear2:
      return capacitorLTEGear2(ind, sim);
    case IntegrateMethod::Trapezoidal:
      return capacitorLTETrap(ind, sim);
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline double
lteNoop(const Device& /*dev*/,
        const Simulator* /*sim*/)
{
  return .0;
}

double
LTE::maxLTE(const Simulator* sim)
{
  static double (*lteFunc[static_cast<size_t>(DeviceType::Total)])(const Device& dev, const Simulator* sim);

  lteFunc[static_cast<size_t>(DeviceType::Resistor)] = lteNoop;
  lteFunc[static_cast<size_t>(DeviceType::Capacitor)] = capacitorLTE;
  lteFunc[static_cast<size_t>(DeviceType::Inductor)] = inductorLTE;
  lteFunc[static_cast<size_t>(DeviceType::VoltageSource)] = lteNoop;
  lteFunc[static_cast<size_t>(DeviceType::CurrentSource)] = lteNoop;
  lteFunc[static_cast<size_t>(DeviceType::VCVS)] = lteNoop;
  lteFunc[static_cast<size_t>(DeviceType::VCCS)] = lteNoop;
  lteFunc[static_cast<size_t>(DeviceType::CCVS)] = lteNoop;
  lteFunc[static_cast<size_t>(DeviceType::CCCS)] = lteNoop;

  double lteValue = .0f;
  const std::vector<Device>& devices = sim->circuit().devices();
  for (const Device& device : devices) {
    double devLTE = lteFunc[static_cast<size_t>(device._type)](device, sim);
    lteValue = std::max(lteValue, std::abs(devLTE));
  }
  return lteValue;
}

static inline double
capacitorStepSizeBE(const Device& cap, const Simulator* sim, double relTol)
{
  double volDeriv = sim->simulationResult().deviceVoltageDerivative(cap, 2, 1);
  if (volDeriv == 0) {
    return 1e99;
  }
  //double tick = sim->simulationTick();
  return std::sqrt(2 * relTol / std::abs(volDeriv));
  //return -tick * tick * volDeriv / 2;
}

static inline double
capacitorStepSizeGear2(const Device& cap, const Simulator* sim, double relTol)
{
  double volDeriv = sim->simulationResult().deviceVoltageDerivative(cap, 3, 1);
  if (volDeriv == 0) {
    return 1e99;
  }
  //double tick = sim->simulationTick();
  return std::cbrt(3 * relTol / std::abs(volDeriv));
  //return tick * tick * tick * volDeriv / 3;
}

static inline double
capacitorStepSizeTrap(const Device& cap, const Simulator* sim, double relTol)
{
  double volDeriv = sim->simulationResult().deviceVoltageDerivative(cap, 3, 1);
  if (volDeriv == 0) {
    return 1e99;
  }
  //double tick = sim->simulationTick();
  return std::cbrt(12 * relTol / std::abs(volDeriv));
  //return -tick * tick * tick * volDeriv / 12;
}

static inline double 
capacitorStepSize(const Device& cap, const Simulator* sim, double relTol)
{
  IntegrateMethod intMethod = sim->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      return capacitorStepSizeBE(cap, sim, relTol);
    case IntegrateMethod::Gear2:
      return capacitorStepSizeGear2(cap, sim, relTol);
    case IntegrateMethod::Trapezoidal:
      return capacitorStepSizeTrap(cap, sim, relTol);
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline double
inductorStepSizeBE(const Device& ind, const Simulator* sim, double relTol)
{
  double curDeriv = sim->simulationResult().deviceCurrentDerivative(ind, 2, 1);
  if (curDeriv == 0) {
    return 1e99;
  }
  //double tick = sim->simulationTick();
  return std::sqrt(2 * relTol / std::abs(curDeriv));
  //return -tick * tick * curDeriv / 2;
}

static inline double
inductorStepSizeGear2(const Device& ind, const Simulator* sim, double relTol)
{
  double curDeriv = sim->simulationResult().deviceCurrentDerivative(ind, 3, 1);
  if (curDeriv == 0) {
    return 1e99;
  }
  //double tick = sim->simulationTick();
  return std::cbrt(3 * relTol / std::abs(curDeriv));
  ///return tick * tick * tick * curDeriv / 3;
}

static inline double
inductorStepSizeTrap(const Device& ind, const Simulator* sim, double relTol)
{
  double curDeriv = sim->simulationResult().deviceCurrentDerivative(ind, 3, 1);
  if (curDeriv == 0) {
    return 1e99;
  }
  //double tick = sim->simulationTick();
  return std::cbrt(12 * relTol / std::abs(curDeriv));
  ///return -tick * tick * tick * curDeriv / 12;
}

static inline double 
inductorStepSize(const Device& ind, const Simulator* sim, double relTol)
{
  IntegrateMethod intMethod = sim->integrateMethod();
  switch (intMethod) {
    case IntegrateMethod::BackwardEuler:
      return capacitorStepSizeBE(ind, sim, relTol);
    case IntegrateMethod::Gear2:
      return capacitorStepSizeGear2(ind, sim, relTol);
    case IntegrateMethod::Trapezoidal:
      return capacitorStepSizeTrap(ind, sim, relTol);
    default:
      assert(false && "Incorrect integrate method");
  }
}

static inline double
stepNoop(const Device& /*dev*/,
        const Simulator* /*sim*/, 
        double /*relTol*/)
{
  return 1e99;
}

double
StepControl::stepLimit(const Simulator* sim, double relTol)
{
  static double (*stepSizeFunc[static_cast<size_t>(DeviceType::Total)])
                  (const Device& dev, const Simulator* sim, double relTol);

  stepSizeFunc[static_cast<size_t>(DeviceType::Resistor)] = stepNoop;
  stepSizeFunc[static_cast<size_t>(DeviceType::Capacitor)] = capacitorStepSize;
  stepSizeFunc[static_cast<size_t>(DeviceType::Inductor)] = inductorStepSize;
  stepSizeFunc[static_cast<size_t>(DeviceType::VoltageSource)] = stepNoop;
  stepSizeFunc[static_cast<size_t>(DeviceType::CurrentSource)] = stepNoop;
  stepSizeFunc[static_cast<size_t>(DeviceType::VCVS)] = stepNoop;
  stepSizeFunc[static_cast<size_t>(DeviceType::VCCS)] = stepNoop;
  stepSizeFunc[static_cast<size_t>(DeviceType::CCVS)] = stepNoop;
  stepSizeFunc[static_cast<size_t>(DeviceType::CCCS)] = stepNoop;

  double stepSizeLimit = std::numeric_limits<double>::max();
  const std::vector<Device>& devices = sim->circuit().devices();
  for (const Device& device : devices) {
    double devStepSize = stepSizeFunc[static_cast<size_t>(device._type)](device, sim, relTol);
    stepSizeLimit = std::min(stepSizeLimit, devStepSize);
  }
  return stepSizeLimit;
}


}