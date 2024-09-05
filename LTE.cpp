#include "LTE.h"
#include "Simulator.h"
#include "Circuit.h"

namespace Tran {

static inline double
capacitorLTEBE(const Device& cap, const Simulator* sim)
{
  double volDeriv = sim->deviceVoltageDerivative(cap, 2, 1);
  double tick = sim->simulationTick();
  return -tick * tick * volDeriv / 2;
}

static inline double
capacitorLTEGear2(const Device& cap, const Simulator* sim)
{
  double volDeriv = sim->deviceVoltageDerivative(cap, 2, 1);
  double tick = sim->simulationTick();
  return -tick * tick * volDeriv / 2;
}

static inline double
capacitorLTETrap(const Device& cap, const Simulator* sim)
{
  double volDeriv = sim->deviceVoltageDerivative(cap, 3, 1);
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
inductorLTEBE(const Device& cap, const Simulator* sim)
{
  double curDeriv = sim->deviceCurrentDerivative(cap, 2, 1);
  double tick = sim->simulationTick();
  return -tick * tick * curDeriv / 2;
}

static inline double
inductorLTEGear2(const Device& cap, const Simulator* sim)
{
  double curDeriv = sim->deviceCurrentDerivative(cap, 2, 1);
  double tick = sim->simulationTick();
  return -tick * tick * curDeriv / 2;
}

static inline double
inductorLTETrap(const Device& cap, const Simulator* sim)
{
  double curDeriv = sim->deviceCurrentDerivative(cap, 3, 1);
  double tick = sim->simulationTick();
  return -tick * tick * tick * curDeriv / 12;
}

static inline double 
inductorLTE(const Device& cap, const Simulator* sim)
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
    lteValue = std::max(lteValue, devLTE);
  }

}


}