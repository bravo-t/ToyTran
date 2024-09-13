#include "Measure.h"

namespace Tran {

double
measure(const Simulator& simulator, const MeasurePoint& mp)
{
  return .0f;
}

void
Measure::run() const
{
  for (const MeasurePoint& mp : _measurePoints) {
    measure(_simulator, mp);
  }
}

}