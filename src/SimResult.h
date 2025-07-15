#ifndef _TRAN_SIMRES_H_
#define _TRAN_SIMRES_H_

#include <vector>
#include <deque>
#include <limits>
#include "Base.h"
#include "Circuit.h"

namespace NA {

struct WaveformPoint {
  double _time = 0;
  double _value = 0;
};

struct Waveform {
  Waveform(const std::vector<WaveformPoint>& points)
  : _points(points) {}

  Waveform(const PWLValue& pwlValue);

  void addPoint(double time, double value) 
  {
    _points.push_back({time, value});
  }

  double measure(double targetValue) const;
  bool isRise() const { return _points[0]._value < _points.back()._value; }
  std::vector<WaveformPoint> data() const { return _points; }
  void range(double& max, double& min) const;

  std::vector<WaveformPoint> _points;
};


/// @brief The map between numbers in x of Ax=b, and the actual meaning of 
///        the number
///        size() should be the dimention of vector x
struct SimResultMap {
  SimResultMap() = default;
  
  void clear() 
  {
    _dimension = 0;
    _nodeVoltageMap.clear();
    _deviceCurrentMap.clear();
  }
  void copy(const SimResultMap& other) 
  {
    clear();
    _dimension = other._dimension;
    _nodeVoltageMap = other._nodeVoltageMap;
    _deviceCurrentMap =other._deviceCurrentMap;
  }
  void swap(SimResultMap& other) 
  {
    _dimension = other._dimension;
    _nodeVoltageMap.swap(other._nodeVoltageMap);
    _deviceCurrentMap.swap(other._deviceCurrentMap);
  }

  static size_t invalidValue() { return static_cast<size_t>(-1); }
  size_t _dimension = 0;
  std::vector<size_t> _nodeVoltageMap; /// node ID to matrix index
  std::vector<size_t> _deviceCurrentMap; /// device ID to matrix index

  size_t size() const { return _dimension; }
  void setDimention(size_t val) { _dimension = val; }
};

/// @brief The solution data of every time step produced by solving Ax=b
class SimResult {
  public:
    SimResult(const Circuit* ckt, const std::string& name);
    SimResult() = default;

    void clear()
    {
      _name.clear();
      _map.clear();
      _ticks.clear();
      _values.clear();
    }

    void copy(const SimResult& other)
    {
      clear();
      _ckt = other._ckt;
      _name = other._name;
      _map.copy(other._map);
      _ticks = other._ticks;
      _values = other._values;
    }
    
    void swap(SimResult& other)
    {
      _ckt = other._ckt;
      _name.swap(other._name);
      _map.swap(other._map);
      _ticks.swap(other._ticks);
      _values.swap(other._values);
    }

    std::string name() const { return _name; }
    /// Return Time of given step
    double stepTime(size_t step) const;

    const Circuit* circuit() const { return _ckt; }

    const SimResultMap& indexMap() const { return _map; }
    const std::vector<double>& ticks() const { return _ticks; }
    const std::deque<double>& values() const { return _values; }
    double tick(size_t i) const { return _ticks[i]; }
    double value(size_t i) const { return _values[i]; }
    
    /// For Simulator access
    SimResultMap& indexMap() { return _map; }
    std::vector<double>& ticks() { return _ticks; }
    std::deque<double>& values() { return _values; }
    
    /// @brief Get the index of the result vector x, which is also the row and column
    ///        Index of matrix A, from given node/device id
    size_t nodeVectorIndex(size_t nodeId) const;
    size_t deviceVectorIndex(size_t deviveId) const;
   
    /// @brief Get accumulated simulation time
    double currentTime() const;
    /// @brief Get size of the simulation result
    size_t size() const { return _ticks.size(); }
    /// @brief Get simulation time step size of previous n steps
    ///        If steps is -1, it means the step size of previous 
    ///        step to current step is returned
    double stepSize(size_t steps) const;
    
    /// @brief Get voltage or current of given node id or device id
    ///        This function returns the data in a forward manner
    ///        Means a timeStep of 0 gives the voltage/current @ 0 tick
    double nodeVoltage(size_t nodeId, size_t timeStep) const;
    double deviceCurrent(size_t devId, size_t timeStep) const;
    /// @brief Get the voltage of given node id
    /// @param nodeId 
    /// @param steps: number of steps BACK with respect to current time
    ///               0 means 1 simTick previous to current time
    ///               if requested steps are not available, initial condition 
    ///               is returned
    double nodeVoltageBackstep(size_t nodeId, size_t steps) const;
    /// @brief Get the current of given device id, parameters are interpreted 
    ///        the same way as nodeVoltageBackstep
    double deviceCurrentBackstep(size_t devId, size_t steps) const;
    
    /// @brief Get derivative of voltage and current.
    ///        order controls the order of derivative you need
    ///        order should be postive, 2 mean 2nd derivative for example
    double nodeVoltageDerivative(size_t nodeId, size_t order, size_t steps) const;
    /// deviceVoltageDerivative calculates the derivative of the voltage diff between
    /// positive node and negative node of the given device
    double deviceVoltageDerivative(const Device& device, size_t order, size_t steps) const;
    double deviceCurrentDerivative(const Device& device, size_t order, size_t steps) const;

    /// Get waveform data
    Waveform nodeVoltageWaveform(const std::string& nodeName, double& max, double& min) const;
    Waveform deviceCurrentWaveform(const std::string& devName, double& max, double& min) const;
    Waveform nodeVoltageWaveform(size_t nodeId) const;
    Waveform deviceCurrentWaveform(size_t devId) const;

    /// get total charge in simulation time
    double totalCharge(const Device& dev) const;

  
  private:
    void init(const Circuit* ckt);
    double nodeVoltageImp(size_t nodeId, size_t timeStep) const;
    double deviceCurrentImp(size_t devId, size_t timeStep) const;
    double nodeVoltageBackstepImp(size_t nodeId, size_t steps) const;
    double deviceCurrentBackstepImp(size_t devId, size_t steps) const;
    Waveform waveformData(size_t rowIndex, double* max, double* min) const;
  
  private:
    const Circuit*      _ckt = nullptr;
    std::string         _name;
    SimResultMap        _map;
    std::vector<double> _ticks;
    std::deque<double>  _values; /// size should be _map.size()*_ticks.size()
  
};
}

#endif
