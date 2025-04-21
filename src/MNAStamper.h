#ifndef _TRAN_MNASTM_H_
#define _TRAN_MNASTM_H_

#include "Base.h"
#include "Circuit.h"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace NA {

class AnalysisParameter;
class Circuit;
class SimResult;

class MNAStamper {
  public:
    MNAStamper(const AnalysisParameter& param, const Circuit& ckt, const SimResult& simResult)
    : _analysisParam(param), _circuit(ckt), _simResult(simResult) {}
    void stamp(Eigen::MatrixXd& G, Eigen::MatrixXd& C, Eigen::VectorXd& b, IntegrateMethod intMethod);
    void updateb(Eigen::VectorXd& b, IntegrateMethod intMethod);

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
    void stampCCVS(Eigen::MatrixXd& G, Eigen::MatrixXd& /*C*/, 
                   Eigen::VectorXd& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampVCVS(Eigen::MatrixXd& G, Eigen::MatrixXd& /*C*/, 
                   Eigen::VectorXd& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampCCCS(Eigen::MatrixXd& G, Eigen::MatrixXd& /*C*/, 
                   Eigen::VectorXd& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampVCCS(Eigen::MatrixXd& G, Eigen::MatrixXd& /*C*/, 
                   Eigen::VectorXd& /*b*/, const Device& dev,
                   IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampVoltageSource(Eigen::MatrixXd& G, Eigen::MatrixXd& /*C*/,
                            Eigen::VectorXd& b, const Device& dev,
                            IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampCurrentSource(Eigen::MatrixXd& /*G*/, Eigen::MatrixXd& /*C*/, 
                            Eigen::VectorXd& b, const Device& dev, 
                            IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampCapacitor(Eigen::MatrixXd& G, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& cap, 
                        IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampResistor(Eigen::MatrixXd& G, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& dev,
                       IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void stampInductor(Eigen::MatrixXd& G, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& ind, 
                       IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    /// update functions for b
    void updatebCapacitor(Eigen::VectorXd& b, const Device& cap, 
                          IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebInductor(Eigen::VectorXd& b, const Device& ind, 
                         IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebVoltageSource(Eigen::VectorXd& b, const Device& dev,
                              IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebCurrentSource(Eigen::VectorXd& b, const Device& dev,
                              IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    void updatebNoop(Eigen::VectorXd& b, const Device& dev,
                     IntegrateMethod intMethod = IntegrateMethod::BackwardEuler) const;
    
    /// stamp and update functions for specific integration methods
    void updatebCapacitorBE(Eigen::VectorXd& b, const Device& cap) const;
    void stampCapacitorBE(Eigen::MatrixXd& G, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& cap) const;
    void updatebCapacitorGear2(Eigen::VectorXd& b, const Device& cap) const;
    void stampCapacitorGear2(Eigen::MatrixXd& /*G*/, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& cap) const;
    void updatebCapacitorTrap(Eigen::VectorXd& b, const Device& cap) const;
    void stampCapacitorTrap(Eigen::MatrixXd& /*G*/, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& cap) const;
    void updatebInductorBE(Eigen::VectorXd& b, const Device& ind) const;
    void stampInductorBE(Eigen::MatrixXd& /*G*/, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& ind) const;
    void updatebInductorGear2(Eigen::VectorXd& b, const Device& ind) const;
    void stampInductorGear2(Eigen::MatrixXd& /*G*/, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& ind) const;
    void updatebInductorTrap(Eigen::VectorXd& b, const Device& ind) const;
    void stampInductorTrap(Eigen::MatrixXd& /*G*/, Eigen::MatrixXd& C, Eigen::VectorXd& b, const Device& ind) const;

  private:
    AnalysisParameter _analysisParam;
    const Circuit& _circuit;
    const SimResult& _simResult;
};

}

#endif