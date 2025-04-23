#ifndef _PZ_ANALYSIS_H_
#define _PZ_ANALYSIS_H_

#include <complex>
#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "Base.h"
#include "Circuit.h"
#include "SimResult.h"

namespace NA {


class PoleZeroAnalysis {
  public:
    typedef std::complex<double> Complex;

    PoleZeroAnalysis(const Circuit& circuit, const AnalysisParameter& param);

    void run();

    const SimResult& result() const { return _result; }

  private:
    bool check() const;
    bool momentMatching(const Eigen::MatrixXd& G, 
                        const Eigen::MatrixXd& C,
                        const Eigen::VectorXd& E, 
                        std::vector<double>& inputMoments,
                        std::vector<double>& outputMoments) const;

  private:
    const Circuit&         _circuit;
    AnalysisParameter      _param;
    Node                   _inNode;
    Node                   _outNode;
    SimResult              _result;
    size_t                 _eqnDim;
    std::vector<Complex>   _poles;
    std::vector<Complex>   _residues;
};

}

#endif