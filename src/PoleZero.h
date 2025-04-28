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
    bool check();
    bool calcMoments(const Eigen::MatrixXd& G, 
                     const Eigen::MatrixXd& C,
                     const Eigen::VectorXd& E, 
                     std::vector<double>& inputMoments,
                     std::vector<double>& outputMoments) const;

    bool calcTFDenominatorCoeff(const std::vector<double>& moments, 
                                std::vector<double>& coeff) const;
    bool calcTFNumeratorCoeff(const std::vector<double>& moments, 
                              const std::vector<double>& denomCoeff,
                              std::vector<double>& coeff) const;

    bool calcPoles(const std::vector<double>& denomCoeff, 
                   std::vector<Complex>& poles) const;

    bool calcZeros(const std::vector<double>& numCoeff, 
                   std::vector<Complex>& zeros) const;

    bool calcResidues(const std::vector<Complex>& poles, 
                      const std::vector<double>& moments, 
                      std::vector<Complex>& residues) const;

  private:
    const Circuit&         _circuit;
    AnalysisParameter      _param;
    Node                   _inNode;
    Node                   _outNode;
    SimResult              _result;
    size_t                 _eqnDim;
    std::vector<double>    _moments;
    std::vector<Complex>   _poles;
    std::vector<Complex>   _zeros;
    std::vector<Complex>   _residues;
};

}

#endif