#ifndef _TRAN_DEBUG_H_
#define _TRAN_DEBUG_H_

#include <Eigen/Core>
#include <unordered_map>
#include "StringUtil.h"

namespace NA {

class Circuit;
class SimResultMap;

enum class DebugModule : unsigned int {
  None = 0,
  All,
  Root,
  Sim,
  Circuit,
  PZ,
  NLDM,
  CCS, 

};

class Debug {
  public:
    static DebugModule stringToDebugModule(const std::string& str)
    {
      if (iequals(str.data(), "all")) {
        return DebugModule::All;
      } else if (iequals(str.data(), "root")) {
        return DebugModule::Root;
      } else if (iequals(str.data(), "sim")) {
        return DebugModule::Sim;
      } else if (iequals(str.data(), "Circuit")) {
        return DebugModule::Circuit;
      } else if (iequals(str.data(), "pz")) {
        return DebugModule::PZ;
      } else if (iequals(str.data(), "nldm")) {
        return DebugModule::NLDM;
      } else if (iequals(str.data(), "ccs")) {
        return DebugModule::CCS;
      }
      return DebugModule::None;
    }
    static bool enabled(DebugModule m, size_t l = 0) 
    { 
      const auto& found = _debugMap.find(m);
      if (found != _debugMap.end() && found->second > l) {
        return true;
      } 
      const auto& foundAll = _debugMap.find(DebugModule::All);
      if (foundAll != _debugMap.end() && found->second > l) {
        return true;
      } 
      return false;
    };
    static void setLevel(DebugModule m, size_t l) { _debugMap.insert({m, l}); }
    static void printEquation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    static void printVector(double time, const char* name, const Eigen::VectorXd& x);
    static void printEquation(const Eigen::MatrixXcd& A, const Eigen::VectorXcd& b);
    static void printVector(const char* name, const Eigen::VectorXcd& x);
    static void printSolution(double time, const char* name, const Eigen::VectorXd& x,
                              const SimResultMap& resultMap, const Circuit& circuit);
    static void printSolution(const char* name, const Eigen::VectorXd& x);
    static void printSolution(const char* name, const Eigen::VectorXcd& x);

  private:
    static std::unordered_map<DebugModule, size_t> _debugMap;

};

}

#endif
