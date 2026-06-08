#include "opendoge_deploy/policy.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace opendoge
{
namespace
{
class NonePolicy final : public Policy
{
public:
  bool load(const std::string &, std::string &) override { return true; }
  bool infer(const std::array<double, kObsDim> &, std::array<double, kNumJoints> & action, std::string &) override
  {
    action.fill(0.0);
    return true;
  }
};

class LinearCsvPolicy final : public Policy
{
public:
  bool load(const std::string & path, std::string & error) override
  {
    std::ifstream file(path);
    if (!file) {
      error = "cannot open linear_csv policy: " + path;
      return false;
    }
    weights_.clear();
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::vector<double> row;
      std::stringstream ss(line);
      std::string token;
      while (std::getline(ss, token, ',')) {
        if (!token.empty()) {
          row.push_back(std::stod(token));
        }
      }
      weights_.push_back(row);
    }
    if (weights_.size() != kNumJoints) {
      error = "linear_csv must have 12 rows";
      return false;
    }
    for (const auto & row : weights_) {
      if (row.size() != kObsDim + 1) {
        error = "linear_csv row must have obs_dim + bias columns";
        return false;
      }
    }
    return true;
  }

  bool infer(const std::array<double, kObsDim> & obs, std::array<double, kNumJoints> & action, std::string &) override
  {
    for (std::size_t out = 0; out < kNumJoints; ++out) {
      double value = weights_[out].back();
      for (std::size_t i = 0; i < kObsDim; ++i) {
        value += weights_[out][i] * obs[i];
      }
      action[out] = std::clamp(value, -1.0, 1.0);
    }
    return true;
  }

private:
  std::vector<std::vector<double>> weights_;
};

class MissingOnnxPolicy final : public Policy
{
public:
  bool load(const std::string &, std::string & error) override
  {
    error = "opendoge_deploy was built without ONNX Runtime";
    return false;
  }
  bool infer(const std::array<double, kObsDim> &, std::array<double, kNumJoints> &, std::string & error) override
  {
    error = "ONNX backend unavailable";
    return false;
  }
};
}  // namespace

std::unique_ptr<Policy> makeOnnxPolicy();

std::unique_ptr<Policy> makePolicy(const std::string & backend)
{
  if (backend == "none") {
    return std::make_unique<NonePolicy>();
  }
  if (backend == "linear_csv") {
    return std::make_unique<LinearCsvPolicy>();
  }
  if (backend == "onnx") {
#ifdef OPENDOGE_HAS_ONNX
    return makeOnnxPolicy();
#else
    return std::make_unique<MissingOnnxPolicy>();
#endif
  }
  return nullptr;
}

}  // namespace opendoge
