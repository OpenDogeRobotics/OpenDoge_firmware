#include "opendoge_deploy/policy.hpp"

#ifdef OPENDOGE_HAS_ONNX

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace opendoge
{
namespace
{
class OnnxPolicy final : public Policy
{
public:
  OnnxPolicy()
  : env_(ORT_LOGGING_LEVEL_WARNING, "opendoge_deploy")
  {
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetInterOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  }

  bool load(const std::string & path, std::string & error) override
  {
    try {
      session_ = std::make_unique<Ort::Session>(env_, path.c_str(), session_options_);
      auto input_name = session_->GetInputNameAllocated(0, allocator_);
      auto output_name = session_->GetOutputNameAllocated(0, allocator_);
      input_name_ = input_name.get();
      output_name_ = output_name.get();
    } catch (const std::exception & exc) {
      error = exc.what();
      return false;
    }
    return true;
  }

  bool infer(const std::array<double, kObsDim> & obs, std::array<double, kNumJoints> & action, std::string & error) override
  {
    if (!session_) {
      error = "ONNX session is not loaded";
      return false;
    }

    std::array<float, kObsDim> input{};
    for (std::size_t i = 0; i < kObsDim; ++i) {
      input[i] = static_cast<float>(obs[i]);
    }
    std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(kObsDim)};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto tensor = Ort::Value::CreateTensor<float>(
      memory, input.data(), input.size(), input_shape.data(), input_shape.size());

    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};
    try {
      auto outputs = session_->Run(
        Ort::RunOptions{nullptr}, input_names, &tensor, 1, output_names, 1);
      const auto * out = outputs[0].GetTensorData<float>();
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        action[i] = std::clamp(static_cast<double>(out[i]), -1.0, 1.0);
      }
    } catch (const std::exception & exc) {
      error = exc.what();
      return false;
    }
    return true;
  }

private:
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::AllocatorWithDefaultOptions allocator_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
};
}  // namespace

std::unique_ptr<Policy> makeOnnxPolicy()
{
  return std::make_unique<OnnxPolicy>();
}

}  // namespace opendoge

#endif
