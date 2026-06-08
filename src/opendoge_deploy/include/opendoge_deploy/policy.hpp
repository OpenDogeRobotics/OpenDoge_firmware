#pragma once

#include <memory>
#include <string>
#include <vector>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

class Policy
{
public:
  virtual ~Policy() = default;
  virtual bool load(const std::string & path, std::string & error) = 0;
  virtual bool infer(const std::array<double, kObsDim> & obs, std::array<double, kNumJoints> & action, std::string & error) = 0;
};

std::unique_ptr<Policy> makePolicy(const std::string & backend);

}  // namespace opendoge
