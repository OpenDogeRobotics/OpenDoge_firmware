#pragma once

#include <string>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

bool loadDeployConfig(
  const std::string & path, const std::array<JointMap, kNumJoints> & joints,
  DeployConfig & config, std::string & error);

bool readCommandFile(const std::string & path, OperatorCommand & command, std::string & error);

bool readImuFile(const std::string & path, ImuSample & imu, double now_s, std::string & error);

}  // namespace opendoge
