#pragma once

#include <string>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

struct Options
{
  std::string policy_backend{"none"};
  std::string policy_path;
  std::string config_path;
  std::string command_file;
  std::string imu_file;
  bool dry_run{true};
  bool enable{false};
  bool clear_fault{false};
  bool start_active{false};
  bool allow_missing_imu{false};
  bool realtime{false};
  int cpu{-1};
  int rt_priority{60};
  double duration_s{0.0};
  std::string status_file;
  OperatorCommand static_command;
};

void printUsage();

bool parseArgs(int argc, char ** argv, Options & opt);

void applyRuntimeTuning(const Options & opt);

}  // namespace opendoge
