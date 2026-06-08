#include "opendoge_deploy/el05_socketcan.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace opendoge
{
namespace
{
constexpr std::uint8_t kCommControl = 0x01;
constexpr std::uint8_t kCommStatus = 0x02;
constexpr std::uint8_t kCommEnable = 0x03;
constexpr std::uint8_t kCommStop = 0x04;
constexpr std::uint8_t kCommReadParam = 0x11;
constexpr std::uint8_t kCommWriteParam = 0x12;
constexpr std::uint32_t kCanEffMask = 0x1FFFFFFF;
constexpr std::uint32_t kStatusFaultMask = 0x3F;
constexpr std::uint16_t kIndexRunMode = 0x7005;
constexpr std::uint16_t kIndexFaultStatus = 0x3022;
constexpr std::uint8_t kRunModeMotion = 0x00;
}  // namespace

double El05SocketCan::clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

std::uint16_t El05SocketCan::floatToUint(double value, double lower, double upper)
{
  const double bounded = clamp(value, lower, upper);
  return static_cast<std::uint16_t>(
    std::lround((bounded - lower) * static_cast<double>(0xFFFF) / (upper - lower)));
}

double El05SocketCan::uintToFloat(std::uint16_t value, double lower, double upper)
{
  return static_cast<double>(value) * (upper - lower) / static_cast<double>(0xFFFF) + lower;
}

std::uint32_t El05SocketCan::buildExtId(
  std::uint8_t comm_type, std::uint16_t data2, std::uint8_t target_id)
{
  return ((static_cast<std::uint32_t>(comm_type) & 0x1F) << 24) |
         ((static_cast<std::uint32_t>(data2) & 0xFFFF) << 8) |
         static_cast<std::uint32_t>(target_id);
}

bool El05SocketCan::open(const std::array<JointMap, kNumJoints> & joints)
{
  ok_ = true;
  last_error_.clear();
  motor_to_index_.clear();
  for (std::size_t i = 0; i < joints.size(); ++i) {
    motor_to_index_[joints[i].motor_id] = i;
    if (!openBus(joints[i].can)) {
      return false;
    }
  }
  return true;
}

void El05SocketCan::close()
{
  for (auto & [_, bus] : buses_) {
    if (bus.fd >= 0) {
      ::close(bus.fd);
      bus.fd = -1;
    }
  }
  buses_.clear();
}

bool El05SocketCan::openBus(const std::string & name)
{
  if (buses_.count(name) != 0) {
    return true;
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd < 0) {
    fail("socket(" + name + "): " + std::strerror(errno));
    return false;
  }

  ifreq ifr{};
  std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", name.c_str());
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    fail("ioctl(" + name + "): " + std::strerror(errno));
    ::close(fd);
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    fail("bind(" + name + "): " + std::strerror(errno));
    ::close(fd);
    return false;
  }

  buses_[name] = Bus{name, fd};
  return true;
}

bool El05SocketCan::sendMotionMode(const JointMap & joint)
{
  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>(kIndexRunMode & 0xFF),
    static_cast<std::uint8_t>((kIndexRunMode >> 8) & 0xFF),
    0,
    0,
    kRunModeMotion,
    0,
    0,
    0};
  return sendFrame(joint.can, kCommWriteParam, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendEnable(const JointMap & joint)
{
  std::array<std::uint8_t, 8> data{};
  return sendFrame(joint.can, kCommEnable, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendStop(const JointMap & joint, bool clear_fault)
{
  std::array<std::uint8_t, 8> data{};
  data[0] = clear_fault ? 1 : 0;
  return sendFrame(joint.can, kCommStop, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendReadFaultStatus(const JointMap & joint)
{
  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>(kIndexFaultStatus & 0xFF),
    static_cast<std::uint8_t>((kIndexFaultStatus >> 8) & 0xFF),
    0,
    0,
    0,
    0,
    0,
    0};
  return sendFrame(joint.can, kCommReadParam, static_cast<std::uint8_t>(joint.motor_id), data, master_id_);
}

bool El05SocketCan::sendMotion(const JointMap & joint, const MotorCommand & command)
{
  const auto tau_u = floatToUint(command.tau, t_min_, t_max_);
  const auto q_u = floatToUint(command.q, p_min_, p_max_);
  const auto dq_u = floatToUint(command.dq, v_min_, v_max_);
  const auto kp_u = floatToUint(command.kp, kp_min_, kp_max_);
  const auto kd_u = floatToUint(command.kd, kd_min_, kd_max_);

  std::array<std::uint8_t, 8> data{
    static_cast<std::uint8_t>((q_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(q_u & 0xFF),
    static_cast<std::uint8_t>((dq_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(dq_u & 0xFF),
    static_cast<std::uint8_t>((kp_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(kp_u & 0xFF),
    static_cast<std::uint8_t>((kd_u >> 8) & 0xFF),
    static_cast<std::uint8_t>(kd_u & 0xFF)};

  return sendFrame(joint.can, kCommControl, static_cast<std::uint8_t>(joint.motor_id), data, tau_u);
}

bool El05SocketCan::sendFrame(
  const std::string & can, std::uint8_t comm_type, std::uint8_t motor_id,
  const std::array<std::uint8_t, 8> & data, std::uint16_t data2)
{
  auto it = buses_.find(can);
  if (it == buses_.end() || it->second.fd < 0) {
    fail("CAN bus unavailable: " + can);
    return false;
  }

  can_frame frame{};
  frame.can_id = buildExtId(comm_type, data2, motor_id) | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::copy(data.begin(), data.end(), frame.data);
  const auto n = ::write(it->second.fd, &frame, sizeof(frame));
  if (n != static_cast<ssize_t>(sizeof(frame))) {
    fail("write(" + can + "): " + std::strerror(errno));
    return false;
  }
  return true;
}

void El05SocketCan::drain(std::array<MotorState, kNumJoints> & states, double now_s)
{
  for (auto & [_, bus] : buses_) {
    while (true) {
      can_frame frame{};
      const auto n = ::read(bus.fd, &frame, sizeof(frame));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        fail("read(" + bus.name + "): " + std::strerror(errno));
        break;
      }
      if (n != static_cast<ssize_t>(sizeof(frame)) || !(frame.can_id & CAN_EFF_FLAG)) {
        continue;
      }
      std::array<std::uint8_t, 8> data{};
      std::copy(frame.data, frame.data + std::min<std::size_t>(frame.can_dlc, 8), data.begin());
      parseFrame(frame.can_id & kCanEffMask, data, states, now_s);
    }
  }
}

void El05SocketCan::parseFrame(
  std::uint32_t can_id, const std::array<std::uint8_t, 8> & data,
  std::array<MotorState, kNumJoints> & states, double now_s)
{
  const auto comm_type = static_cast<std::uint8_t>((can_id >> 24) & 0x1F);
  const auto data2 = static_cast<std::uint16_t>((can_id >> 8) & 0xFFFF);
  const int motor_id = data2 & 0xFF;
  const auto it = motor_to_index_.find(motor_id);
  if (it == motor_to_index_.end()) {
    return;
  }

  auto & state = states[it->second];
  if (comm_type == kCommReadParam) {
    const auto index = static_cast<std::uint16_t>(data[0] | (data[1] << 8));
    if (index == kIndexFaultStatus) {
      state.param_fault =
        static_cast<std::uint32_t>(data[4]) |
        (static_cast<std::uint32_t>(data[5]) << 8) |
        (static_cast<std::uint32_t>(data[6]) << 16) |
        (static_cast<std::uint32_t>(data[7]) << 24);
    }
    return;
  }

  if (comm_type != kCommStatus) {
    return;
  }

  const auto pos_u = static_cast<std::uint16_t>((data[0] << 8) | data[1]);
  const auto vel_u = static_cast<std::uint16_t>((data[2] << 8) | data[3]);
  const auto trq_u = static_cast<std::uint16_t>((data[4] << 8) | data[5]);
  const auto tmp_u = static_cast<std::uint16_t>((data[6] << 8) | data[7]);

  state.position = uintToFloat(pos_u, p_min_, p_max_);
  state.velocity = uintToFloat(vel_u, v_min_, v_max_);
  state.torque = uintToFloat(trq_u, t_min_, t_max_);
  state.temperature = static_cast<double>(tmp_u) * 0.1;
  state.fault = static_cast<std::uint16_t>((data2 >> 8) & kStatusFaultMask);
  state.received = true;
  state.last_feedback_s = now_s;
}

void El05SocketCan::fail(const std::string & error)
{
  ok_ = false;
  last_error_ = error;
}

}  // namespace opendoge
