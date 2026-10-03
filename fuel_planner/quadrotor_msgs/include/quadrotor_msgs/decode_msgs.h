#ifndef QUADROTOR_MSGS_DECODE_MSGS_H
#define QUADROTOR_MSGS_DECODE_MSGS_H

#include <stdint.h>
#include <vector>
#include <quadrotor_msgs/msg/output_data.hpp>
#include <quadrotor_msgs/msg/status_data.hpp>
#include <quadrotor_msgs/msg/ppr_output_data.hpp>

namespace quadrotor_msgs {

bool decodeOutputData(const std::vector<uint8_t>& data, quadrotor_msgs::msg::OutputData& output);
bool decodeStatusData(const std::vector<uint8_t>& data, quadrotor_msgs::msg::StatusData& status);
bool decodePPROutputData(const std::vector<uint8_t>& data,
                         quadrotor_msgs::msg::PPROutputData& output);

}  // namespace quadrotor_msgs

#endif
