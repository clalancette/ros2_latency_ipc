// Copyright 2022 Rex Schilasky
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "latency_log.hpp"

namespace latency_rec
{

class LatencyRecRaw final : public rclcpp::Node
{
public:
  LatencyRecRaw(rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : rclcpp::Node("LatencyRecRaw", options.use_intra_process_comms(false))
  {
    // prepare timestamp array to avoid allocations
    latency_array_.reserve(10000);

    // define qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();

    // create subscriber for topic 'ping'
    sub_ =
      create_subscription<std_msgs::msg::String>(
      "ping", qos, std::bind(&LatencyRecRaw::OnReceive, this, std::placeholders::_1));
  }

  void OnReceive(const std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    auto buf = msg->get_rcl_serialized_message().buffer;
    char snd_time_str[20];
    memcpy(snd_time_str, &buf[9], 20);
    std::stringstream msg_stream(snd_time_str);
    uint64_t snd_time;
    char stop_byte = buf[8];
    size_t data_size = msg->size() - 12;  // 12 bytes of overhead in "raw" messages

    msg_stream >> snd_time;

    if (stop_byte == '2') {
      // A warmup message, so just throw it away;
      return;
    }

    // take receive time
    uint64_t rec_time = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

    if (stop_byte == '0') {
      // A regular message

      // update latency and size
      uint64_t latency = rec_time - snd_time;
      latency_array_.push_back(latency);
      if (rec_size_ != 0 && data_size != rec_size_) {
        RCLCPP_WARN(get_logger(), "Message of different size included in results!");
      }
      rec_size_ = data_size;
    } else if (stop_byte == '1') {
      // Final message, evaluate
      evaluate(latency_array_, rec_size_);

      // reset latency array and receive size
      latency_array_.clear();
      rec_size_ = 0;
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::vector<uint64_t> latency_array_;
  size_t rec_size_ = 0;
};

}  // namespace latency_rec

RCLCPP_COMPONENTS_REGISTER_NODE(latency_rec::LatencyRecRaw)
