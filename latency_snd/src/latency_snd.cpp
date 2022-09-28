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
#include <iostream>
#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

namespace latency_snd
{

class LatencySnd final : public rclcpp::Node
{
public:
  LatencySnd(rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : rclcpp::Node("LatencySnd", options.use_intra_process_comms(false))
  {
    runs_ = declare_parameter("runs", 100);
    uint64_t snd_size_kb = declare_parameter("send_size_kb", 64);
    uint64_t delay_ms_ = declare_parameter("delay_ms", 100);
    warmup_time_ms_ = declare_parameter("warmup_time_ms", 100);

    // log test
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Runs                    : " << runs_ << std::endl;
    std::cout << "Message size            : " << snd_size_kb << " kB" << std::endl;
    std::cout << "Message delay           : " << delay_ms_ << " ms" << std::endl;

    // create message string
    msg_ = std_msgs::msg::String();
    msg_.data.resize(snd_size_kb * 1024);
    std::fill(msg_.data.begin(), msg_.data.end(), '#');

    // define qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();

    // create publisher for topic 'ping'
    pub_ = create_publisher<std_msgs::msg::String>("ping", qos);

    // finally create and start timer for publishing
    auto timer_delay = std::chrono::milliseconds(delay_ms_);
    timer_ = create_wall_timer(timer_delay, std::bind(&LatencySnd::OnPublish, this));
  }

  void OnPublish()
  {
    uint64_t snd_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    if (start_time_us_ == 0) {
      start_time_us_ = snd_time_us;
    }

    char key;

    if ((snd_time_us - start_time_us_) < (warmup_time_ms_ * 1000)) {
      // 2 means a warmup packet
      key = '2';
      fprintf(stderr, "Send time (warmup): %lu\n", snd_time_us);
    } else if (snd_pkgs_ < runs_) {
      // 0 means a data packet
      key = '0';
      fprintf(stderr, "Send time: %lu\n", snd_time_us);
    } else {
      // 1 means EOF
      key = '1';
      fprintf(stderr, "Send time (EOF): %lu\n", snd_time_us);

      // stop timer
      timer_->cancel();

      std::cout << "Messages sent           : " << snd_pkgs_ << std::endl;
      std::cout << "----------------------------------------" << std::endl;
    }

    std::stringstream msg_stream;
    msg_stream << key << snd_time_us;

    std::string tmp_str = msg_stream.str();
    memcpy(&msg_.data[0], tmp_str.c_str(), tmp_str.size());

    pub_->publish(msg_);

    if (key == '0') {
      snd_pkgs_++;
    } else if (key == '1') {
      rclcpp::shutdown();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std_msgs::msg::String msg_;
  size_t snd_pkgs_ = 0;
  size_t runs_ = 0;
  uint64_t warmup_time_ms_ = 0;
  uint64_t start_time_us_ = 0;
};

}  // namespace latency_snd

RCLCPP_COMPONENTS_REGISTER_NODE(latency_snd::LatencySnd)
