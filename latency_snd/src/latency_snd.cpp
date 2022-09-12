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

#include "rcutils/cmdline_parser.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class LatencySnd final : public rclcpp::Node
{
public:
  LatencySnd(const rclcpp::NodeOptions & options)
  : rclcpp::Node("LatencySnd", options)
  {
    runs_ = declare_parameter("runs", 100);
    uint64_t snd_size_kb = declare_parameter("send_size_kb", 64);
    uint64_t delay_ms_ = declare_parameter("delay_ms", 100);

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
    uint64_t snd_time = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

    char key;

    if (snd_pkgs_ < warmups_) {
      // 2 means a warmup packet
      key = '2';

      // How often this method gets called depends on what the user passed in for the delay.
      // However, we want to make sure that warmups always take the same amount of time,
      // so only send a warmup packet if enough time has elapsed since the last warmup packet.
      if ((snd_time - last_warmup_packet_time_) < warmup_delay_us_) {
        return;
      }

      last_warmup_packet_time_ = snd_time;

      fprintf(stderr, "Send time (warmup): %lu\n", snd_time);
    } else if (snd_pkgs_ < runs_ + warmups_) {
      // 0 means a data packet
      key = '0';
      fprintf(stderr, "Send time: %lu\n", snd_time);
    } else {
      // 1 means EOF
      key = '1';
      fprintf(stderr, "Send time (EOF): %lu\n", snd_time);
    }

    std::stringstream msg_stream;
    msg_stream << key << snd_time;

    std::string tmp_str = msg_stream.str();
    memcpy(&msg_.data[0], tmp_str.c_str(), tmp_str.size());

    pub_->publish(msg_);

    if (key == '1') {
      // stop timer
      timer_->cancel();

      std::cout << "Messages sent           : " << snd_pkgs_ - warmups_ << std::endl;
      std::cout << "----------------------------------------" << std::endl;

      // shutdown here
      rclcpp::shutdown();
    }

    snd_pkgs_++;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std_msgs::msg::String msg_;
  size_t snd_pkgs_ = 0;
  size_t runs_ = 0;
  const size_t warmups_ = 10;
  uint64_t last_warmup_packet_time_ = 0;
  uint64_t warmup_delay_us_ = 10000;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto node = std::make_shared<LatencySnd>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
