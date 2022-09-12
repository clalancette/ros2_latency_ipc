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

#include "rcutils/cmdline_parser.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "latency_log.hpp"

class LatencyRec : public rclcpp::Node
{
public:
  LatencyRec(const std::string & log_file)
  : Node("LatencyRec"), log_file_(log_file)
  {
    // prepare timestamp array to avoid allocations
    latency_array_.reserve(10000);

    // define qos
    auto qos = rclcpp::QoS(rclcpp::KeepLast(0)).best_effort().durability_volatile();

    // create subscriber for topic 'ping'
    sub_ =
      create_subscription<std_msgs::msg::String>(
      "ping", qos, std::bind(&LatencyRec::OnReceive, this, std::placeholders::_1));
  }

  void OnReceive(std_msgs::msg::String::UniquePtr msg)
  {
    // read send time
    std::stringstream msg_stream(msg->data);
    uint64_t snd_time;
    char stop_byte;

    msg_stream >> stop_byte >> snd_time;

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
      rec_size_ = msg->data.size();
    } else if (stop_byte == '1') {
      // Final message, evaluate
      evaluate(latency_array_, rec_size_, log_file_);

      // log all latencies into file
      log2file(latency_array_, rec_size_, log_file_);

      // reset latency array and receive size
      latency_array_.clear();
      rec_size_ = 0;
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::vector<uint64_t> latency_array_;
  size_t rec_size_ = 0;
  std::string log_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string log_file;  // base file name to export results
  {
    char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-l");
    if (cli_option) {
      log_file = cli_option;
    }
  }

  rclcpp::spin(std::make_shared<LatencyRec>(log_file));
  rclcpp::shutdown();

  return 0;
}
