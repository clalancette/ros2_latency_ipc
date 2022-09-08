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

#ifndef LATENCY_LOG_HPP_
#define LATENCY_LOG_HPP_

#include <vector>
#include <string>

void evaluate(
  std::vector<uint64_t> & lat_arr_, size_t rec_size_, size_t warmups_,
  const std::string & log_file_);

void log2file(
  const std::vector<uint64_t> & lat_arr_, size_t rec_size_, const std::string & log_file_);

#endif  // LATENCY_LOG_HPP_
