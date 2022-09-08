#pragma once

#include <vector>
#include <string>

void evaluate(std::vector<uint64_t> & lat_arr_, size_t rec_size_, size_t warmups_, const std::string & log_file_);
void log2file(const std::vector<uint64_t> & lat_arr_, size_t rec_size_, const std::string & log_file_);
