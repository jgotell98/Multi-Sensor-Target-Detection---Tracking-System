#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace mst {

class CsvWriter final {
 public:
  CsvWriter() = default;
  explicit CsvWriter(const std::filesystem::path& path) { open(path); }

  void open(const std::filesystem::path& path) {
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_) throw std::runtime_error("failed to open: " + path.string());
  }

  void write_line(const std::string& line) { out_ << line << "\n"; }
  void flush() { out_.flush(); }

 private:
  std::ofstream out_;
};

}  // namespace mst

