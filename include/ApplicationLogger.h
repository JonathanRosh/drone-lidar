#pragma once

#include "Score.h"

#include <filesystem>
#include <iosfwd>

class ApplicationLogger {
  std::ostream *output_;

public:
  explicit ApplicationLogger(std::ostream *output = nullptr);

  void started(const std::filesystem::path &input_output_path) const;
  void scoreSummary(const MappingStats &stats) const;
  void mapOutputWritten(const std::filesystem::path &output_path) const;
};
