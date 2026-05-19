#include "ApplicationLogger.h"

#include <ostream>

ApplicationLogger::ApplicationLogger(std::ostream *output)
    : output_(output) {}

void ApplicationLogger::started(
    const std::filesystem::path &input_output_path) const {
  if (output_ == nullptr) {
    return;
  }

  *output_ << "drone_mapper log enabled\n";
  *output_ << "input_output_path: " << input_output_path << '\n';
}

void ApplicationLogger::scoreSummary(const MappingStats &stats) const {
  if (output_ == nullptr) {
    return;
  }

  Score::writeLogSummary(stats, *output_);
}

void ApplicationLogger::mapOutputWritten(
    const std::filesystem::path &output_path) const {
  if (output_ == nullptr) {
    return;
  }

  *output_ << "wrote map output: " << output_path << '\n';
}
