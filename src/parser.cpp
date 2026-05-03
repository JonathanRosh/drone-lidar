#include "parser.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace parser {
namespace {

std::string trim(const std::string &text) {
  size_t start = 0;
  while (start < text.size() &&
         std::isspace(static_cast<unsigned char>(text[start])) != 0) {
    ++start;
  }

  size_t end = text.size();
  while (end > start &&
         std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
    --end;
  }

  return text.substr(start, end - start);
}

double parseDoubleOrThrow(const std::string &text, const std::string &key,
                          int line_number) {
  std::istringstream iss(text);
  double value = 0.0;
  iss >> value;
  if (!iss || !iss.eof()) {
    throw std::runtime_error("Invalid numeric value for key '" + key +
                             "' at line " + std::to_string(line_number));
  }
  return value;
}

int parseIntOrThrow(const std::string &text, const std::string &key,
                    int line_number) {
  std::istringstream iss(text);
  int value = 0;
  iss >> value;
  if (!iss || !iss.eof()) {
    throw std::runtime_error("Invalid integer value for key '" + key +
                             "' at line " + std::to_string(line_number));
  }
  return value;
}

} // namespace

DroneConfig parseDroneConfigFile(const std::string &path) {
  DroneConfig drone_config;

  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open drone config file: " + path);
  }

  std::string line;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    const std::string cleaned = trim(line);

    if (cleaned.empty() || cleaned[0] == '#') {
      continue;
    }

    const size_t colon_pos = cleaned.find(':');
    if (colon_pos == std::string::npos) {
      throw std::runtime_error("Missing ':' in drone config at line " +
                               std::to_string(line_number));
    }

    const std::string key = trim(cleaned.substr(0, colon_pos));
    const std::string value_text = trim(cleaned.substr(colon_pos + 1));
    if (key.empty() || value_text.empty()) {
      throw std::runtime_error("Empty key/value in drone config at line " +
                               std::to_string(line_number));
    }

    if (key == "min_pass_width") {
      drone_config.min_pass.width =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "min_pass_length") {
      drone_config.min_pass.length =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "min_pass_height") {
      drone_config.min_pass.height =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "max_rotate") {
      drone_config.max_command.rotate =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::deg;
    } else if (key == "max_advance") {
      drone_config.max_command.advance =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "max_elevate") {
      drone_config.max_command.elevate =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "lidar_z_min") {
      drone_config.lidar_config.z_min =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "lidar_z_max") {
      drone_config.lidar_config.z_max =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "lidar_d") {
      drone_config.lidar_config.d =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "lidar_fovc") {
      drone_config.lidar_config.fovc =
          parseIntOrThrow(value_text, key, line_number);
    } else {
      throw std::runtime_error("Unknown key in drone config: '" + key +
                               "' at line " + std::to_string(line_number));
    }
  }

  if (drone_config.min_pass.width <= 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error(
        "drone config invalid: min_pass_width must be > 0");
  }
  if (drone_config.min_pass.length <= 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error(
        "drone config invalid: min_pass_length must be > 0");
  }
  if (drone_config.min_pass.height <= 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error(
        "drone config invalid: min_pass_height must be > 0");
  }

  if (drone_config.max_command.rotate < 0.0 * mp_units::si::unit_symbols::deg) {
    throw std::runtime_error("drone config invalid: max_rotate must be >= 0");
  }
  if (drone_config.max_command.advance < 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error("drone config invalid: max_advance must be >= 0");
  }
  if (drone_config.max_command.elevate < 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error("drone config invalid: max_elevate must be >= 0");
  }

  if (drone_config.lidar_config.z_min < 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error("drone config invalid: lidar_z_min must be >= 0");
  }
  if (drone_config.lidar_config.z_max < 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error("drone config invalid: lidar_z_max must be >= 0");
  }
  if (drone_config.lidar_config.d <= 0.0 * mp_units::si::unit_symbols::cm) {
    throw std::runtime_error("drone config invalid: lidar_d must be > 0");
  }
  if (drone_config.lidar_config.z_max < drone_config.lidar_config.z_min) {
    throw std::runtime_error(
        "drone config invalid: lidar_z_max must be >= lidar_z_min");
  }
  if (drone_config.lidar_config.fovc < 1) {
    throw std::runtime_error("drone config invalid: lidar_fovc must be >= 1");
  }

  return drone_config;
}

MissionConfig parseMissionConfigFile(const std::string &path) {
  MissionConfig mission_config;

  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open mission config file: " + path);
  }

  std::string line;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    const std::string cleaned = trim(line);

    if (cleaned.empty() || cleaned[0] == '#') {
      continue;
    }

    const size_t colon_pos = cleaned.find(':');
    if (colon_pos == std::string::npos) {
      throw std::runtime_error("Missing ':' in mission config at line " +
                               std::to_string(line_number));
    }

    const std::string key = trim(cleaned.substr(0, colon_pos));
    const std::string value_text = trim(cleaned.substr(colon_pos + 1));
    if (key.empty() || value_text.empty()) {
      throw std::runtime_error("Empty key/value in mission config at line " +
                               std::to_string(line_number));
    }

    if (key == "map_boundary_x_min") {
      mission_config.map_boundary.x_min =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "map_boundary_x_max") {
      mission_config.map_boundary.x_max =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "map_boundary_y_min") {
      mission_config.map_boundary.y_min =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "map_boundary_y_max") {
      mission_config.map_boundary.y_max =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "map_boundary_height_min") {
      mission_config.map_boundary.height_min =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "map_boundary_height_max") {
      mission_config.map_boundary.height_max =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "start_position_x") {
      mission_config.start_position.x =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "start_position_y") {
      mission_config.start_position.y =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "start_position_height") {
      mission_config.start_position.height =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::cm;
    } else if (key == "start_position_angle") {
      mission_config.start_position.angle =
          parseDoubleOrThrow(value_text, key, line_number) *
          mp_units::si::unit_symbols::deg;
    } else if (key == "resolution_xy") {
      mission_config.resolution.xy =
          parseIntOrThrow(value_text, key, line_number);
    } else if (key == "resolution_height") {
      mission_config.resolution.height =
          parseIntOrThrow(value_text, key, line_number);
    } else {
      throw std::runtime_error("Unknown key in mission config: '" + key +
                               "' at line " + std::to_string(line_number));
    }
  }

  if (mission_config.resolution.xy < 0) {
    throw std::runtime_error(
        "mission config invalid: resolution_xy must be >= 0");
  }
  if (mission_config.resolution.height < 0) {
    throw std::runtime_error(
        "mission config invalid: resolution_height must be >= 0");
  }

  if (mission_config.map_boundary.x_min > mission_config.map_boundary.x_max) {
    throw std::runtime_error("mission config invalid: map_boundary_x_min must "
                             "be <= map_boundary_x_max");
  }
  if (mission_config.map_boundary.y_min > mission_config.map_boundary.y_max) {
    throw std::runtime_error("mission config invalid: map_boundary_y_min must "
                             "be <= map_boundary_y_max");
  }
  if (mission_config.map_boundary.height_min >
      mission_config.map_boundary.height_max) {
    throw std::runtime_error("mission config invalid: map_boundary_height_min "
                             "must be <= map_boundary_height_max");
  }

  return mission_config;
}

InputMap parseInputMapFile(const std::string &path) {
  InputMap input_map;

  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open map input file: " + path);
  }

  std::string line;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    std::string cleaned = trim(line);

    if (cleaned.empty() || cleaned[0] == '#') {
      continue;
    }

    // Be forgiving: support either "x,y,z" or "x y z".
    std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

    std::istringstream iss(cleaned);
    double x = 0.0;
    double y = 0.0;
    double height = 0.0;
    if (!(iss >> x >> y >> height) || !(iss >> std::ws).eof()) {
      throw std::runtime_error("Invalid map coordinate line at line " +
                               std::to_string(line_number) +
                               ". Expected format: x,y,height");
    }

    InputMap::OccupiedPoint point;
    point.x = x * mp_units::si::unit_symbols::cm;
    point.y = y * mp_units::si::unit_symbols::cm;
    point.height = height * mp_units::si::unit_symbols::cm;
    input_map.occupied_points.push_back(point);
  }

  return input_map;
}

std::string buildPath(const std::string &base_path,
                      const std::string &filename) {
  if (base_path.empty()) {
    return filename;
  }

  if (base_path.back() == '/' || base_path.back() == '\\') {
    return base_path + filename;
  }

  return base_path + "/" + filename;
}

void writeInputErrorsFile(const std::string &base_path,
                          const std::vector<std::string> &errors) {
  if (errors.empty()) {
    return;
  }

  const std::string path = buildPath(base_path, "input_errors.txt");
  std::ofstream out(path);
  if (!out.is_open()) {
    return;
  }

  for (const std::string &error : errors) {
    out << error << "\n";
  }
}

} // namespace parser
