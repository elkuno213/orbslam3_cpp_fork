#include "LoggingUtils.h"
#include <spdlog/spdlog.h>

namespace ORB_SLAM3::logging {

void InitializeAppLogger(const std::string& name, const bool empty_sink) {
  auto logger = spdlog::default_logger()->clone(name);
  if (empty_sink) {
    logger->sinks().clear();
  }
  spdlog::set_default_logger(logger);
  spdlog::set_pattern("[%Y-%m-%d %T.%e] [%n] [%^%l%$] %v");
}

std::shared_ptr<spdlog::logger> CreateModuleLogger(const std::string& name) {
  // Check if logger exists already.
  auto existing_logger = spdlog::get(name);
  if (existing_logger) {
    return existing_logger;
  }

  // Otherwise, clone base logger and return the new one.
  auto base_logger = spdlog::default_logger();
  if (!base_logger) {
    throw std::runtime_error(
      fmt::format("Base logger not configured when creating logger of {} module", name)
    );
  }

  auto new_logger = spdlog::default_logger()->clone(name);
  spdlog::register_logger(new_logger);
  return new_logger;
}

} // namespace ORB_SLAM3::logging
