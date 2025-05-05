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
  auto logger = spdlog::default_logger()->clone(name);
  spdlog::register_logger(logger);
  return logger;
}

} // namespace ORB_SLAM3::logging
