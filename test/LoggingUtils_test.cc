#include "LoggingUtils.h"
#include <ranges>
#include <sstream>
#include <string>
#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

namespace ORB_SLAM3::logging {

TEST(LoggingUtilsTest, InitializeAppLoggerWithEmptySink) {
  const std::string name       = "Application";
  const bool        empty_sink = true;
  InitializeAppLogger(name, empty_sink);

  auto logger = spdlog::get(name);

  // Assert.
  ASSERT_NE(logger, nullptr);
  EXPECT_EQ(logger->name(), name);
  EXPECT_TRUE(logger->sinks().empty());
}

TEST(LoggingUtilsTest, InitializeAppLoggerWithDefaultSink) {
  const std::string name       = "Application";
  const bool        empty_sink = false;
  InitializeAppLogger(name, empty_sink);

  auto logger = spdlog::get(name);

  // Assert.
  ASSERT_NE(logger, nullptr);
  EXPECT_EQ(logger->name(), name);
  EXPECT_FALSE(logger->sinks().empty());
}

TEST(LoggingUtilsTest, CreateModuleLogger) {
  const std::string name   = "Module";
  auto              logger = CreateModuleLogger(name);

  // Assert.
  ASSERT_NE(logger, nullptr);
  EXPECT_EQ(logger->name(), name);

  // Verify that the logger is registered.
  auto registered_logger = spdlog::get(name);
  ASSERT_NE(registered_logger, nullptr);
  EXPECT_EQ(registered_logger, logger);
}

TEST(LoggingUtilsTest, LogMessagesToStreamSink) {
  const std::string app_name    = "Application";
  const std::string module_name = "Module";

  // Create a shared stream for both loggers.
  auto stream = std::make_shared<std::ostringstream>();
  auto sink   = std::make_shared<spdlog::sinks::ostream_sink_mt>(*stream);

  // Initialize the application logger.
  InitializeAppLogger(app_name, true);
  auto app_logger = spdlog::get(app_name);
  ASSERT_NE(app_logger, nullptr);
  app_logger->sinks().push_back(sink);

  // Create the module logger.
  auto module_logger = CreateModuleLogger(module_name);
  ASSERT_NE(module_logger, nullptr);

  // Assert same sinks of both loggers.
  const auto& app_sinks    = app_logger->sinks();
  const auto& module_sinks = module_logger->sinks();
  ASSERT_EQ(app_sinks.size(), module_sinks.size());
  EXPECT_TRUE(std::ranges::equal(app_sinks, module_sinks));

  // Log messages from both loggers.
  app_logger->info("Application info message.");
  app_logger->warn("Application warning message.");
  module_logger->info("Module info message.");
  module_logger->error("Module error message.");

  // Flush the loggers to ensure all messages are written to the stream.
  app_logger->flush();
  module_logger->flush();

  // Verify the stream content
  const std::string log_output = stream->str(); // clang-format off
  EXPECT_NE(log_output.find("Application info message."   ), std::string::npos);
  EXPECT_NE(log_output.find("Application warning message."), std::string::npos);
  EXPECT_NE(log_output.find("Module info message."        ), std::string::npos);
  EXPECT_NE(log_output.find("Module error message."       ), std::string::npos); // clang-format on
}

} // namespace ORB_SLAM3::logging
