#include "Viewer.h"
#include <fstream>
#include <gtest/gtest.h>

using namespace ORB_SLAM3;

const std::string kSettingsFile = "/tmp/settings.yaml";

const char* kValidYamlConfig = R"(%YAML:1.0
---
Camera.fps: 20
Camera.height: 480
Camera.width: 752
Viewer.imageViewScale: 1.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
)";

const char* kInvalidYamlConfig = R"(%YAML:1.0
---
Viewer.InexistentParam: 1
)";

class ViewerTest : public ::testing::Test {
protected:
  void TearDown() override {
    std::remove(kSettingsFile.c_str());
  }

  void CreateValidConfig() {
    std::ofstream ofs(kSettingsFile);
    ofs << kValidYamlConfig;
    ofs.close();
  }

  void CreateInvalidConfig() {
    std::ofstream ofs(kSettingsFile);
    ofs << kInvalidYamlConfig;
    ofs.close();
  }
};

TEST_F(ViewerTest, ConstructorNonExistentFile) {
  ASSERT_THROW(
    { Viewer v(nullptr, nullptr, nullptr, nullptr, "nonexistent.yaml", nullptr); },
    std::runtime_error
  );
}

TEST_F(ViewerTest, ConstructorInvalidFile) {
  CreateInvalidConfig();
  ASSERT_THROW(
    { Viewer v(nullptr, nullptr, nullptr, nullptr, kSettingsFile, nullptr); },
    std::runtime_error
  );
}

TEST_F(ViewerTest, ConstructorValidFile) {
  CreateValidConfig();
  ASSERT_NO_THROW({ Viewer v(nullptr, nullptr, nullptr, nullptr, kSettingsFile, nullptr); });
}
