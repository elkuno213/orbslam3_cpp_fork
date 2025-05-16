#include "MapDrawer.h"
#include <fstream>
#include <gtest/gtest.h>

using namespace ORB_SLAM3;

const std::string kSettingsFile = "/tmp/settings.yaml";

const char* kValidYamlConfig = R"(%YAML:1.0
---
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
)";

const char* kInvalidYamlConfig = R"(%YAML:1.0
---
Viewer.InexistentParam: 1
)";

class MapDrawerTest : public ::testing::Test {
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

TEST_F(MapDrawerTest, ConstructorNonExistentFile) {
  ASSERT_THROW({ MapDrawer m(nullptr, "nonexistent.yaml", nullptr); }, std::runtime_error);
}

TEST_F(MapDrawerTest, ConstructorInvalidFile) {
  CreateInvalidConfig();
  ASSERT_THROW({ MapDrawer m(nullptr, kSettingsFile, nullptr); }, std::runtime_error);
}

TEST_F(MapDrawerTest, ConstructorValidFile) {
  CreateValidConfig();
  ASSERT_NO_THROW({ MapDrawer m(nullptr, kSettingsFile, nullptr); });
}
