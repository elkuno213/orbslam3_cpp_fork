/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONFIG_H
#define CONFIG_H

// Standard
#include <memory>
#include <string>

namespace ORB_SLAM3 {

// TODO: implement these classes.
class ViewerConfig;
class CameraConfig;
class ORBExtractorConfig;
class IMUConfig;

// TODO: implement the method ParseConfigFile.
class ConfigParser {
public:
  bool ParseConfigFile(const std::string& file);

private:
  std::unique_ptr<ViewerConfig> viewer_config_;
  std::unique_ptr<CameraConfig> camera_config_;
  std::unique_ptr<ORBExtractorConfig> orb_config_;
  std::unique_ptr<IMUConfig> imu_config_;
};

} // namespace ORB_SLAM3

#endif // CONFIG_H
