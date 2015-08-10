/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "CinderTango.h"
#include "cinder/app/App.h"
#include "cinder/Log.h"

CinderTango::CinderTango() : tango_position(glm::vec3(0.0f, 0.0f, 0.0f)),
      tango_rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)),
      config_(nullptr) {}

// This is called when new pose updates become available. Pair was set to start-
// of-service with respect to ADF frame, which will be available only once
// localized against an ADF. Use this function to check localization status, and
// use GetPoseAtTime to get the current pose.
static void onPoseAvailable(void*, const TangoPoseData* pose) {
  pthread_mutex_lock(&CinderTango::GetInstance().pose_mutex);
  // Update Tango localization status.
  if (pose->status_code == TANGO_POSE_VALID) {
    CinderTango::GetInstance().is_localized = true;
    CI_LOG_I("valid pose onPoseAvailable");
  } else {
    CinderTango::GetInstance().is_localized = false;
  }
  pthread_mutex_unlock(&CinderTango::GetInstance().pose_mutex);
}

// Tango event callback.
static void onTangoEvent(void*, const TangoEvent* event) {
  pthread_mutex_lock(&CinderTango::GetInstance().event_mutex);
  // Update the status string for debug display.
  std::stringstream string_stream;
  string_stream << event->event_key << ": " << event->event_value;
  CinderTango::GetInstance().event_string = string_stream.str();
  pthread_mutex_unlock(&CinderTango::GetInstance().event_mutex);
}

// Get status string based on the pose status code.
const char* CinderTango::getStatusStringFromStatusCode(
    TangoPoseStatusType status) {
  const char* ret_string;
  switch (status) {
    case TANGO_POSE_INITIALIZING:
      ret_string = "Initializing";
      break;
    case TANGO_POSE_VALID:
      ret_string = "Valid";
      break;
    case TANGO_POSE_INVALID:
      ret_string = "Invalid";
      break;
    case TANGO_POSE_UNKNOWN:
      ret_string = "Unknown";
      break;
    default:
      ret_string = "Status_Code_Invalid";
      break;
  }
  if (static_cast<int>(status) < 3) {
    status_count[static_cast<int>(status)] += 1;
  }
  return ret_string;
}

TangoErrorType CinderTango::Initialize(JNIEnv* env, jobject activity) {
  // Initialize Tango Service.
  // The initialize function perform API and Tango Service version check,
  // the there is a mis-match between API and Tango Service version, the
  // function will return TANGO_INVALID.
  return TangoService_initialize(env, activity);
}

bool CinderTango::SetConfig(bool is_auto_recovery) {
  // Get the default TangoConfig.
  // We get the default config first and change the config
  // flag as needed.
  config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (config_ == NULL) {
    ci::app::console()<<"TangoService_getConfig(): Failed"<<std::endl;
    return false;
  }
  // Enable color camera.
  if (TangoConfig_setBool(config_, "config_enable_color_camera", true) !=
      TANGO_SUCCESS) {
    ci::app::console()<<"config_enable_color_camera Failed"<<std::endl;
    return false;
  }
  
  // Turn on auto recovery for motion tracking.
  // Note that the auto-recovery is on by default.
  if (TangoConfig_setBool(config_, "config_enable_auto_recovery",
                          is_auto_recovery) != TANGO_SUCCESS) {
    CI_LOG_E("config_enable_auto_recovery(): Failed");
    return false;
  }

  if (TangoConfig_setBool(config_, "config_enable_low_latency_imu_integration",
                          true) != TANGO_SUCCESS) {
    CI_LOG_E("config_enable_low_latency_imu_integration(): Failed");
    return false;
  }

  // Get library version string from service.
  TangoConfig_getString(config_, "tango_service_library_version",
                        const_cast<char*>(lib_version_string.c_str()),
                        kVersionStringLength);

  // Setting up the start of service to ADF frame for the onPoseAvailable
  // callback,
  // it will check the localization status.
  TangoCoordinateFramePair pair;
  pair.base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
  pair.target = TANGO_COORDINATE_FRAME_START_OF_SERVICE;

  // Attach onPoseAvailable callback.
  // The callback will be called after the service is connected.
  /*if (TangoService_connectOnPoseAvailable(1, &pair, onPoseAvailable) !=
      TANGO_SUCCESS) {
    CI_LOG_E("TangoService_connectOnPoseAvailable(): Failed");
    return false;
  }*/

  // Attach onEventAvailable callback.
  // The callback will be called after the service is connected.
  if (TangoService_connectOnTangoEvent(onTangoEvent) != TANGO_SUCCESS) {
    CI_LOG_E("TangoService_connectOnTangoEvent(): Failed");
    return false;
  }

  // Load the most recent ADF.
  char* uuid_list;

  // uuid_list will contain a comma separated list of UUIDs.
  if (TangoService_getAreaDescriptionUUIDList(&uuid_list) != TANGO_SUCCESS) {
    CI_LOG_I("TangoService_getAreaDescriptionUUIDList failed");
  }
  CI_LOG_I("area list "<<uuid_list[0]);
  // Parse the uuid_list to get the individual uuids.
  if (uuid_list != NULL && uuid_list[0] != '\0') {
    std::vector<std::string> adf_list;

    char* parsing_char;
    char* saved_ptr;
    parsing_char = strtok_r(uuid_list, ",", &saved_ptr);
    while (parsing_char != NULL) {
      std::string s = std::string(parsing_char);
      adf_list.push_back(s);
      parsing_char = strtok_r(NULL, ",", &saved_ptr);
    }

    int list_size = adf_list.size();
    if (list_size == 0) {
      CI_LOG_E("List size is 0");
      return false;
    }
    cur_uuid = adf_list[list_size - 1];
    if (TangoConfig_setString(config_, "config_load_area_description_UUID",
                              adf_list[list_size - 1].c_str()) !=
        TANGO_SUCCESS) {
      CI_LOG_E("config_load_area_description_uuid Failed");
      return false;
    } else {
      CI_LOG_I("Load ADF: " << adf_list[list_size - 1].c_str());
    }
  } else {
    CI_LOG_E("No area description file available, no file loaded.");
  }

  CI_LOG_I("tango setConfig success is_localized "<< is_localized);
  return true;
}

void CinderTango::ConnectTexture(GLuint texture_id) {
  if (TangoService_connectTextureId(TANGO_CAMERA_COLOR, texture_id, nullptr,
                                    nullptr) != TANGO_SUCCESS) {
    ci::app::console()<<"TangoService_connectTextureId(): Failed"<< std::endl;
  }
}

/// Connect to the Tango Service.
/// Note: connecting Tango service will start the motion
/// tracking automatically.
bool CinderTango::Connect() {
  if (TangoService_connect(nullptr, config_) != TANGO_SUCCESS) {
    ci::app::console()<<"TangoService_connect(): Failed"<<std::endl;
    return false;
  }
  return true;
}

void CinderTango::UpdateColorTexture() {
  // TangoService_updateTexture() updates target camera's
  // texture and timestamp.
  if (TangoService_updateTexture(TANGO_CAMERA_COLOR, &timestamp) !=
      TANGO_SUCCESS) {
      ci::app::console()<<"TangoService_updateTexture(): Failed"<<std::endl;
  }
}

bool CinderTango::GetPoseAtTime() {
  // Set the reference frame pair after connect to service.
  // Currently the API will set this set below as default.

  TangoCoordinateFramePair frame_pair;
  frame_pair.base = is_localized ? TANGO_COORDINATE_FRAME_AREA_DESCRIPTION
                                 : TANGO_COORDINATE_FRAME_START_OF_SERVICE;
  frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;

  TangoPoseData pose_latest;
  TangoErrorType result_latest =
      TangoService_getPoseAtTime(0., frame_pair, &pose_latest);
  TangoPoseData pose_texture;
  TangoErrorType result_texture =
      TangoService_getPoseAtTime(timestamp, frame_pair, &pose_texture);
  bool ok_latest = (result_latest == TANGO_SUCCESS &&
                    pose_latest.status_code == TANGO_POSE_VALID);
  bool ok_texture = (result_texture == TANGO_SUCCESS &&
                     pose_texture.status_code == TANGO_POSE_VALID);

  if (ok_latest) {
    const TangoPoseData& pose = ok_texture ? pose_texture : pose_latest;
    tango_position = glm::vec3(pose.translation[0], pose.translation[1],
                               pose.translation[2]);
    tango_rotation = glm::quat(pose.orientation[3], pose.orientation[0],
                               pose.orientation[1], pose.orientation[2]);
    return true;
  }

  /*
  std::stringstream string_stream;
  string_stream.setf(std::ios_base::fixed, std::ios_base::floatfield);
  string_stream.precision(2);
  string_stream << "Tango system event: " << event_string << "\n" << frame_pair
                << "\n"
                << "  status: "
                << getStatusStringFromStatusCode(pose.status_code)
                << ", count: " << status_count[pose.status_code]
                << ", timestamp(ms): " << timestamp << ", position(m): ["
                << pose.translation[0] << ", " << pose.translation[1] << ", "
                << pose.translation[2] << "]"
                << ", orientation: [" << pose.orientation[0] << ", "
                << pose.orientation[1] << ", " << pose.orientation[2] << ", "
                << pose.orientation[3] << "]\n"
                << "Color Camera Intrinsics(px):\n"
                << "  width: " << cc_width << ", height: " << cc_height
                << ", fx: " << cc_fx << ", fy: " << cc_fy;
  pose_string = string_stream.str();
  */
  return true;
}


bool CinderTango::GetExtrinsics() {
  // Retrieve the Extrinsic
  TangoPoseData poseData;
  TangoCoordinateFramePair pair;
  pair.base = TANGO_COORDINATE_FRAME_IMU;
  pair.target = TANGO_COORDINATE_FRAME_DEVICE;
  if (TangoService_getPoseAtTime(0.0, pair, &poseData) != TANGO_SUCCESS) {
    CI_LOG_E("TangoService_getPoseAtTime(): Failed");
    return false;
  }
  imu_p_device = glm::vec3(poseData.translation[0], poseData.translation[1],
                           poseData.translation[2]);
  imu_q_device = glm::quat(poseData.orientation[3], poseData.orientation[0],
                           poseData.orientation[1], poseData.orientation[2]);

  pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  if (TangoService_getPoseAtTime(0.0, pair, &poseData) != TANGO_SUCCESS) {
    CI_LOG_E("TangoService_getPoseAtTime(): Failed");
    return false;
  }
  imu_p_cc = glm::vec3(poseData.translation[0], poseData.translation[1],
                       poseData.translation[2]);
  imu_q_cc = glm::quat(poseData.orientation[3], poseData.orientation[0],
                       poseData.orientation[1], poseData.orientation[2]);
  return true;
}

bool CinderTango::GetIntrinsics() {
  // Retrieve the Intrinsic
  TangoCameraIntrinsics ccIntrinsics;
  if (TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ccIntrinsics) !=
      TANGO_SUCCESS) {
    CI_LOG_E("TangoService_getCameraIntrinsics(): Failed");
    return false;
  }

  // Color camera's image plane width.
  cc_width = ccIntrinsics.width;
  // Color camera's image plane height.
  cc_height = ccIntrinsics.height;
  // Color camera's x axis focal length.
  cc_fx = ccIntrinsics.fx;
  // Color camera's y axis focal length.
  cc_fy = ccIntrinsics.fy;
  // Principal point x coordinate on the image.
  cc_cx = ccIntrinsics.cx;
  // Principal point y coordinate on the image.
  cc_cy = ccIntrinsics.cy;
  for (int i = 0; i < 5; i++) {
    cc_distortion[i] = ccIntrinsics.distortion[i];
  }
  return true;
}

void CinderTango::Disconnect() {
  TangoConfig_free(config_);
  config_ = NULL;
  TangoService_disconnect();
}

CinderTango::~CinderTango() {}
