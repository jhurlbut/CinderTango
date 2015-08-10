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

#include "tango_data.h"

TangoData::TangoData()
    : tango_position(glm::vec3(0.0f, 0.0f, 0.0f)),
      tango_rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f)),
      config_(nullptr) {}

// This is called when new pose updates become available. Pair was set to start-
// of-service with respect to ADF frame, which will be available only once
// localized against an ADF. Use this function to check localization status, and
// use GetPoseAtTime to get the current pose.
static void onPoseAvailable(void*, const TangoPoseData* pose) {
  pthread_mutex_lock(&TangoData::GetInstance().pose_mutex);
  // Update Tango localization status.
  if (pose->status_code == TANGO_POSE_VALID) {
    TangoData::GetInstance().is_localized = true;
  } else {
    TangoData::GetInstance().is_localized = false;
  }
  pthread_mutex_unlock(&TangoData::GetInstance().pose_mutex);
}

// Tango event callback.
static void onTangoEvent(void*, const TangoEvent* event) {
  pthread_mutex_lock(&TangoData::GetInstance().event_mutex);
  // Update the status string for debug display.
  std::stringstream string_stream;
  string_stream << event->event_key << ": " << event->event_value;
  TangoData::GetInstance().event_string = string_stream.str();
  pthread_mutex_unlock(&TangoData::GetInstance().event_mutex);
}

// Get status string based on the pose status code.
const char* TangoData::getStatusStringFromStatusCode(
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

TangoErrorType TangoData::Initialize(JNIEnv* env, jobject activity) {
  // Initialize Tango Service.
  // The initialize function perform API and Tango Service version check,
  // the there is a mis-match between API and Tango Service version, the
  // function will return TANGO_INVALID.
  return TangoService_initialize(env, activity);
}

bool TangoData::SetConfig(bool is_auto_recovery) {
   // Get the default TangoConfig.
  // We get the default config first and change the config
  // flag as needed.
  config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (config_ == NULL) {
    //ci::app::console()<<"TangoService_getConfig(): Failed"<<std::endl;
    return false;
  }
  // Enable color camera.
  if (TangoConfig_setBool(config_, "config_enable_color_camera", true) !=
      TANGO_SUCCESS) {
    //ci::app::console()<<"config_enable_color_camera Failed"<<std::endl;
    return false;
  }
  return true;
}

bool TangoData::GetPoseAtTime() {
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

bool TangoData::GetExtrinsics() {
  // Retrieve the Extrinsic
  TangoPoseData poseData;
  TangoCoordinateFramePair pair;
  pair.base = TANGO_COORDINATE_FRAME_IMU;
  pair.target = TANGO_COORDINATE_FRAME_DEVICE;
  if (TangoService_getPoseAtTime(0.0, pair, &poseData) != TANGO_SUCCESS) {
    LOGE("TangoService_getPoseAtTime(): Failed");
    return false;
  }
  imu_p_device = glm::vec3(poseData.translation[0], poseData.translation[1],
                           poseData.translation[2]);
  imu_q_device = glm::quat(poseData.orientation[3], poseData.orientation[0],
                           poseData.orientation[1], poseData.orientation[2]);

  pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  if (TangoService_getPoseAtTime(0.0, pair, &poseData) != TANGO_SUCCESS) {
    LOGE("TangoService_getPoseAtTime(): Failed");
    return false;
  }
  imu_p_cc = glm::vec3(poseData.translation[0], poseData.translation[1],
                       poseData.translation[2]);
  imu_q_cc = glm::quat(poseData.orientation[3], poseData.orientation[0],
                       poseData.orientation[1], poseData.orientation[2]);
  return true;
}

bool TangoData::GetIntrinsics() {
  // Retrieve the Intrinsic
  TangoCameraIntrinsics ccIntrinsics;
  if (TangoService_getCameraIntrinsics(TANGO_CAMERA_COLOR, &ccIntrinsics) !=
      TANGO_SUCCESS) {
    LOGE("TangoService_getCameraIntrinsics(): Failed");
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

void TangoData::ConnectTexture(GLuint texture_id) {
  if (TangoService_connectTextureId(TANGO_CAMERA_COLOR, texture_id, nullptr,
                                    nullptr) == TANGO_SUCCESS) {
    LOGI("TangoService_connectTextureId(): Success!");
  } else {
    LOGE("TangoService_connectTextureId(): Failed!");
  }
}

void TangoData::UpdateColorTexture() {
  if (TangoService_updateTexture(TANGO_CAMERA_COLOR, &timestamp) !=
      TANGO_SUCCESS) {
    LOGE("TangoService_updateTexture(): Failed");
  }
}

// Reset the Motion Tracking.
void TangoData::ResetMotionTracking() { TangoService_resetMotionTracking(); }

// Connect to Tango Service, service will start running, and
// POSE can be queried.
TangoErrorType TangoData::Connect() {
  return TangoService_connect(nullptr, config_);
}

// Disconnect Tango Service.
// Disconnect will disconnect all callback from Tango Service,
// after resume, the application should re-connect all callback
// and connect to service.
// Disconnect will also reset all configuration to default.
// Before disconnecting the service, the application is reponsible to
// free the config_ handle as well.
//
// When running two Tango applications, the first application needs to
// disconnect the service, so that second application could connect the
// service with a new configration handle. The disconnect function will
// reset the configuration each time it's being called.
void TangoData::Disconnect() { TangoService_disconnect(); }
