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

#ifndef VIDEO_OVERLAY_JNI_EXAMPLE_EXPERIMENTAL_TANGO_DATA_H_
#define VIDEO_OVERLAY_JNI_EXAMPLE_EXPERIMENTAL_TANGO_DATA_H_
#define GLM_FORCE_RADIANS

#include <sys/time.h>
#include <tango_client_api.h>

#include "cinder/gl/gl.h"
const int kVersionStringLength = 27;

class CinderTango {
 public:
  static CinderTango& GetInstance() {
    static CinderTango instance;
    return instance;
  }
  CinderTango();
  ~CinderTango();

  TangoErrorType Initialize(JNIEnv* env, jobject activity);
  bool SetConfig(bool is_auto_recovery);
  bool Connect();
  void Disconnect();
  bool GetPoseAtTime();
  bool GetIntrinsics();
  bool GetExtrinsics();

  void ConnectTexture(GLuint texture_id);
  void UpdateColorTexture();
  void ResetMotionTracking();

  const char* getStatusStringFromStatusCode(TangoPoseStatusType status);

  
  pthread_mutex_t pose_mutex;
  pthread_mutex_t event_mutex;

  glm::vec3 tango_position;
  glm::quat tango_rotation;

  int status_count[3];
  std::string event_string;
  std::string lib_version_string;
  std::string pose_string;

  // Extrinsics for imu_T_device (position and hamilton quaternion).
  glm::vec3 imu_p_device;
  glm::quat imu_q_device;

  // Extrinsics for imu_T_color_camera (position and hamilton quaternion).
  glm::vec3 imu_p_cc;
  glm::quat imu_q_cc;

  // Intrinsics for color camera.
  int cc_width;
  int cc_height;
  double cc_fx;
  double cc_fy;
  double cc_cx;
  double cc_cy;
  double cc_distortion[5];

  // Localization status.
  bool is_localized;
  std::string cur_uuid;


 private:
  TangoConfig config_;
  double timestamp;
};

#endif  // VIDEO_OVERLAY_JNI_EXAMPLE_EXPERIMENTAL_TANGO_DATA_H_
