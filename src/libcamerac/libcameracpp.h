#ifndef LIBCAMERACPP_H_
#define LIBCAMERACPP_H_

#include <memory>

#include "libcamerac.h"

static std::shared_ptr<libcamerac_CameraManager>
libcameracpp_CameraManager_new() {
  return std::shared_ptr<libcamerac_CameraManager>(
      libcamerac_CameraManager_new(), libcamerac_CameraManager_destroy);
}

static std::shared_ptr<libcamerac_vector_Camera>
libcameracpp_CameraManager_cameras(libcamerac_CameraManager* p) {
  return std::shared_ptr<libcamerac_vector_Camera>(
      libcamerac_CameraManager_cameras(p), libcamerac_vector_Camera_destroy);
}

static std::shared_ptr<libcamerac_Camera> libcameracpp_CameraManager_get(
    libcamerac_CameraManager* p,
    const char* id) {
  return std::shared_ptr<libcamerac_Camera>(libcamerac_CameraManager_get(p, id),
                                            libcamerac_Camera_disown);
}

static std::shared_ptr<libcamerac_vector_StreamRole>
libcameracpp_vector_StreamRole_new() {
  return std::shared_ptr<libcamerac_vector_StreamRole>(
      libcamerac_vector_StreamRole_new(), libcamerac_vector_StreamRole_destroy);
}

static std::shared_ptr<libcamerac_Camera> libcameracpp_Camera_own(
    libcamerac_Camera* p) {
  return std::shared_ptr<libcamerac_Camera>(libcamerac_Camera_own(p),
                                            libcamerac_Camera_disown);
}

static std::shared_ptr<libcamerac_CameraConfiguration>
libcameracpp_Camera_generateConfiguration(libcamerac_Camera* p,
                                          libcamerac_vector_StreamRole* roles) {
  return std::shared_ptr<libcamerac_CameraConfiguration>(
      libcamerac_Camera_generateConfiguration(p, roles),
      libcamerac_CameraConfiguration_destroy);
}

static std::shared_ptr<libcamerac_Request> libcameracpp_Camera_createRequest(
    libcamerac_Camera* p) {
  return std::shared_ptr<libcamerac_Request>(libcamerac_Camera_createRequest(p),
                                             libcamerac_Request_destroy);
}

static std::shared_ptr<libcamerac_FrameBufferAllocator>
libcameracpp_FrameBufferAllocator_new(libcamerac_Camera* camera) {
  return std::shared_ptr<libcamerac_FrameBufferAllocator>(
      libcamerac_FrameBufferAllocator_new(camera),
      libcamerac_FrameBufferAllocator_destroy);
}

static std::shared_ptr<libcamerac_ControlList>
libcameracpp_ControlList_controls() {
  return std::shared_ptr<libcamerac_ControlList>(
      libcamerac_ControlList_controls(), libcamerac_ControlList_destroy);
}
#endif