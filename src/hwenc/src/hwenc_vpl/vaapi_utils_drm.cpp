// https://github.com/Intel-Media-SDK/MediaSDK/blob/master/samples/sample_common/src/vaapi_utils_drm.cpp より。
// オリジナルのライセンスは以下。
/******************************************************************************\
Copyright (c) 2005-2019, Intel Corporation
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
This sample was distributed or derived from the Intel's Media Samples package.
The original version of this sample may be obtained from https://software.intel.com/en-us/intel-media-server-studio
or https://software.intel.com/en-us/media-client-solutions-support.
\**********************************************************************************/
#include "vaapi_utils_drm.h"

// Linux
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

// WebRTC
#include <rtc_base/logging.h>

// DRM
#include <drm/drm.h>

// libva
#include <va/va_drm.h>

namespace sora {

constexpr mfxU32 MFX_DRI_MAX_NODES_NUM = 16;
constexpr mfxU32 MFX_DRI_RENDER_START_INDEX = 128;
constexpr mfxU32 MFX_DRI_CARD_START_INDEX = 0;
constexpr mfxU32 MFX_DRM_DRIVER_NAME_LEN = 4;
const char* MFX_DRM_INTEL_DRIVER_NAME = "i915";
const char* MFX_DRI_PATH = "/dev/dri/";
const char* MFX_DRI_NODE_RENDER = "renderD";
const char* MFX_DRI_NODE_CARD = "card";

int get_drm_driver_name(int fd, char* name, int name_size) {
  drm_version_t version = {};
  version.name_len = name_size;
  version.name = name;
  return ioctl(fd, DRM_IOWR(0, drm_version), &version);
}

int open_first_intel_adapter() {
  std::string adapterPath = MFX_DRI_PATH;
  char driverName[MFX_DRM_DRIVER_NAME_LEN + 1] = {};
  mfxU32 nodeIndex;

  adapterPath += MFX_DRI_NODE_RENDER;
  nodeIndex = MFX_DRI_RENDER_START_INDEX;

  for (mfxU32 i = 0; i < MFX_DRI_MAX_NODES_NUM; ++i) {
    std::string curAdapterPath = adapterPath + std::to_string(nodeIndex + i);

    int fd = open(curAdapterPath.c_str(), O_RDWR);
    if (fd < 0)
      continue;

    if (!get_drm_driver_name(fd, driverName, MFX_DRM_DRIVER_NAME_LEN) &&
        !strcmp(driverName, MFX_DRM_INTEL_DRIVER_NAME)) {
      return fd;
    }
    close(fd);
  }

  return -1;
}

int open_intel_adapter(const std::string& devicePath) {
  if (devicePath.empty())
    return open_first_intel_adapter();

  int fd = open(devicePath.c_str(), O_RDWR);

  if (fd < 0) {
    RTC_LOG(LS_ERROR) << "Failed to open specified device";
    return -1;
  }

  char driverName[MFX_DRM_DRIVER_NAME_LEN + 1] = {};
  if (!get_drm_driver_name(fd, driverName, MFX_DRM_DRIVER_NAME_LEN) &&
      !strcmp(driverName, MFX_DRM_INTEL_DRIVER_NAME)) {
    return fd;
  } else {
    close(fd);
    RTC_LOG(LS_ERROR) << "Specified device is not Intel one";
    return -1;
  }
}

DRMLibVA::DRMLibVA(const std::string& devicePath) : CLibVA(), m_fd(-1) {
  mfxStatus sts = MFX_ERR_NONE;

  m_fd = open_intel_adapter(devicePath);
  if (m_fd < 0)
    throw std::range_error("Intel GPU was not found");

  m_va_dpy = vaGetDisplayDRM(m_fd);
  if (m_va_dpy) {
    int major_version = 0, minor_version = 0;
    VAStatus va_res = vaInitialize(m_va_dpy, &major_version, &minor_version);
    sts = va_to_mfx_status(va_res);
  } else {
    sts = MFX_ERR_NULL_PTR;
  }

  if (MFX_ERR_NONE != sts) {
    if (m_va_dpy)
      vaTerminate(m_va_dpy);
    close(m_fd);
    throw std::runtime_error("Loading of VA display was failed");
  }
}

DRMLibVA::~DRMLibVA(void) {
  if (m_va_dpy) {
    vaTerminate(m_va_dpy);
  }
  if (m_fd >= 0) {
    close(m_fd);
  }
}

std::unique_ptr<DRMLibVA> CreateDRMLibVA(const std::string& devicePath) {
  try {
    return std::unique_ptr<DRMLibVA>(new DRMLibVA(devicePath));
  } catch (std::exception& e) {
    return nullptr;
  }
}

}  // namespace sora
