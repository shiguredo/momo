// https://github.com/Intel-Media-SDK/MediaSDK/blob/master/samples/sample_common/include/vaapi_utils_drm.h より。
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

#ifndef VAAPI_UTILS_DRM_H_
#define VAAPI_UTILS_DRM_H_

#include <memory>
#include <string>

#include "vaapi_utils.h"

class DRMLibVA : public CLibVA {
 public:
  DRMLibVA(const std::string& devicePath);
  virtual ~DRMLibVA(void);

  inline int getFD() { return m_fd; }

 protected:
  int m_fd;

 private:
  DRMLibVA(DRMLibVA&&) = delete;
  DRMLibVA(const DRMLibVA&) = delete;
  DRMLibVA& operator=(DRMLibVA&&) = delete;
  DRMLibVA& operator=(const DRMLibVA&) = delete;
};

std::unique_ptr<DRMLibVA> CreateDRMLibVA(const std::string& devicePath = "");

#endif  // VAAPI_UTILS_DRM_H_