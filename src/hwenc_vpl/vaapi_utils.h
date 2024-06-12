// https://github.com/Intel-Media-SDK/MediaSDK/blob/master/samples/sample_common/include/vaapi_utils.h より。
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

#ifndef VAAPI_UTILS_H_
#define VAAPI_UTILS_H_

// oneVPL
#include <vpl/mfxdefs.h>

// libva
#include <va/va.h>

class CLibVA {
 public:
  virtual ~CLibVA(void) {}

  VADisplay GetVADisplay() { return m_va_dpy; }

 protected:
  CLibVA() : m_va_dpy(NULL) {}
  VADisplay m_va_dpy;

 private:
  CLibVA(CLibVA&&) = delete;
  CLibVA(const CLibVA&) = delete;
  CLibVA& operator=(CLibVA&&) = delete;
  CLibVA& operator=(const CLibVA&) = delete;
};

mfxStatus va_to_mfx_status(VAStatus va_res);

#endif  // VAAPI_UTILS_H_