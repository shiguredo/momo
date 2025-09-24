/*
 * This copyright notice applies to this header file only:
 *
 * Copyright (c) 2010-2024 NVIDIA Corporation
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the software, and to permit persons to whom the
 * software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "NvEncoder/NvEncoder.h"

#ifndef _WIN32
#include <cstring>
static inline bool operator==(const GUID &guid1, const GUID &guid2) {
    return !memcmp(&guid1, &guid2, sizeof(GUID));
}

static inline bool operator!=(const GUID &guid1, const GUID &guid2) {
    return !(guid1 == guid2);
}
#endif

NvEncoder::NvEncoder(NV_ENC_DEVICE_TYPE eDeviceType, void *pDevice, uint32_t nWidth, uint32_t nHeight, NV_ENC_BUFFER_FORMAT eBufferFormat,
                            uint32_t nExtraOutputDelay, bool bMotionEstimationOnly, bool bOutputInVideoMemory, bool bDX12Encode, bool bUseIVFContainer) :
    m_pDevice(pDevice), 
    m_eDeviceType(eDeviceType),
    m_nWidth(nWidth),
    m_nHeight(nHeight),
    m_nMaxEncodeWidth(nWidth),
    m_nMaxEncodeHeight(nHeight),
    m_eBufferFormat(eBufferFormat), 
    m_bMotionEstimationOnly(bMotionEstimationOnly), 
    m_bOutputInVideoMemory(bOutputInVideoMemory),
    m_bIsDX12Encode(bDX12Encode),
    m_bUseIVFContainer(bUseIVFContainer),
    m_nExtraOutputDelay(nExtraOutputDelay), 
    m_hEncoder(nullptr)
{
    LoadNvEncApi();

    if (!m_nvenc.nvEncOpenEncodeSession) 
    {
        m_nEncoderBuffer = 0;
        NVENC_THROW_ERROR("EncodeAPI not found", NV_ENC_ERR_NO_ENCODE_DEVICE);
    }

    NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS encodeSessionExParams = { NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER };
    encodeSessionExParams.device = m_pDevice;
    encodeSessionExParams.deviceType = m_eDeviceType;
    encodeSessionExParams.apiVersion = NVENCAPI_VERSION;
    void *hEncoder = NULL;
    NVENC_API_CALL(m_nvenc.nvEncOpenEncodeSessionEx(&encodeSessionExParams, &hEncoder));
    m_hEncoder = hEncoder;
}

void NvEncoder::TryLoadNvEncApi()
{
    #if defined(_WIN32)
    #if defined(_WIN64)
      HMODULE hModule = LoadLibrary(TEXT("nvEncodeAPI64.dll"));
    #else
      HMODULE hModule = LoadLibrary(TEXT("nvEncodeAPI.dll"));
    #endif
    #else
      void* hModule = dlopen("libnvidia-encode.so.1", RTLD_LAZY);
    #endif
    
      if (hModule == NULL) {
        NVENC_THROW_ERROR(
            "NVENC library file is not found. Please ensure NV driver is installed",
            NV_ENC_ERR_NO_ENCODE_DEVICE);
      }
    
      typedef NVENCSTATUS(NVENCAPI *
                          NvEncodeAPIGetMaxSupportedVersion_Type)(uint32_t*);
    #if defined(_WIN32)
      NvEncodeAPIGetMaxSupportedVersion_Type NvEncodeAPIGetMaxSupportedVersion =
          (NvEncodeAPIGetMaxSupportedVersion_Type)GetProcAddress(
              hModule, "NvEncodeAPIGetMaxSupportedVersion");
    #else
      NvEncodeAPIGetMaxSupportedVersion_Type NvEncodeAPIGetMaxSupportedVersion =
          (NvEncodeAPIGetMaxSupportedVersion_Type)dlsym(
              hModule, "NvEncodeAPIGetMaxSupportedVersion");
    #endif
      if (NvEncodeAPIGetMaxSupportedVersion == NULL) {
        NVENC_THROW_ERROR(
            "NvEncodeAPIGetMaxSupportedVersion function is not exported",
            NV_ENC_ERR_NO_ENCODE_DEVICE);
      }
    
      uint32_t version = 0;
      uint32_t currentVersion =
          (NVENCAPI_MAJOR_VERSION << 4) | NVENCAPI_MINOR_VERSION;
      NVENC_API_CALL(NvEncodeAPIGetMaxSupportedVersion(&version));
      if (currentVersion > version) {
    #if defined(_WIN32)
        FreeLibrary((HMODULE)hModule);
    #else
        dlclose(hModule);
    #endif
        NVENC_THROW_ERROR(
            "Current Driver Version does not support this NvEncodeAPI version, "
            "please upgrade driver",
            NV_ENC_ERR_INVALID_VERSION);
      }
    #if defined(_WIN32)
      FreeLibrary((HMODULE)hModule);
    #else
      dlclose(hModule);
    #endif
}

void NvEncoder::LoadNvEncApi()
{

    #if defined(_WIN32)
    #if defined(_WIN64)
      HMODULE hModule = LoadLibrary(TEXT("nvEncodeAPI64.dll"));
    #else
      HMODULE hModule = LoadLibrary(TEXT("nvEncodeAPI.dll"));
    #endif
    #else
      void* hModule = dlopen("libnvidia-encode.so.1", RTLD_LAZY);
    #endif
    
      if (hModule == NULL) {
        NVENC_THROW_ERROR(
            "NVENC library file is not found. Please ensure NV driver is installed",
            NV_ENC_ERR_NO_ENCODE_DEVICE);
      }
    
      m_hModule = hModule;

      typedef NVENCSTATUS(NVENCAPI *
                          NvEncodeAPIGetMaxSupportedVersion_Type)(uint32_t*);
    #if defined(_WIN32)
      NvEncodeAPIGetMaxSupportedVersion_Type NvEncodeAPIGetMaxSupportedVersion =
          (NvEncodeAPIGetMaxSupportedVersion_Type)GetProcAddress(
              hModule, "NvEncodeAPIGetMaxSupportedVersion");
    #else
      NvEncodeAPIGetMaxSupportedVersion_Type NvEncodeAPIGetMaxSupportedVersion =
          (NvEncodeAPIGetMaxSupportedVersion_Type)dlsym(
              hModule, "NvEncodeAPIGetMaxSupportedVersion");
    #endif
      if (NvEncodeAPIGetMaxSupportedVersion == NULL) {
        NVENC_THROW_ERROR(
            "NvEncodeAPIGetMaxSupportedVersion function is not exported",
            NV_ENC_ERR_NO_ENCODE_DEVICE);
      }

    uint32_t version = 0;
    uint32_t currentVersion = (NVENCAPI_MAJOR_VERSION << 4) | NVENCAPI_MINOR_VERSION;
    NVENC_API_CALL(NvEncodeAPIGetMaxSupportedVersion(&version));
    if (currentVersion > version)
    {
        NVENC_THROW_ERROR("Current Driver Version does not support this NvEncodeAPI version, please upgrade driver", NV_ENC_ERR_INVALID_VERSION);
    }

    typedef NVENCSTATUS(NVENCAPI *NvEncodeAPICreateInstance_Type)(NV_ENCODE_API_FUNCTION_LIST*);
    #if defined(_WIN32)
      NvEncodeAPICreateInstance_Type NvEncodeAPICreateInstance =
          (NvEncodeAPICreateInstance_Type)GetProcAddress(hModule, "NvEncodeAPICreateInstance");
    #else
      NvEncodeAPICreateInstance_Type NvEncodeAPICreateInstance =
          (NvEncodeAPICreateInstance_Type)dlsym(hModule, "NvEncodeAPICreateInstance");
    #endif
    
      if (!NvEncodeAPICreateInstance) {
        NVENC_THROW_ERROR(
            "Cannot find NvEncodeAPICreateInstance() entry in NVENC library",
            NV_ENC_ERR_NO_ENCODE_DEVICE);
      }

    m_nvenc = { NV_ENCODE_API_FUNCTION_LIST_VER };
    NVENC_API_CALL(NvEncodeAPICreateInstance(&m_nvenc));
}

NvEncoder::~NvEncoder()
{
    DestroyHWEncoder();

    if (m_hModule)
    {
        #if defined(_WIN32)
            FreeLibrary((HMODULE)m_hModule);
        #else
            dlclose(m_hModule);
        #endif
            m_hModule = nullptr;
    }
}

void NvEncoder::CreateDefaultEncoderParams(NV_ENC_INITIALIZE_PARAMS* pIntializeParams, GUID codecGuid, GUID presetGuid, NV_ENC_TUNING_INFO tuningInfo)
{
    if (!m_hEncoder)
    {
        NVENC_THROW_ERROR("Encoder Initialization failed", NV_ENC_ERR_NO_ENCODE_DEVICE);
        return;
    }

    if (pIntializeParams == nullptr || pIntializeParams->encodeConfig == nullptr)
    {
        NVENC_THROW_ERROR("pInitializeParams and pInitializeParams->encodeConfig can't be NULL", NV_ENC_ERR_INVALID_PTR);
    }

    memset(pIntializeParams->encodeConfig, 0, sizeof(NV_ENC_CONFIG));
    auto pEncodeConfig = pIntializeParams->encodeConfig;
    memset(pIntializeParams, 0, sizeof(NV_ENC_INITIALIZE_PARAMS));
    pIntializeParams->encodeConfig = pEncodeConfig;


    pIntializeParams->encodeConfig->version = NV_ENC_CONFIG_VER;
    pIntializeParams->version = NV_ENC_INITIALIZE_PARAMS_VER;

    pIntializeParams->encodeGUID = codecGuid;
    pIntializeParams->presetGUID = presetGuid;
    pIntializeParams->encodeWidth = m_nWidth;
    pIntializeParams->encodeHeight = m_nHeight;
    pIntializeParams->darWidth = m_nWidth;
    pIntializeParams->darHeight = m_nHeight;
    pIntializeParams->frameRateNum = 30;
    pIntializeParams->frameRateDen = 1;
    pIntializeParams->enablePTD = 1;
    pIntializeParams->reportSliceOffsets = 0;
    pIntializeParams->enableSubFrameWrite = 0;
    pIntializeParams->maxEncodeWidth = m_nWidth;
    pIntializeParams->maxEncodeHeight = m_nHeight;
    pIntializeParams->enableMEOnlyMode = m_bMotionEstimationOnly;
    pIntializeParams->enableOutputInVidmem = m_bOutputInVideoMemory;
#if defined(_WIN32)
    if (!m_bOutputInVideoMemory)
    {
        pIntializeParams->enableEncodeAsync = GetCapabilityValue(codecGuid, NV_ENC_CAPS_ASYNC_ENCODE_SUPPORT);
    }
#endif

    pIntializeParams->tuningInfo = tuningInfo;
    pIntializeParams->encodeConfig->rcParams.rateControlMode = NV_ENC_PARAMS_RC_CONSTQP;

    //There are changes in the structure layout, therefore users are recommended to be careful while moving their application to the new header. 
    //Following initialization has changed for the same reason.
    NV_ENC_PRESET_CONFIG presetConfig = { NV_ENC_PRESET_CONFIG_VER, 0, { NV_ENC_CONFIG_VER } };
    m_nvenc.nvEncGetEncodePresetConfigEx(m_hEncoder, codecGuid, presetGuid, tuningInfo, &presetConfig);
    memcpy(pIntializeParams->encodeConfig, &presetConfig.presetCfg, sizeof(NV_ENC_CONFIG));


    if(m_bMotionEstimationOnly)
    {
        m_encodeConfig.version = NV_ENC_CONFIG_VER;
        m_encodeConfig.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CONSTQP;
        m_encodeConfig.rcParams.constQP = { 28, 31, 25 };
    }
    
    if (pIntializeParams->encodeGUID == NV_ENC_CODEC_H264_GUID)
    {
        if (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444_10BIT)
        {
            pIntializeParams->encodeConfig->encodeCodecConfig.h264Config.chromaFormatIDC = 3;
        }
        else if (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_NV16 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_P210)
        {
            pIntializeParams->encodeConfig->encodeCodecConfig.h264Config.chromaFormatIDC = 2;
        }
        pIntializeParams->encodeConfig->encodeCodecConfig.h264Config.idrPeriod = pIntializeParams->encodeConfig->gopLength;
    }
    else if (pIntializeParams->encodeGUID == NV_ENC_CODEC_HEVC_GUID)
    {
        pIntializeParams->encodeConfig->encodeCodecConfig.hevcConfig.inputBitDepth = pIntializeParams->encodeConfig->encodeCodecConfig.hevcConfig.outputBitDepth =
            (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV420_10BIT || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444_10BIT ) ? NV_ENC_BIT_DEPTH_10 : NV_ENC_BIT_DEPTH_8;
        if (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444_10BIT)
        {
            pIntializeParams->encodeConfig->encodeCodecConfig.hevcConfig.chromaFormatIDC = 3;
        }
        else if (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_NV16 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_P210)
        {
            pIntializeParams->encodeConfig->encodeCodecConfig.hevcConfig.chromaFormatIDC = 2;
        }
        pIntializeParams->encodeConfig->encodeCodecConfig.hevcConfig.idrPeriod = pIntializeParams->encodeConfig->gopLength;
    }
    else if (pIntializeParams->encodeGUID == NV_ENC_CODEC_AV1_GUID)
    {
		pIntializeParams->encodeConfig->encodeCodecConfig.av1Config.inputBitDepth = (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV420_10BIT) ? NV_ENC_BIT_DEPTH_10 : NV_ENC_BIT_DEPTH_8;
        pIntializeParams->encodeConfig->encodeCodecConfig.av1Config.chromaFormatIDC = 1;
        pIntializeParams->encodeConfig->encodeCodecConfig.av1Config.idrPeriod = pIntializeParams->encodeConfig->gopLength;
        if (m_bOutputInVideoMemory)
        {
            pIntializeParams->encodeConfig->frameIntervalP = 1;
        }
    }

    if (m_bIsDX12Encode)
    {
        pIntializeParams->bufferFormat = m_eBufferFormat;
    }
    
    return;
}

void NvEncoder::CreateEncoder(const NV_ENC_INITIALIZE_PARAMS* pEncoderParams)
{
    if (!m_hEncoder)
    {
        NVENC_THROW_ERROR("Encoder Initialization failed", NV_ENC_ERR_NO_ENCODE_DEVICE);
    }

    if (!pEncoderParams)
    {
        NVENC_THROW_ERROR("Invalid NV_ENC_INITIALIZE_PARAMS ptr", NV_ENC_ERR_INVALID_PTR);
    }

    if (pEncoderParams->encodeWidth == 0 || pEncoderParams->encodeHeight == 0)
    {
        NVENC_THROW_ERROR("Invalid encoder width and height", NV_ENC_ERR_INVALID_PARAM);
    }

    if (pEncoderParams->encodeGUID != NV_ENC_CODEC_H264_GUID && pEncoderParams->encodeGUID != NV_ENC_CODEC_HEVC_GUID && pEncoderParams->encodeGUID != NV_ENC_CODEC_AV1_GUID)
    {
        NVENC_THROW_ERROR("Invalid codec guid", NV_ENC_ERR_INVALID_PARAM);
    }

    if (pEncoderParams->encodeGUID == NV_ENC_CODEC_AV1_GUID)
    {
        if (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444_10BIT)
        {
            NVENC_THROW_ERROR("YUV444 format isn't supported by AV1 encoder", NV_ENC_ERR_INVALID_PARAM);
        }
    }

    // set other necessary params if not set yet
    if (pEncoderParams->encodeGUID == NV_ENC_CODEC_H264_GUID)
    {
        bool yuv10BitFormat = (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV420_10BIT || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_P210 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444_10BIT) ? true : false;
        if (yuv10BitFormat && pEncoderParams->encodeConfig->encodeCodecConfig.h264Config.inputBitDepth != NV_ENC_BIT_DEPTH_10)
        {
            NVENC_THROW_ERROR("Invalid PixelBitdepth", NV_ENC_ERR_INVALID_PARAM);
        }

        if ((m_eBufferFormat == NV_ENC_BUFFER_FORMAT_NV16 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_P210) &&
            (pEncoderParams->encodeConfig->encodeCodecConfig.h264Config.chromaFormatIDC != 2))
        {
            NVENC_THROW_ERROR("Invalid ChromaFormatIDC", NV_ENC_ERR_INVALID_PARAM);
        }

        if ((m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444) &&
            (pEncoderParams->encodeConfig->encodeCodecConfig.h264Config.chromaFormatIDC != 3))
        {
            NVENC_THROW_ERROR("Invalid ChromaFormatIDC", NV_ENC_ERR_INVALID_PARAM);
        }
    }

    if (pEncoderParams->encodeGUID == NV_ENC_CODEC_HEVC_GUID)
    {
        bool yuv10BitFormat = (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV420_10BIT || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_P210 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444_10BIT) ? true : false;
        if (yuv10BitFormat && pEncoderParams->encodeConfig->encodeCodecConfig.hevcConfig.inputBitDepth != NV_ENC_BIT_DEPTH_10)
        {
            NVENC_THROW_ERROR("Invalid PixelBitdepth", NV_ENC_ERR_INVALID_PARAM);
        }

        if ((m_eBufferFormat == NV_ENC_BUFFER_FORMAT_NV16 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_P210) &&
            (pEncoderParams->encodeConfig->encodeCodecConfig.hevcConfig.chromaFormatIDC != 2))
        {
            NVENC_THROW_ERROR("Invalid ChromaFormatIDC", NV_ENC_ERR_INVALID_PARAM);
        }

        if ((m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444 || m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV444_10BIT) &&
            (pEncoderParams->encodeConfig->encodeCodecConfig.hevcConfig.chromaFormatIDC != 3))
        {
            NVENC_THROW_ERROR("Invalid ChromaFormatIDC", NV_ENC_ERR_INVALID_PARAM);
        }
    }

    if (pEncoderParams->encodeGUID == NV_ENC_CODEC_AV1_GUID)
    {
        bool yuv10BitFormat = (m_eBufferFormat == NV_ENC_BUFFER_FORMAT_YUV420_10BIT) ? true : false;
        if (yuv10BitFormat && pEncoderParams->encodeConfig->encodeCodecConfig.av1Config.inputBitDepth != NV_ENC_BIT_DEPTH_10)
        {
            NVENC_THROW_ERROR("Invalid PixelBitdepth", NV_ENC_ERR_INVALID_PARAM);
        }

        if (pEncoderParams->encodeConfig->encodeCodecConfig.av1Config.chromaFormatIDC != 1)
        {
            NVENC_THROW_ERROR("Invalid ChromaFormatIDC", NV_ENC_ERR_INVALID_PARAM);
        }

        if (m_bOutputInVideoMemory && pEncoderParams->encodeConfig->frameIntervalP > 1)
        {
            NVENC_THROW_ERROR("Alt Ref frames not supported for AV1 in case of OutputInVideoMemory", NV_ENC_ERR_INVALID_PARAM);
        }
    }

    memcpy(&m_initializeParams, pEncoderParams, sizeof(m_initializeParams));
    m_initializeParams.version = NV_ENC_INITIALIZE_PARAMS_VER;

    if (pEncoderParams->encodeConfig)
    {
        memcpy(&m_encodeConfig, pEncoderParams->encodeConfig, sizeof(m_encodeConfig));
        m_encodeConfig.version = NV_ENC_CONFIG_VER;
    }
    else
    {
        //There are changes in the structure layout, therefore users are recommended to be careful while moving their application to the new header. 
        //Following initialization has changed for the same reason.
        NV_ENC_PRESET_CONFIG presetConfig = { NV_ENC_PRESET_CONFIG_VER, 0, { NV_ENC_CONFIG_VER } };
        if (!m_bMotionEstimationOnly)
        {
            m_nvenc.nvEncGetEncodePresetConfigEx(m_hEncoder, pEncoderParams->encodeGUID, pEncoderParams->presetGUID, pEncoderParams->tuningInfo, &presetConfig);
            memcpy(&m_encodeConfig, &presetConfig.presetCfg, sizeof(NV_ENC_CONFIG));
            if (m_bOutputInVideoMemory && pEncoderParams->encodeGUID == NV_ENC_CODEC_AV1_GUID)
            {
                m_encodeConfig.frameIntervalP = 1;
            }
        }
        else
        {
            m_encodeConfig.version = NV_ENC_CONFIG_VER;
            m_encodeConfig.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CONSTQP;
            m_encodeConfig.rcParams.constQP = { 28, 31, 25 };
        }
    }

    if (((uint32_t)m_encodeConfig.frameIntervalP) > m_encodeConfig.gopLength)
    {
        m_encodeConfig.frameIntervalP = m_encodeConfig.gopLength;
    }

    m_initializeParams.encodeConfig = &m_encodeConfig;

    NVENC_API_CALL(m_nvenc.nvEncInitializeEncoder(m_hEncoder, &m_initializeParams));

    m_bEncoderInitialized = true;
    m_nWidth = m_initializeParams.encodeWidth;
    m_nHeight = m_initializeParams.encodeHeight;
    m_nMaxEncodeWidth = m_initializeParams.maxEncodeWidth;
    m_nMaxEncodeHeight = m_initializeParams.maxEncodeHeight;

    m_nEncoderBuffer = m_encodeConfig.frameIntervalP + m_encodeConfig.rcParams.lookaheadDepth + m_nExtraOutputDelay;

    if (pEncoderParams->encodeGUID == NV_ENC_CODEC_HEVC_GUID)
    {
        m_enableStereoMVHEVC = m_initializeParams.encodeConfig->encodeCodecConfig.hevcConfig.enableMVHEVC;
        if (m_enableStereoMVHEVC)
        {
            m_nEncoderBuffer = m_nEncoderBuffer * 2;
            m_outputHevc3DReferenceDisplayInfo = m_initializeParams.encodeConfig->encodeCodecConfig.hevcConfig.outputHevc3DReferenceDisplayInfo;
        }
    }

    m_nOutputDelay = m_nEncoderBuffer - 1;

    if (!m_bOutputInVideoMemory)
    {
        m_vpCompletionEvent.resize(m_nEncoderBuffer, nullptr);
    }

#if defined(_WIN32)
    for (uint32_t i = 0; i < m_vpCompletionEvent.size(); i++) 
    {
        m_vpCompletionEvent[i] = CreateEvent(NULL, FALSE, FALSE, NULL);
        if (!m_bIsDX12Encode)
        {
            NV_ENC_EVENT_PARAMS eventParams = { NV_ENC_EVENT_PARAMS_VER };
            eventParams.completionEvent = m_vpCompletionEvent[i];
            m_nvenc.nvEncRegisterAsyncEvent(m_hEncoder, &eventParams);
        }
    }
#endif

    m_vMappedInputBuffers.resize(m_nEncoderBuffer, nullptr);

    if (m_bMotionEstimationOnly)
    {
        m_vMappedRefBuffers.resize(m_nEncoderBuffer, nullptr);

        if (!m_bOutputInVideoMemory)
        {
            InitializeMVOutputBuffer();
        }
    }
    else
    {
        if (!m_bOutputInVideoMemory && !m_bIsDX12Encode)
        {
            m_vBitstreamOutputBuffer.resize(m_nEncoderBuffer, nullptr);
            InitializeBitstreamBuffer();
        }
    }

    AllocateInputBuffers(m_nEncoderBuffer);
}

void NvEncoder::DestroyEncoder()
{
    if (!m_hEncoder)
    {
        return;
    }

    ReleaseInputBuffers();

    DestroyHWEncoder();
}

void NvEncoder::DestroyHWEncoder()
{
    if (!m_hEncoder)
    {
        return;
    }

#if defined(_WIN32)
    for (uint32_t i = 0; i < m_vpCompletionEvent.size(); i++)
    {
        if (m_vpCompletionEvent[i])
        {
            if (!m_bIsDX12Encode)
            {
                NV_ENC_EVENT_PARAMS eventParams = { NV_ENC_EVENT_PARAMS_VER };
                eventParams.completionEvent = m_vpCompletionEvent[i];
                m_nvenc.nvEncUnregisterAsyncEvent(m_hEncoder, &eventParams);
            }
            CloseHandle(m_vpCompletionEvent[i]);
        }
    }
    m_vpCompletionEvent.clear();
#endif

    if (m_bMotionEstimationOnly)
    {
        DestroyMVOutputBuffer();
    }
    else
    {
        if (!m_bIsDX12Encode)
            DestroyBitstreamBuffer();
    }

    m_nvenc.nvEncDestroyEncoder(m_hEncoder);

    m_hEncoder = nullptr;

    m_bEncoderInitialized = false;
}

const NvEncInputFrame* NvEncoder::GetNextInputFrame()
{
    int i = m_iToSend % m_nEncoderBuffer;
    return &m_vInputFrames[i];
}

const NvEncInputFrame* NvEncoder::GetNextInputFrame(uint32_t frameIdx) // for external lookahead API in Iterative Encoder sample (AppEncQual)
{
    int i = frameIdx % m_nEncoderBuffer;
    return &m_vInputFrames[i];
}

const NvEncInputFrame* NvEncoder::GetNextReferenceFrame()
{
    int i = m_iToSend % m_nEncoderBuffer;
    return &m_vReferenceFrames[i];
}

void NvEncoder::MapResources(uint32_t bfrIdx)
{
    NV_ENC_MAP_INPUT_RESOURCE mapInputResource = { NV_ENC_MAP_INPUT_RESOURCE_VER };

    mapInputResource.registeredResource = m_vRegisteredResources[bfrIdx];
    NVENC_API_CALL(m_nvenc.nvEncMapInputResource(m_hEncoder, &mapInputResource));
    m_vMappedInputBuffers[bfrIdx] = mapInputResource.mappedResource;

    if (m_bMotionEstimationOnly)
    {
        mapInputResource.registeredResource = m_vRegisteredResourcesForReference[bfrIdx];
        NVENC_API_CALL(m_nvenc.nvEncMapInputResource(m_hEncoder, &mapInputResource));
        m_vMappedRefBuffers[bfrIdx] = mapInputResource.mappedResource;
    }
}

void NvEncoder::EncodeFrame(std::vector<NvEncOutputFrame> &vPacket, NV_ENC_PIC_PARAMS *pPicParams)
{
    vPacket.clear();
    if (!IsHWEncoderInitialized())
    {
        NVENC_THROW_ERROR("Encoder device not found", NV_ENC_ERR_NO_ENCODE_DEVICE);
    }

    int bfrIdx = m_iToSend % m_nEncoderBuffer;

    MapResources(bfrIdx);

    NVENCSTATUS nvStatus = DoEncode(m_vMappedInputBuffers[bfrIdx], m_vBitstreamOutputBuffer[bfrIdx], pPicParams);

    if (nvStatus == NV_ENC_SUCCESS || nvStatus == NV_ENC_ERR_NEED_MORE_INPUT)
    {
        m_iToSend++;
        GetEncodedPacket(m_vBitstreamOutputBuffer, vPacket, true);
    }
    else
    {
        NVENC_THROW_ERROR("nvEncEncodePicture API failed", nvStatus);
    }
}

void NvEncoder::RunMotionEstimation(std::vector<uint8_t> &mvData)
{
    if (!m_hEncoder)
    {
        NVENC_THROW_ERROR("Encoder Initialization failed", NV_ENC_ERR_NO_ENCODE_DEVICE);
        return;
    }

    const uint32_t bfrIdx = m_iToSend % m_nEncoderBuffer;

    MapResources(bfrIdx);

    NVENCSTATUS nvStatus = DoMotionEstimation(m_vMappedInputBuffers[bfrIdx], m_vMappedRefBuffers[bfrIdx], m_vMVDataOutputBuffer[bfrIdx]);

    if (nvStatus == NV_ENC_SUCCESS)
    {
        m_iToSend++;
        std::vector<NvEncOutputFrame> vPacket;
        GetEncodedPacket(m_vMVDataOutputBuffer, vPacket, true);
        if (vPacket.size() != 1)
        {
            NVENC_THROW_ERROR("GetEncodedPacket() doesn't return one (and only one) MVData", NV_ENC_ERR_GENERIC);
        }
        mvData = vPacket[0].frame;
    }
    else
    {
        NVENC_THROW_ERROR("nvEncEncodePicture API failed", nvStatus);
    }
}


void NvEncoder::GetSequenceParams(std::vector<uint8_t> &seqParams)
{
    uint8_t spsppsData[1024]; // Assume maximum spspps data is 1KB or less
    memset(spsppsData, 0, sizeof(spsppsData));
    NV_ENC_SEQUENCE_PARAM_PAYLOAD payload = { NV_ENC_SEQUENCE_PARAM_PAYLOAD_VER };
    uint32_t spsppsSize = 0;

    payload.spsppsBuffer = spsppsData;
    payload.inBufferSize = sizeof(spsppsData);
    payload.outSPSPPSPayloadSize = &spsppsSize;
    NVENC_API_CALL(m_nvenc.nvEncGetSequenceParams(m_hEncoder, &payload));
    seqParams.clear();
    seqParams.insert(seqParams.end(), &spsppsData[0], &spsppsData[spsppsSize]);
}

NVENCSTATUS NvEncoder::DoEncode(NV_ENC_INPUT_PTR inputBuffer, NV_ENC_OUTPUT_PTR outputBuffer, NV_ENC_PIC_PARAMS *pPicParams)
{
    NV_ENC_PIC_PARAMS picParams = {};
    if (pPicParams)
    {
        picParams = *pPicParams;
    }

    HEVC_3D_REFERENCE_DISPLAY_INFO *p3dReferenceDisplayInfo = NULL;
    picParams.version = NV_ENC_PIC_PARAMS_VER;
    picParams.pictureStruct = NV_ENC_PIC_STRUCT_FRAME;
    picParams.inputTimeStamp = m_nInputTimeStamp++;
    picParams.inputBuffer = inputBuffer;
    picParams.bufferFmt = GetPixelFormat();
    picParams.inputWidth = GetEncodeWidth();
    picParams.inputHeight = GetEncodeHeight();
    picParams.frameIdx = m_iToSend;
    picParams.outputBitstream = outputBuffer;
    picParams.completionEvent = GetCompletionEvent(m_iToSend % m_nEncoderBuffer);

    if (m_enableStereoMVHEVC)
    {
        picParams.codecPicParams.hevcPicParams.viewId = m_viewId;
        m_viewId = m_viewId ^ 1;
        if (m_outputHevc3DReferenceDisplayInfo == 1)
        {

            // Example code to insert HEVC_3D_REFERENCE_DISPLAY_INFO
            // One 3D Reference Displays Information SEI message NAL unit (per G.14.2.3), with
            // nuh_layer_id equal to 0, and with the following values:
            // - To indicate the display width value is unspecified by using 0, use:
            //   - prec_ref_display_width = 31
            //   - exponent_ref_display_width = 0
            //   - mantissa_ref_display_width = 0
            // - ref_viewing_distance_flag = 0
            // - prec_ref_viewing_dist syntax element skipped
            // - num_ref_displays_minusl = 0
            // - left_view_id[ 0 ] is set to reference the texture layer used for the viewer_s left
            //   eye using values in vps_extension (per F.7.3.2.1.1)
            // - right_view_id[ 0 ] is set to reference the texture layer used for the viewer_s
            //   right eye using values in vps_extension (per F.7.3.2.1.1)
            // - additional_shift_present_flag[ 0 ] = 0
            // - exponent_ref_viewing_distancel 0 ], mantissa_ref_viewing_distance[ 0 ],
            //   num_sample_shift_plus512[ 0 ] syntax elements are skipped
            p3dReferenceDisplayInfo = new HEVC_3D_REFERENCE_DISPLAY_INFO;
            memset(p3dReferenceDisplayInfo, 0, sizeof(HEVC_3D_REFERENCE_DISPLAY_INFO));
            p3dReferenceDisplayInfo->precRefDisplayWidth = 31;
            p3dReferenceDisplayInfo->leftViewId[0] = 0;
            p3dReferenceDisplayInfo->rightViewId[0] = 1;
            picParams.codecPicParams.hevcPicParams.p3DReferenceDisplayInfo = p3dReferenceDisplayInfo;
        }
    }

    NVENCSTATUS nvStatus = m_nvenc.nvEncEncodePicture(m_hEncoder, &picParams);

    if (p3dReferenceDisplayInfo)
    {
        delete p3dReferenceDisplayInfo;
        p3dReferenceDisplayInfo = NULL;
    }

    return nvStatus; 
}

void NvEncoder::SendEOS()
{
    NV_ENC_PIC_PARAMS picParams = { NV_ENC_PIC_PARAMS_VER };
    picParams.encodePicFlags = NV_ENC_PIC_FLAG_EOS;
    picParams.completionEvent = GetCompletionEvent(m_iToSend % m_nEncoderBuffer);
    NVENC_API_CALL(m_nvenc.nvEncEncodePicture(m_hEncoder, &picParams));
}

void NvEncoder::EndEncode(std::vector<NvEncOutputFrame> &vPacket)
{
    vPacket.clear();
    if (!IsHWEncoderInitialized())
    {
        NVENC_THROW_ERROR("Encoder device not initialized", NV_ENC_ERR_ENCODER_NOT_INITIALIZED);
    }

    SendEOS();

    GetEncodedPacket(m_vBitstreamOutputBuffer, vPacket, false);
}

void NvEncoder::GetEncodedPacket(std::vector<NV_ENC_OUTPUT_PTR> &vOutputBuffer, std::vector<NvEncOutputFrame> &vPacket, bool bOutputDelay)
{
    unsigned i = 0;
    int iEnd = bOutputDelay ? m_iToSend - m_nOutputDelay : m_iToSend;
    for (; m_iGot < iEnd; m_iGot++)
    {
        WaitForCompletionEvent(m_iGot % m_nEncoderBuffer);
        NV_ENC_LOCK_BITSTREAM lockBitstreamData = { NV_ENC_LOCK_BITSTREAM_VER };
        lockBitstreamData.outputBitstream = vOutputBuffer[m_iGot % m_nEncoderBuffer];
        lockBitstreamData.doNotWait = false;
        NVENC_API_CALL(m_nvenc.nvEncLockBitstream(m_hEncoder, &lockBitstreamData));
  
        uint8_t *pData = (uint8_t *)lockBitstreamData.bitstreamBufferPtr;
        if (vPacket.size() < i + 1)
        {
            NvEncOutputFrame nvEncOutputFrame;
            vPacket.push_back(nvEncOutputFrame);
        }
        vPacket[i].frame.clear();
       
        if ((m_initializeParams.encodeGUID == NV_ENC_CODEC_AV1_GUID) && (m_bUseIVFContainer))
        {
            if (m_bWriteIVFFileHeader)
            {
                m_IVFUtils.WriteFileHeader(vPacket[i].frame, MAKE_FOURCC('A', 'V', '0', '1'), m_initializeParams.encodeWidth, m_initializeParams.encodeHeight, m_initializeParams.frameRateNum, m_initializeParams.frameRateDen, 0xFFFF);
                m_bWriteIVFFileHeader = false;
            }

            m_IVFUtils.WriteFrameHeader(vPacket[i].frame, lockBitstreamData.bitstreamSizeInBytes, lockBitstreamData.outputTimeStamp);
        }
        vPacket[i].frame.insert(vPacket[i].frame.end(), &pData[0], &pData[lockBitstreamData.bitstreamSizeInBytes]);
        vPacket[i].pictureType = lockBitstreamData.pictureType;
        vPacket[i].timeStamp = lockBitstreamData.outputTimeStamp;
        i++;

        NVENC_API_CALL(m_nvenc.nvEncUnlockBitstream(m_hEncoder, lockBitstreamData.outputBitstream));

        if (m_vMappedInputBuffers[m_iGot % m_nEncoderBuffer])
        {
            NVENC_API_CALL(m_nvenc.nvEncUnmapInputResource(m_hEncoder, m_vMappedInputBuffers[m_iGot % m_nEncoderBuffer]));
            m_vMappedInputBuffers[m_iGot % m_nEncoderBuffer] = nullptr;
        }

        if (m_bMotionEstimationOnly && m_vMappedRefBuffers[m_iGot % m_nEncoderBuffer])
        {
            NVENC_API_CALL(m_nvenc.nvEncUnmapInputResource(m_hEncoder, m_vMappedRefBuffers[m_iGot % m_nEncoderBuffer]));
            m_vMappedRefBuffers[m_iGot % m_nEncoderBuffer] = nullptr;
        }
    }
}

bool NvEncoder::Reconfigure(const NV_ENC_RECONFIGURE_PARAMS *pReconfigureParams)
{
    NVENC_API_CALL(m_nvenc.nvEncReconfigureEncoder(m_hEncoder, const_cast<NV_ENC_RECONFIGURE_PARAMS*>(pReconfigureParams)));

    memcpy(&m_initializeParams, &(pReconfigureParams->reInitEncodeParams), sizeof(m_initializeParams));
    if (pReconfigureParams->reInitEncodeParams.encodeConfig)
    {
        memcpy(&m_encodeConfig, pReconfigureParams->reInitEncodeParams.encodeConfig, sizeof(m_encodeConfig));
    }

    m_nWidth = m_initializeParams.encodeWidth;
    m_nHeight = m_initializeParams.encodeHeight;
    m_nMaxEncodeWidth = m_initializeParams.maxEncodeWidth;
    m_nMaxEncodeHeight = m_initializeParams.maxEncodeHeight;

    return true;
}

NV_ENC_REGISTERED_PTR NvEncoder::RegisterResource(void *pBuffer, NV_ENC_INPUT_RESOURCE_TYPE eResourceType,
    int width, int height, int pitch, NV_ENC_BUFFER_FORMAT bufferFormat, NV_ENC_BUFFER_USAGE bufferUsage, 
    NV_ENC_FENCE_POINT_D3D12* pInputFencePoint)
{
    NV_ENC_REGISTER_RESOURCE registerResource = { NV_ENC_REGISTER_RESOURCE_VER };
    registerResource.resourceType = eResourceType;
    registerResource.resourceToRegister = pBuffer;
    registerResource.width = width;
    registerResource.height = height;
    registerResource.pitch = pitch;
    registerResource.bufferFormat = bufferFormat;
    registerResource.bufferUsage = bufferUsage;
    registerResource.pInputFencePoint = pInputFencePoint;
    NVENC_API_CALL(m_nvenc.nvEncRegisterResource(m_hEncoder, &registerResource));

    return registerResource.registeredResource;
}

void NvEncoder::RegisterInputResources(std::vector<void*> inputframes, NV_ENC_INPUT_RESOURCE_TYPE eResourceType,
                                         int width, int height, int pitch, NV_ENC_BUFFER_FORMAT bufferFormat, bool bReferenceFrame)
{
    for (uint32_t i = 0; i < inputframes.size(); ++i)
    {
        NV_ENC_REGISTERED_PTR registeredPtr = RegisterResource(inputframes[i], eResourceType, width, height, pitch, bufferFormat, NV_ENC_INPUT_IMAGE);
        
        std::vector<uint32_t> _chromaOffsets;
        NvEncoder::GetChromaSubPlaneOffsets(bufferFormat, pitch, height, _chromaOffsets);
        NvEncInputFrame inputframe = {};
        inputframe.inputPtr = (void *)inputframes[i];
        inputframe.chromaOffsets[0] = 0;
        inputframe.chromaOffsets[1] = 0;
        for (uint32_t ch = 0; ch < _chromaOffsets.size(); ch++)
        {
            inputframe.chromaOffsets[ch] = _chromaOffsets[ch];
        }
        inputframe.numChromaPlanes = NvEncoder::GetNumChromaPlanes(bufferFormat);
        inputframe.pitch = pitch;
        inputframe.chromaPitch = NvEncoder::GetChromaPitch(bufferFormat, pitch);
        inputframe.bufferFormat = bufferFormat;
        inputframe.resourceType = eResourceType;

        if (bReferenceFrame)
        {
            m_vRegisteredResourcesForReference.push_back(registeredPtr);
            m_vReferenceFrames.push_back(inputframe);
        }
        else
        {
            m_vRegisteredResources.push_back(registeredPtr);
            m_vInputFrames.push_back(inputframe);
        }
    }
}

void NvEncoder::FlushEncoder()
{
    if (!m_bMotionEstimationOnly && !m_bOutputInVideoMemory)
    {
        // Incase of error it is possible for buffers still mapped to encoder.
        // flush the encoder queue and then unmapped it if any surface is still mapped
        try
        {
            std::vector<NvEncOutputFrame> vPacket;
            EndEncode(vPacket);
        }
        catch (...)
        {

        }
    }
}

void NvEncoder::UnregisterInputResources()
{
    FlushEncoder();
    
    if (m_bMotionEstimationOnly)
    {
        for (uint32_t i = 0; i < m_vMappedRefBuffers.size(); ++i)
        {
            if (m_vMappedRefBuffers[i])
            {
                m_nvenc.nvEncUnmapInputResource(m_hEncoder, m_vMappedRefBuffers[i]);
            }
        }
    }
    m_vMappedRefBuffers.clear();

    for (uint32_t i = 0; i < m_vMappedInputBuffers.size(); ++i)
    {
        if (m_vMappedInputBuffers[i])
        {
            m_nvenc.nvEncUnmapInputResource(m_hEncoder, m_vMappedInputBuffers[i]);
        }
    }
    m_vMappedInputBuffers.clear();

    for (uint32_t i = 0; i < m_vRegisteredResources.size(); ++i)
    {
        if (m_vRegisteredResources[i])
        {
            m_nvenc.nvEncUnregisterResource(m_hEncoder, m_vRegisteredResources[i]);
        }
    }
    m_vRegisteredResources.clear();


    for (uint32_t i = 0; i < m_vRegisteredResourcesForReference.size(); ++i)
    {
        if (m_vRegisteredResourcesForReference[i])
        {
            m_nvenc.nvEncUnregisterResource(m_hEncoder, m_vRegisteredResourcesForReference[i]);
        }
    }
    m_vRegisteredResourcesForReference.clear();

}


void NvEncoder::WaitForCompletionEvent(int iEvent)
{
#if defined(_WIN32)
    // Check if we are in async mode. If not, don't wait for event;
    NV_ENC_CONFIG sEncodeConfig = { 0 };
    NV_ENC_INITIALIZE_PARAMS sInitializeParams = { 0 };
    sInitializeParams.encodeConfig = &sEncodeConfig;
    GetInitializeParams(&sInitializeParams);

    if (0U == sInitializeParams.enableEncodeAsync)
    {
        return;
    }
#ifdef DEBUG
    WaitForSingleObject(m_vpCompletionEvent[iEvent], INFINITE);
#else
    // wait for 20s which is infinite on terms of gpu time
    if (WaitForSingleObject(m_vpCompletionEvent[iEvent], 20000) == WAIT_FAILED)
    {
        NVENC_THROW_ERROR("Failed to encode frame", NV_ENC_ERR_GENERIC);
    }
#endif
#endif
}

uint32_t NvEncoder::GetWidthInBytes(const NV_ENC_BUFFER_FORMAT bufferFormat, const uint32_t width)
{
    switch (bufferFormat) {
    case NV_ENC_BUFFER_FORMAT_NV12:
    case NV_ENC_BUFFER_FORMAT_YV12:
    case NV_ENC_BUFFER_FORMAT_IYUV:
    case NV_ENC_BUFFER_FORMAT_NV16:
    case NV_ENC_BUFFER_FORMAT_YUV444:
        return width;
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
    case NV_ENC_BUFFER_FORMAT_P210:
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        return width * 2;
    case NV_ENC_BUFFER_FORMAT_ARGB:
    case NV_ENC_BUFFER_FORMAT_ARGB10:
    case NV_ENC_BUFFER_FORMAT_AYUV:
    case NV_ENC_BUFFER_FORMAT_ABGR:
    case NV_ENC_BUFFER_FORMAT_ABGR10:
        return width * 4;
    default:
        NVENC_THROW_ERROR("Invalid Buffer format", NV_ENC_ERR_INVALID_PARAM);
        return 0;
    }
}

uint32_t NvEncoder::GetNumChromaPlanes(const NV_ENC_BUFFER_FORMAT bufferFormat)
{
    switch (bufferFormat) 
    {
    case NV_ENC_BUFFER_FORMAT_NV12:
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
    case NV_ENC_BUFFER_FORMAT_NV16:
    case NV_ENC_BUFFER_FORMAT_P210:
        return 1;
    case NV_ENC_BUFFER_FORMAT_YV12:
    case NV_ENC_BUFFER_FORMAT_IYUV:
    case NV_ENC_BUFFER_FORMAT_YUV444:
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        return 2;
    case NV_ENC_BUFFER_FORMAT_ARGB:
    case NV_ENC_BUFFER_FORMAT_ARGB10:
    case NV_ENC_BUFFER_FORMAT_AYUV:
    case NV_ENC_BUFFER_FORMAT_ABGR:
    case NV_ENC_BUFFER_FORMAT_ABGR10:
        return 0;
    default:
        NVENC_THROW_ERROR("Invalid Buffer format", NV_ENC_ERR_INVALID_PARAM);
        return -1;
    }
}

uint32_t NvEncoder::GetChromaPitch(const NV_ENC_BUFFER_FORMAT bufferFormat,const uint32_t lumaPitch)
{
    switch (bufferFormat)
    {
    case NV_ENC_BUFFER_FORMAT_NV12:
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
    case NV_ENC_BUFFER_FORMAT_NV16:
    case NV_ENC_BUFFER_FORMAT_P210:
    case NV_ENC_BUFFER_FORMAT_YUV444:
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        return lumaPitch;
    case NV_ENC_BUFFER_FORMAT_YV12:
    case NV_ENC_BUFFER_FORMAT_IYUV:
        return (lumaPitch + 1)/2;
    case NV_ENC_BUFFER_FORMAT_ARGB:
    case NV_ENC_BUFFER_FORMAT_ARGB10:
    case NV_ENC_BUFFER_FORMAT_AYUV:
    case NV_ENC_BUFFER_FORMAT_ABGR:
    case NV_ENC_BUFFER_FORMAT_ABGR10:
        return 0;
    default:
        NVENC_THROW_ERROR("Invalid Buffer format", NV_ENC_ERR_INVALID_PARAM);
        return -1;
    }
}

void NvEncoder::GetChromaSubPlaneOffsets(const NV_ENC_BUFFER_FORMAT bufferFormat, const uint32_t pitch, const uint32_t height, std::vector<uint32_t>& chromaOffsets)
{
    chromaOffsets.clear();
    switch (bufferFormat)
    {
    case NV_ENC_BUFFER_FORMAT_NV12:
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
    case NV_ENC_BUFFER_FORMAT_NV16:
    case NV_ENC_BUFFER_FORMAT_P210:
        chromaOffsets.push_back(pitch * height);
        return;
    case NV_ENC_BUFFER_FORMAT_YV12:
    case NV_ENC_BUFFER_FORMAT_IYUV:
        chromaOffsets.push_back(pitch * height);
        chromaOffsets.push_back(chromaOffsets[0] + (NvEncoder::GetChromaPitch(bufferFormat, pitch) * GetChromaHeight(bufferFormat, height)));
        return;
    case NV_ENC_BUFFER_FORMAT_YUV444:
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        chromaOffsets.push_back(pitch * height);
        chromaOffsets.push_back(chromaOffsets[0] + (pitch * height));
        return;
    case NV_ENC_BUFFER_FORMAT_ARGB:
    case NV_ENC_BUFFER_FORMAT_ARGB10:
    case NV_ENC_BUFFER_FORMAT_AYUV:
    case NV_ENC_BUFFER_FORMAT_ABGR:
    case NV_ENC_BUFFER_FORMAT_ABGR10:
        return;
    default:
        NVENC_THROW_ERROR("Invalid Buffer format", NV_ENC_ERR_INVALID_PARAM);
        return;
    }
}

uint32_t NvEncoder::GetChromaHeight(const NV_ENC_BUFFER_FORMAT bufferFormat, const uint32_t lumaHeight)
{
    switch (bufferFormat)
    {
    case NV_ENC_BUFFER_FORMAT_YV12:
    case NV_ENC_BUFFER_FORMAT_IYUV:
    case NV_ENC_BUFFER_FORMAT_NV12:
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
        return (lumaHeight + 1)/2;
    case NV_ENC_BUFFER_FORMAT_NV16:
    case NV_ENC_BUFFER_FORMAT_P210:
        return lumaHeight;
    case NV_ENC_BUFFER_FORMAT_YUV444:
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        return lumaHeight;
    case NV_ENC_BUFFER_FORMAT_ARGB:
    case NV_ENC_BUFFER_FORMAT_ARGB10:
    case NV_ENC_BUFFER_FORMAT_AYUV:
    case NV_ENC_BUFFER_FORMAT_ABGR:
    case NV_ENC_BUFFER_FORMAT_ABGR10:
        return 0;
    default:
        NVENC_THROW_ERROR("Invalid Buffer format", NV_ENC_ERR_INVALID_PARAM);
        return 0;
    }
}

uint32_t NvEncoder::GetChromaWidthInBytes(const NV_ENC_BUFFER_FORMAT bufferFormat, const uint32_t lumaWidth)
{
    switch (bufferFormat)
    {
    case NV_ENC_BUFFER_FORMAT_YV12:
    case NV_ENC_BUFFER_FORMAT_IYUV:
        return (lumaWidth + 1) / 2;
    case NV_ENC_BUFFER_FORMAT_NV12:
        return lumaWidth;
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
        return 2 * lumaWidth;
    case NV_ENC_BUFFER_FORMAT_NV16:
        return lumaWidth;
    case NV_ENC_BUFFER_FORMAT_P210:
        return 2 * lumaWidth;
    case NV_ENC_BUFFER_FORMAT_YUV444:
        return lumaWidth;
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        return 2 * lumaWidth;
    case NV_ENC_BUFFER_FORMAT_ARGB:
    case NV_ENC_BUFFER_FORMAT_ARGB10:
    case NV_ENC_BUFFER_FORMAT_AYUV:
    case NV_ENC_BUFFER_FORMAT_ABGR:
    case NV_ENC_BUFFER_FORMAT_ABGR10:
        return 0;
    default:
        NVENC_THROW_ERROR("Invalid Buffer format", NV_ENC_ERR_INVALID_PARAM);
        return 0;
    }
}


int NvEncoder::GetCapabilityValue(GUID guidCodec, NV_ENC_CAPS capsToQuery)
{
    if (!m_hEncoder)
    {
        return 0;
    }
    NV_ENC_CAPS_PARAM capsParam = { NV_ENC_CAPS_PARAM_VER };
    capsParam.capsToQuery = capsToQuery;
    int v;
    m_nvenc.nvEncGetEncodeCaps(m_hEncoder, guidCodec, &capsParam, &v);
    return v;
}

int NvEncoder::GetFrameSize() const
{
    switch (GetPixelFormat())
    {
    case NV_ENC_BUFFER_FORMAT_YV12:
    case NV_ENC_BUFFER_FORMAT_IYUV:
    case NV_ENC_BUFFER_FORMAT_NV12:
        return GetEncodeWidth() * (GetEncodeHeight() + (GetEncodeHeight() + 1) / 2);
    case NV_ENC_BUFFER_FORMAT_YUV420_10BIT:
        return 2 * GetEncodeWidth() * (GetEncodeHeight() + (GetEncodeHeight() + 1) / 2);
    case NV_ENC_BUFFER_FORMAT_NV16:
        return 2 * GetEncodeWidth() * GetEncodeHeight();
    case NV_ENC_BUFFER_FORMAT_P210:
        return 4 * GetEncodeWidth() * GetEncodeHeight();
    case NV_ENC_BUFFER_FORMAT_YUV444:
        return GetEncodeWidth() * GetEncodeHeight() * 3;
    case NV_ENC_BUFFER_FORMAT_YUV444_10BIT:
        return 2 * GetEncodeWidth() * GetEncodeHeight() * 3;
    case NV_ENC_BUFFER_FORMAT_ARGB:
    case NV_ENC_BUFFER_FORMAT_ARGB10:
    case NV_ENC_BUFFER_FORMAT_AYUV:
    case NV_ENC_BUFFER_FORMAT_ABGR:
    case NV_ENC_BUFFER_FORMAT_ABGR10:
        return 4 * GetEncodeWidth() * GetEncodeHeight();
    default:
        NVENC_THROW_ERROR("Invalid Buffer format", NV_ENC_ERR_INVALID_PARAM);
        return 0;
    }
}

void NvEncoder::GetInitializeParams(NV_ENC_INITIALIZE_PARAMS *pInitializeParams)
{
    if (!pInitializeParams || !pInitializeParams->encodeConfig)
    {
        NVENC_THROW_ERROR("Both pInitializeParams and pInitializeParams->encodeConfig can't be NULL", NV_ENC_ERR_INVALID_PTR);
    }
    NV_ENC_CONFIG *pEncodeConfig = pInitializeParams->encodeConfig;
    *pEncodeConfig = m_encodeConfig;
    *pInitializeParams = m_initializeParams;
    pInitializeParams->encodeConfig = pEncodeConfig;
}

void NvEncoder::InitializeBitstreamBuffer()
{
    for (int i = 0; i < m_nEncoderBuffer; i++)
    {
        NV_ENC_CREATE_BITSTREAM_BUFFER createBitstreamBuffer = { NV_ENC_CREATE_BITSTREAM_BUFFER_VER };
        NVENC_API_CALL(m_nvenc.nvEncCreateBitstreamBuffer(m_hEncoder, &createBitstreamBuffer));
        m_vBitstreamOutputBuffer[i] = createBitstreamBuffer.bitstreamBuffer;
    }
}

void NvEncoder::DestroyBitstreamBuffer()
{
    for (uint32_t i = 0; i < m_vBitstreamOutputBuffer.size(); i++)
    {
        if (m_vBitstreamOutputBuffer[i])
        {
            m_nvenc.nvEncDestroyBitstreamBuffer(m_hEncoder, m_vBitstreamOutputBuffer[i]);
        }
    }

    m_vBitstreamOutputBuffer.clear();
}

void NvEncoder::InitializeMVOutputBuffer()
{
    for (int i = 0; i < m_nEncoderBuffer; i++)
    {
        NV_ENC_CREATE_MV_BUFFER createMVBuffer = { NV_ENC_CREATE_MV_BUFFER_VER };
        NVENC_API_CALL(m_nvenc.nvEncCreateMVBuffer(m_hEncoder, &createMVBuffer));
        m_vMVDataOutputBuffer.push_back(createMVBuffer.mvBuffer);
    }
}

void NvEncoder::DestroyMVOutputBuffer()
{
    for (uint32_t i = 0; i < m_vMVDataOutputBuffer.size(); i++)
    {
        if (m_vMVDataOutputBuffer[i])
        {
            m_nvenc.nvEncDestroyMVBuffer(m_hEncoder, m_vMVDataOutputBuffer[i]);
        }
    }

    m_vMVDataOutputBuffer.clear();
}

NVENCSTATUS NvEncoder::DoMotionEstimation(NV_ENC_INPUT_PTR inputBuffer, NV_ENC_INPUT_PTR inputBufferForReference, NV_ENC_OUTPUT_PTR outputBuffer)
{
    NV_ENC_MEONLY_PARAMS meParams = { NV_ENC_MEONLY_PARAMS_VER };
    meParams.inputBuffer = inputBuffer;
    meParams.referenceFrame = inputBufferForReference;
    meParams.inputWidth = GetEncodeWidth();
    meParams.inputHeight = GetEncodeHeight();
    meParams.mvBuffer = outputBuffer;
    meParams.completionEvent = GetCompletionEvent(m_iToSend % m_nEncoderBuffer);
    NVENCSTATUS nvStatus = m_nvenc.nvEncRunMotionEstimationOnly(m_hEncoder, &meParams);
    
    return nvStatus;
}
