/*
* Copyright 2017-2020 NVIDIA Corporation.  All rights reserved.
*
* Please refer to the NVIDIA end user license agreement (EULA) associated
* with this source code for terms and conditions that govern your use of
* this software. Any use, reproduction, disclosure, or distribution of
* this software and related documentation outside the terms of the EULA
* is strictly prohibited.
*
*/

#pragma once

#include <assert.h>
#include <stdint.h>
#include <mutex>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <string.h>
#include "../Utils/NvCodecUtils.h"
#include "dyn/nvcuvid.h"

/**
* @brief Exception class for error reporting from the decode API.
*/
class NVDECException : public std::exception
{
public:
    NVDECException(const std::string& errorStr, const CUresult errorCode)
        : m_errorString(errorStr), m_errorCode(errorCode) {}

    virtual ~NVDECException() throw() {}
    virtual const char* what() const throw() { return m_errorString.c_str(); }
    CUresult  getErrorCode() const { return m_errorCode; }
    const std::string& getErrorString() const { return m_errorString; }
    static NVDECException makeNVDECException(const std::string& errorStr, const CUresult errorCode,
        const std::string& functionName, const std::string& fileName, int lineNo);
private:
    std::string m_errorString;
    CUresult m_errorCode;
};

inline NVDECException NVDECException::makeNVDECException(const std::string& errorStr, const CUresult errorCode, const std::string& functionName,
    const std::string& fileName, int lineNo)
{
    std::ostringstream errorLog;
    errorLog << functionName << " : " << errorStr << " at " << fileName << ":" << lineNo << std::endl;
    NVDECException exception(errorLog.str(), errorCode);
    return exception;
}

#define NVDEC_THROW_ERROR( errorStr, errorCode )                                                         \
    do                                                                                                   \
    {                                                                                                    \
        throw NVDECException::makeNVDECException(errorStr, errorCode, __FUNCTION__, __FILE__, __LINE__); \
    } while (0)


#define NVDEC_API_CALL( cuvidAPI )                                                                                 \
    do                                                                                                             \
    {                                                                                                              \
        CUresult errorCode = cuvidAPI;                                                                             \
        if( errorCode != CUDA_SUCCESS)                                                                             \
        {                                                                                                          \
            std::ostringstream errorLog;                                                                           \
            errorLog << #cuvidAPI << " returned error " << errorCode;                                              \
            throw NVDECException::makeNVDECException(errorLog.str(), errorCode, __FUNCTION__, __FILE__, __LINE__); \
        }                                                                                                          \
    } while (0)

struct Rect {
    int l, t, r, b;
};

struct Dim {
    int w, h;
};

/**
* @brief Base class for decoder interface.
*/
class NvDecoder {

public:
    /**
    *  @brief This function is used to initialize the decoder session.
    *  Application must call this function to initialize the decoder, before
    *  starting to decode any frames.
    */
    NvDecoder(CUcontext cuContext, bool bUseDeviceFrame, cudaVideoCodec eCodec, bool bLowLatency = false,
              bool bDeviceFramePitched = false, const Rect *pCropRect = NULL, const Dim *pResizeDim = NULL,
              int maxWidth = 0, int maxHeight = 0, unsigned int clkRate = 1000);
    ~NvDecoder();

    /**
    *  @brief  This function is used to get the current CUDA context.
    */
    CUcontext GetContext() { return m_cuContext; }

    /**
    *  @brief  This function is used to get the current decode width.
    */
    int GetWidth() { assert(m_nWidth); return m_nWidth; }

    /**
    *  @brief  This function is used to get the current decode height (Luma height).
    */
    int GetHeight() { assert(m_nLumaHeight); return m_nLumaHeight; }

    /**
    *  @brief  This function is used to get the current chroma height.
    */
    int GetChromaHeight() { assert(m_nChromaHeight); return m_nChromaHeight; }

    /**
    *  @brief  This function is used to get the number of chroma planes.
    */
    int GetNumChromaPlanes() { assert(m_nNumChromaPlanes); return m_nNumChromaPlanes; }
    
    /**
    *   @brief  This function is used to get the current frame size based on pixel format.
    */
    int GetFrameSize() { assert(m_nWidth); return m_nWidth * (m_nLumaHeight + m_nChromaHeight * m_nNumChromaPlanes) * m_nBPP; }

    /**
    *  @brief  This function is used to get the pitch of the device buffer holding the decoded frame.
    */
    int GetDeviceFramePitch() { assert(m_nWidth); return m_nDeviceFramePitch ? (int)m_nDeviceFramePitch : m_nWidth * m_nBPP; }

    /**
    *   @brief  This function is used to get the bit depth associated with the pixel format.
    */
    int GetBitDepth() { assert(m_nWidth); return m_nBitDepthMinus8 + 8; }

    /**
    *   @brief  This function is used to get the bytes used per pixel.
    */
    int GetBPP() { assert(m_nWidth); return m_nBPP; }

    /**
    *   @brief  This function is used to get the YUV chroma format
    */
    cudaVideoSurfaceFormat GetOutputFormat() { return m_eOutputFormat; }

    /**
    *   @brief  This function is used to get information about the video stream (codec, display parameters etc)
    */
    CUVIDEOFORMAT GetVideoFormatInfo() { assert(m_nWidth); return m_videoFormat; }

    /**
    *   @brief  This function is used to get codec string from codec id
    */
    const char *GetCodecString(cudaVideoCodec eCodec);

    /**
    *   @brief  This function is used to print information about the video stream
    */
    std::string GetVideoInfo() const { return m_videoInfo.str(); }

    /**
    *   @brief  This function decodes a frame and returns the number of frames that are available for
    *   display. All frames that are available for display should be read before making a subsequent decode call.
    *   @param  pData - pointer to the data buffer that is to be decoded
    *   @param  nSize - size of the data buffer in bytes
    *   @param  nFlags - CUvideopacketflags for setting decode options
    *   @param  nTimestamp - presentation timestamp
    */
    int Decode(const uint8_t *pData, int nSize, int nFlags = 0, int64_t nTimestamp = 0);

    /**
    *   @brief  This function returns a decoded frame and timestamp. This function should be called in a loop for
    *   fetching all the frames that are available for display.
    */
    uint8_t* GetFrame(int64_t* pTimestamp = nullptr);


    /**
    *   @brief  This function decodes a frame and returns the locked frame buffers
    *   This makes the buffers available for use by the application without the buffers
    *   getting overwritten, even if subsequent decode calls are made. The frame buffers
    *   remain locked, until UnlockFrame() is called
    */
    uint8_t* GetLockedFrame(int64_t* pTimestamp = nullptr);

    /**
    *   @brief  This function unlocks the frame buffer and makes the frame buffers available for write again
    *   @param  ppFrame - pointer to array of frames that are to be unlocked	
    *   @param  nFrame - number of frames to be unlocked
    */
    void UnlockFrame(uint8_t **pFrame);

    /**
    *   @brief  This function allow app to set decoder reconfig params
    *   @param  pCropRect - cropping rectangle coordinates
    *   @param  pResizeDim - width and height of resized output
    */
    int setReconfigParams(const Rect * pCropRect, const Dim * pResizeDim);

    // start a timer
    void   startTimer() { m_stDecode_time.Start(); }

    // stop the timer
    double stopTimer() { return m_stDecode_time.Stop(); }
private:
    /**
    *   @brief  Callback function to be registered for getting a callback when decoding of sequence starts
    */
    static int CUDAAPI HandleVideoSequenceProc(void *pUserData, CUVIDEOFORMAT *pVideoFormat) { return ((NvDecoder *)pUserData)->HandleVideoSequence(pVideoFormat); }

    /**
    *   @brief  Callback function to be registered for getting a callback when a decoded frame is ready to be decoded
    */
    static int CUDAAPI HandlePictureDecodeProc(void *pUserData, CUVIDPICPARAMS *pPicParams) { return ((NvDecoder *)pUserData)->HandlePictureDecode(pPicParams); }

    /**
    *   @brief  Callback function to be registered for getting a callback when a decoded frame is available for display
    */
    static int CUDAAPI HandlePictureDisplayProc(void *pUserData, CUVIDPARSERDISPINFO *pDispInfo) { return ((NvDecoder *)pUserData)->HandlePictureDisplay(pDispInfo); }

    /**
    *   @brief  This function gets called when a sequence is ready to be decoded. The function also gets called
        when there is format change
    */
    int HandleVideoSequence(CUVIDEOFORMAT *pVideoFormat);

    /**
    *   @brief  This function gets called when a picture is ready to be decoded. cuvidDecodePicture is called from this function
    *   to decode the picture
    */
    int HandlePictureDecode(CUVIDPICPARAMS *pPicParams);

    /**
    *   @brief  This function gets called after a picture is decoded and available for display. Frames are fetched and stored in 
        internal buffer
    */
    int HandlePictureDisplay(CUVIDPARSERDISPINFO *pDispInfo);

    /**
    *   @brief  This function reconfigure decoder if there is a change in sequence params.
    */
    int ReconfigureDecoder(CUVIDEOFORMAT *pVideoFormat);

private:
    CUcontext m_cuContext = NULL;
    CUvideoctxlock m_ctxLock;
    CUvideoparser m_hParser = NULL;
    CUvideodecoder m_hDecoder = NULL;
    bool m_bUseDeviceFrame;
    // dimension of the output
    unsigned int m_nWidth = 0, m_nLumaHeight = 0, m_nChromaHeight = 0;
    unsigned int m_nNumChromaPlanes = 0;
    // height of the mapped surface 
    int m_nSurfaceHeight = 0;
    int m_nSurfaceWidth = 0;
    cudaVideoCodec m_eCodec = cudaVideoCodec_NumCodecs;
    cudaVideoChromaFormat m_eChromaFormat;
    cudaVideoSurfaceFormat m_eOutputFormat;
    int m_nBitDepthMinus8 = 0;
    int m_nBPP = 1;
    CUVIDEOFORMAT m_videoFormat = {};
    Rect m_displayRect = {};
    // stock of frames
    std::vector<uint8_t *> m_vpFrame;
    // timestamps of decoded frames
    std::vector<int64_t> m_vTimestamp;
    int m_nDecodedFrame = 0, m_nDecodedFrameReturned = 0;
    int m_nDecodePicCnt = 0, m_nPicNumInDecodeOrder[32];
    bool m_bEndDecodeDone = false;
    std::mutex m_mtxVPFrame;
    int m_nFrameAlloc = 0;
    CUstream m_cuvidStream = 0;
    bool m_bDeviceFramePitched = false;
    size_t m_nDeviceFramePitch = 0;
    Rect m_cropRect = {};
    Dim m_resizeDim = {};

    std::ostringstream m_videoInfo;
    unsigned int m_nMaxWidth = 0, m_nMaxHeight = 0;
    bool m_bReconfigExternal = false;
    bool m_bReconfigExtPPChange = false;
    StopWatch m_stDecode_time;
};
