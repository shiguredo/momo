#ifndef LIBCAMERAC_H_
#define LIBCAMERAC_H_

// libcamera を C API から使うためのヘッダ

#ifdef __cplusplus
extern "C" {
#endif

struct libcamerac_CameraManager_t;
typedef struct libcamerac_CameraManager_t libcamerac_CameraManager;

struct libcamerac_Camera_t;
typedef struct libcamerac_Camera_t libcamerac_Camera;

struct libcamerac_vector_Camera_t;
typedef struct libcamerac_vector_Camera_t libcamerac_vector_Camera;

struct libcamerac_CameraConfiguration_t;
typedef struct libcamerac_CameraConfiguration_t libcamerac_CameraConfiguration;

struct libcamerac_StreamConfiguration_t;
typedef struct libcamerac_StreamConfiguration_t libcamerac_StreamConfiguration;

struct libcamerac_Stream_t;
typedef struct libcamerac_Stream_t libcamerac_Stream;

struct libcamerac_FrameBufferAllocator_t;
typedef struct libcamerac_FrameBufferAllocator_t
    libcamerac_FrameBufferAllocator;

struct libcamerac_FrameBuffer_t;
typedef struct libcamerac_FrameBuffer_t libcamerac_FrameBuffer;

struct libcamerac_FrameBuffer_Plane_t;
typedef struct libcamerac_FrameBuffer_Plane_t libcamerac_FrameBuffer_Plane;

struct libcamerac_Request_t;
typedef struct libcamerac_Request_t libcamerac_Request;

typedef enum {
  libcamerac_Request_Status_RequestPending,
  libcamerac_Request_Status_RequestComplete,
  libcamerac_Request_Status_RequestCancelled,
} libcamerac_Request_Status;

struct libcamerac_Request_BufferMap_t;
typedef struct libcamerac_Request_BufferMap_t libcamerac_Request_BufferMap;

typedef enum {
  libcamerac_StreamRole_Raw,
  libcamerac_StreamRole_StillCapture,
  libcamerac_StreamRole_VideoRecording,
  libcamerac_StreamRole_Viewfinder,
} libcamerac_StreamRole;

struct libcamerac_vector_StreamRole_t;
typedef struct libcamerac_vector_StreamRole_t libcamerac_vector_StreamRole;

typedef enum {
  libcamerac_CameraConfiguration_Status_Valid,
  libcamerac_CameraConfiguration_Status_Adjusted,
  libcamerac_CameraConfiguration_Status_Invalid,
} libcamerac_CameraConfiguration_Status;

struct libcamerac_ControlList_t;
typedef struct libcamerac_ControlList_t libcamerac_ControlList;

struct libcamerac_PixelFormat_t;
typedef struct libcamerac_PixelFormat_t libcamerac_PixelFormat;

struct libcamerac_ColorSpace_t;
typedef struct libcamerac_ColorSpace_t libcamerac_ColorSpace;

struct libcamerac_Signal_Request_t;
typedef struct libcamerac_Signal_Request_t libcamerac_Signal_Request;

// libcamerac_CameraManager

extern libcamerac_CameraManager* libcamerac_CameraManager_new();
extern void libcamerac_CameraManager_destroy(libcamerac_CameraManager* p);
extern int libcamerac_CameraManager_start(libcamerac_CameraManager* p);
extern void libcamerac_CameraManager_stop(libcamerac_CameraManager* p);
extern libcamerac_vector_Camera* libcamerac_CameraManager_cameras(
    libcamerac_CameraManager* p);
extern libcamerac_Camera* libcamerac_CameraManager_get(
    libcamerac_CameraManager* p,
    const char* id);

// libcamerac_vector_Camera

extern void libcamerac_vector_Camera_destroy(libcamerac_vector_Camera* p);
extern int libcamerac_vector_Camera_size(libcamerac_vector_Camera* p);
extern libcamerac_Camera* libcamerac_vector_Camera_at(
    libcamerac_vector_Camera* p,
    int i);

// libcamerac_vector_StreamRole

extern libcamerac_vector_StreamRole* libcamerac_vector_StreamRole_new();
extern void libcamerac_vector_StreamRole_destroy(
    libcamerac_vector_StreamRole* p);
extern void libcamerac_vector_StreamRole_push_back(
    libcamerac_vector_StreamRole* p,
    libcamerac_StreamRole v);
extern int libcamerac_vector_StreamRole_size(libcamerac_vector_StreamRole* p);
extern void libcamerac_vector_StreamRole_clear(libcamerac_vector_StreamRole* p);

// libcamerac_Camera

extern libcamerac_Camera* libcamerac_Camera_own(libcamerac_Camera* p);
extern void libcamerac_Camera_disown(libcamerac_Camera* p);
extern const char* libcamerac_Camera_id(libcamerac_Camera* p);
extern libcamerac_CameraConfiguration* libcamerac_Camera_generateConfiguration(
    libcamerac_Camera* p,
    libcamerac_vector_StreamRole* roles);
extern int libcamerac_Camera_configure(libcamerac_Camera* p,
                                       libcamerac_CameraConfiguration* config);
extern libcamerac_Request* libcamerac_Camera_createRequest(
    libcamerac_Camera* p);
extern int libcamerac_Camera_queueRequest(libcamerac_Camera* p,
                                          libcamerac_Request* request);
extern int libcamerac_Camera_start(libcamerac_Camera* p,
                                   const libcamerac_ControlList* control_list);
extern int libcamerac_Camera_stop(libcamerac_Camera* p);
extern int libcamerac_Camera_acquire(libcamerac_Camera* p);
extern int libcamerac_Camera_release(libcamerac_Camera* p);
extern libcamerac_Signal_Request* libcamerac_Camera_requestCompleted(
    libcamerac_Camera* p);

// libcamerac_CameraConfiguration

extern void libcamerac_CameraConfiguration_destroy(
    libcamerac_CameraConfiguration* p);
extern libcamerac_StreamConfiguration* libcamerac_CameraConfiguration_at(
    libcamerac_CameraConfiguration* p,
    int i);
extern libcamerac_CameraConfiguration_Status
libcamerac_CameraConfiguration_validate(libcamerac_CameraConfiguration* p);

// libcamerac_StreamConfiguration

extern libcamerac_Stream* libcamerac_StreamConfiguration_stream(
    libcamerac_StreamConfiguration* p);
extern int libcamerac_StreamConfiguration_get_size_width(
    libcamerac_StreamConfiguration* p);
extern int libcamerac_StreamConfiguration_get_size_height(
    libcamerac_StreamConfiguration* p);
int libcamerac_StreamConfiguration_get_stride(
    libcamerac_StreamConfiguration* p);
extern void libcamerac_StreamConfiguration_set_pixelFormat(
    libcamerac_StreamConfiguration* p,
    libcamerac_PixelFormat* pixelFormat);
extern void libcamerac_StreamConfiguration_set_size_width(
    libcamerac_StreamConfiguration* p,
    int width);
extern void libcamerac_StreamConfiguration_set_size_height(
    libcamerac_StreamConfiguration* p,
    int height);
extern void libcamerac_StreamConfiguration_set_stride(
    libcamerac_StreamConfiguration* p,
    int stride);
extern void libcamerac_StreamConfiguration_set_frameSize(
    libcamerac_StreamConfiguration* p,
    int frameSize);
extern void libcamerac_StreamConfiguration_set_bufferCount(
    libcamerac_StreamConfiguration* p,
    int bufferCount);
extern void libcamerac_StreamConfiguration_set_colorSpace(
    libcamerac_StreamConfiguration* p,
    libcamerac_ColorSpace* colorSpace);

// libcamerac_FrameBufferAllocator

extern libcamerac_FrameBufferAllocator* libcamerac_FrameBufferAllocator_new(
    libcamerac_Camera* camera);
extern void libcamerac_FrameBufferAllocator_destroy(
    libcamerac_FrameBufferAllocator* p);
extern int libcamerac_FrameBufferAllocator_allocate(
    libcamerac_FrameBufferAllocator* p,
    libcamerac_Stream* stream);
extern int libcamerac_FrameBufferAllocator_free(
    libcamerac_FrameBufferAllocator* p,
    libcamerac_Stream* stream);
extern libcamerac_FrameBuffer* libcamerac_FrameBufferAllocator_buffers_at(
    const libcamerac_FrameBufferAllocator* p,
    libcamerac_Stream* stream,
    int i);
extern int libcamerac_FrameBufferAllocator_buffers_size(
    const libcamerac_FrameBufferAllocator* p,
    libcamerac_Stream* stream);

// libcamerac_FrameBuffer

extern int libcamerac_FrameBuffer_planes_size(const libcamerac_FrameBuffer* p);
extern const libcamerac_FrameBuffer_Plane* libcamerac_FrameBuffer_planes_at(
    const libcamerac_FrameBuffer* p,
    int i);

// libcamerac_FrameBuffer_Plane

extern int libcamerac_FrameBuffer_Plane_fd(
    const libcamerac_FrameBuffer_Plane* p);
extern int libcamerac_FrameBuffer_Plane_length(
    const libcamerac_FrameBuffer_Plane* p);

// libcamerac_ControlList

extern libcamerac_ControlList* libcamerac_ControlList_controls();
extern void libcamerac_ControlList_destroy(libcamerac_ControlList* p);
extern void libcamerac_ControlList_clear(libcamerac_ControlList* p);
extern void libcamerac_ControlList_copy(const libcamerac_ControlList* p,
                                        libcamerac_ControlList* control_list);

// libcamerac_Request

extern void libcamerac_Request_destroy(libcamerac_Request* p);
extern libcamerac_Request_Status libcamerac_Request_status(
    const libcamerac_Request* p);
extern libcamerac_FrameBuffer* libcamerac_Request_findBuffer(
    const libcamerac_Request* p,
    const libcamerac_Stream* stream);
extern int libcamerac_Request_addBuffer(libcamerac_Request* p,
                                        const libcamerac_Stream* stream,
                                        libcamerac_FrameBuffer* buffer);
extern void libcamerac_Request_reuse(libcamerac_Request* p);
extern const libcamerac_Request_BufferMap* libcamerac_Request_buffers(
    const libcamerac_Request* p);
extern libcamerac_ControlList* libcamerac_Request_controls(
    libcamerac_Request* p);

// libcamerac_Request_BufferMap

extern void libcamerac_Request_BufferMap_foreach(
    const libcamerac_Request_BufferMap* p,
    void (*callback)(const libcamerac_Stream* stream,
                     libcamerac_FrameBuffer* buffer,
                     void* user_data),
    void* user_data);

// libcamerac_PixelFormat

extern libcamerac_PixelFormat* libcamerac_PixelFormat_YUV420();

// libcamerac_ColorSpace

//extern libcamerac_ColorSpace* libcamerac_ColorSpace_Jpeg();

// libcamerac_Signal_Request

extern void libcamerac_Signal_Request_connect(
    libcamerac_Signal_Request* p,
    void (*callback)(libcamerac_Request* request, void* user_data),
    void* user_data);
extern void libcamerac_Signal_Request_disconnect(
    libcamerac_Signal_Request* p,
    void (*callback)(libcamerac_Request* request, void* user_data),
    void* user_data);

#ifdef __cplusplus
}
#endif

#endif
