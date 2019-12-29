#ifndef MOMO_VERSION_H_INCLUDED
#define MOMO_VERSION_H_INCLUDED

// バージョンやコミットハッシュ情報
// 通常は外から渡すが、渡されていなかった場合の対応
#ifndef MOMO_VERSION
#define MOMO_VERSION "internal-build"
#endif

#ifndef MOMO_COMMIT_SHORT
#define MOMO_COMMIT_SHORT "unknown"
#endif

#define MOMO_NAME \
  "WebRTC Native Client Momo " MOMO_VERSION " (" MOMO_COMMIT_SHORT ")"

#if defined(WEBRTC_READABLE_VERSION) && defined(WEBRTC_COMMIT_SHORT) && \
    defined(WEBRTC_BUILD_VERSION)

#define LIBWEBRTC_NAME                                                 \
  "Shiguredo-Build " WEBRTC_READABLE_VERSION " (" WEBRTC_BUILD_VERSION \
  " " WEBRTC_COMMIT_SHORT ")"

#else

#define LIBWEBRTC_NAME "WebRTC custom build"

#endif

#endif  // MOMO_VERSION_H_INCLUDED
