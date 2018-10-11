# armv8 用の WebRTC ビルド全体で必要なイメージ
FROM momo/webrtc-arm-buildbase

COPY patch/arm64.conf /root/

RUN set -x \
 && cd /root \
 && multistrap --no-auth -a arm64 -d rootfs -f /root/arm64.conf \
 && find rootfs/usr/lib/aarch64-linux-gnu -lname '/*' -printf '%p %l\n' | while read link target; do ln -snfv "../../..${target}" "${link}"; done \
 && find rootfs/usr/lib/aarch64-linux-gnu/pkgconfig -printf "%f\n" | while read target; do ln -snfv "../../lib/aarch64-linux-gnu/pkgconfig/${target}" rootfs/usr/share/pkgconfig/${target}; done \
 && unlink rootfs/usr/lib/gcc/aarch64-linux-gnu/5/libgcc_s.so \
 && ln -s ../../../../../lib64/aarch64-linux-gnu/libgcc_s.so.1 rootfs/usr/lib/gcc/aarch64-linux-gnu/5/libgcc_s.so
