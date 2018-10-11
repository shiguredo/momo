# armv6 用の WebRTC ビルド全体で必要なイメージ
FROM momo/webrtc-arm-buildbase

COPY patch/rpi-raspbian.conf /root/

RUN set -x \
 && cd /root \
 && multistrap --no-auth -a armhf -d rootfs -f /root/rpi-raspbian.conf \
 && find rootfs/usr/lib/arm-linux-gnueabihf -lname '/*' -printf '%p %l\n' | while read link target; do ln -snfv "../../..${target}" "${link}"; done \
 && find rootfs/usr/lib/arm-linux-gnueabihf/pkgconfig -printf "%f\n" | while read target; do ln -snfv "../../lib/arm-linux-gnueabihf/pkgconfig/${target}" rootfs/usr/share/pkgconfig/${target}; done
