"""Momo プロセスを管理するためのクラス"""

import json
import platform
import shlex
import subprocess
import time
from enum import StrEnum
from pathlib import Path
from types import TracebackType
from typing import Any, Literal, Self

import httpx


class MomoMode(StrEnum):
    """Momo の動作モード"""

    TEST = "test"
    AYAME = "ayame"
    SORA = "sora"


class Momo:
    """Momo プロセスを管理するクラス"""

    def __init__(
        self,
        # モード設定（必須）
        mode: MomoMode,
        # === 共通オプション ===
        no_google_stun: bool = False,
        no_video_device: bool = False,
        no_audio_device: bool = False,
        fake_capture_device: bool = True,
        force_i420: bool = False,
        hw_mjpeg_decoder: bool | None = None,
        use_libcamera: bool = False,
        use_libcamera_native: bool = False,
        libcamera_control: list[tuple[str, str]] | None = None,
        video_device: str | None = None,
        resolution: str | None = None,  # QVGA, VGA, HD, FHD, 4K, or [WIDTH]x[HEIGHT]
        framerate: int | None = None,  # 1-60
        fixed_resolution: bool = False,
        priority: Literal["BALANCE", "FRAMERATE", "RESOLUTION"] | None = None,
        use_sdl: bool = False,
        window_width: int | None = None,  # 180-16384
        window_height: int | None = None,  # 180-16384
        fullscreen: bool = False,
        version: bool = False,
        insecure: bool = False,
        log_level: Literal["verbose", "info", "warning", "error", "none"] | None = None,
        screen_capture: bool = False,
        disable_echo_cancellation: bool = False,
        disable_auto_gain_control: bool = False,
        disable_noise_suppression: bool = False,
        disable_highpass_filter: bool = False,
        video_codec_engines: bool = False,
        # コーデック設定
        vp8_encoder: Literal["default", "software"] | None = None,
        vp8_decoder: Literal["default", "software"] | None = None,
        vp9_encoder: Literal["default", "software"] | None = None,
        vp9_decoder: Literal["default", "software"] | None = None,
        av1_encoder: Literal["default", "software"] | None = None,
        av1_decoder: Literal["default", "software"] | None = None,
        h264_encoder: Literal["default", "videotoolbox", "software"] | None = None,
        h264_decoder: Literal["default", "videotoolbox"] | None = None,
        h265_encoder: Literal["default", "videotoolbox"] | None = None,
        h265_decoder: Literal["default", "videotoolbox"] | None = None,
        openh264: str | None = None,  # ファイルパス
        # その他の共通設定
        serial: str | None = None,  # [DEVICE],[BAUDRATE]
        metrics_port: int = 9090,
        metrics_allow_external_ip: bool = False,
        client_cert: str | None = None,  # PEM ファイルパス
        client_key: str | None = None,  # PEM ファイルパス
        proxy_url: str | None = None,
        proxy_username: str | None = None,
        proxy_password: str | None = None,
        # === test モード固有 ===
        document_root: str | None = None,  # ディレクトリパス
        port: int | None = None,  # test モードのポート
        # === ayame モード固有 ===
        ayame_signaling_url: str | None = None,
        room_id: str | None = None,
        client_id: str | None = None,
        signaling_key: str | None = None,
        # === sora モード固有 ===
        signaling_urls: str | None = None,  # 複数URL可（スペース区切り）
        channel_id: str | None = None,
        auto: bool | None = None,
        video: bool | None = None,
        audio: bool | None = None,
        video_codec_type: Literal["VP8", "VP9", "AV1", "H264", "H265"] | None = None,
        audio_codec_type: Literal["OPUS"] | None = None,
        video_bit_rate: int | None = None,  # 0-30000
        audio_bit_rate: int | None = None,  # 0-510
        role: Literal["sendonly", "recvonly", "sendrecv"] | None = None,
        spotlight: bool | None = None,
        spotlight_number: int | None = None,  # 0-8
        sora_port: int | None = None,  # -1-65535
        simulcast: bool | None = None,
        data_channel_signaling: Literal["true", "false", "none"] | None = None,
        data_channel_signaling_timeout: int | None = None,
        ignore_disconnect_websocket: Literal["true", "false", "none"] | None = None,
        disconnect_wait_timeout: int | None = None,
        metadata: dict[str, Any] | None = None,
        # その他のカスタム引数
        extra_args: list[str] | None = None,
    ) -> None:
        """
        Momo プロセスを管理するクラス

        使用例:
            with Momo(
                mode=MomoMode.TEST,
                resolution="HD"
            ) as m:
                # テストコード
                response = http_client.get(f"http://localhost:{m.metrics_port}/metrics")

            # Sora モードの例
            with Momo(
                mode=MomoMode.SORA,
                signaling_urls="wss://sora.example.com/signaling wss://sora2.example.com/signaling",  # スペース区切りで複数指定可
                channel_id="test-channel",
                role="sendonly",
                video_codec_type="H264",
            ) as m:
                # テストコード
        """
        # 実行ファイルのパスを自動検出
        self.executable_path = self._get_momo_executable_path()
        self.process: subprocess.Popen[Any] | None = None
        self.metrics_port = metrics_port

        # すべての引数を保存
        self.kwargs: dict[str, Any] = {
            "mode": mode,
            "no_google_stun": no_google_stun,
            "no_video_device": no_video_device,
            "no_audio_device": no_audio_device,
            "fake_capture_device": fake_capture_device,
            "force_i420": force_i420,
            "hw_mjpeg_decoder": hw_mjpeg_decoder,
            "use_libcamera": use_libcamera,
            "use_libcamera_native": use_libcamera_native,
            "libcamera_control": libcamera_control,
            "video_device": video_device,
            "resolution": resolution,
            "framerate": framerate,
            "fixed_resolution": fixed_resolution,
            "priority": priority,
            "use_sdl": use_sdl,
            "window_width": window_width,
            "window_height": window_height,
            "fullscreen": fullscreen,
            "version": version,
            "insecure": insecure,
            "log_level": log_level,
            "screen_capture": screen_capture,
            "disable_echo_cancellation": disable_echo_cancellation,
            "disable_auto_gain_control": disable_auto_gain_control,
            "disable_noise_suppression": disable_noise_suppression,
            "disable_highpass_filter": disable_highpass_filter,
            "video_codec_engines": video_codec_engines,
            "vp8_encoder": vp8_encoder,
            "vp8_decoder": vp8_decoder,
            "vp9_encoder": vp9_encoder,
            "vp9_decoder": vp9_decoder,
            "av1_encoder": av1_encoder,
            "av1_decoder": av1_decoder,
            "h264_encoder": h264_encoder,
            "h264_decoder": h264_decoder,
            "h265_encoder": h265_encoder,
            "h265_decoder": h265_decoder,
            "openh264": openh264,
            "serial": serial,
            "metrics_port": metrics_port,
            "metrics_allow_external_ip": metrics_allow_external_ip,
            "client_cert": client_cert,
            "client_key": client_key,
            "proxy_url": proxy_url,
            "proxy_username": proxy_username,
            "proxy_password": proxy_password,
            "document_root": document_root,
            "port": port,
            "ayame_signaling_url": ayame_signaling_url,
            "room_id": room_id,
            "client_id": client_id,
            "signaling_key": signaling_key,
            "signaling_urls": signaling_urls,
            "channel_id": channel_id,
            "auto": auto,
            "video": video,
            "audio": audio,
            "video_codec_type": video_codec_type,
            "audio_codec_type": audio_codec_type,
            "video_bit_rate": video_bit_rate,
            "audio_bit_rate": audio_bit_rate,
            "role": role,
            "spotlight": spotlight,
            "spotlight_number": spotlight_number,
            "sora_port": sora_port,
            "simulcast": simulcast,
            "data_channel_signaling": data_channel_signaling,
            "data_channel_signaling_timeout": data_channel_signaling_timeout,
            "ignore_disconnect_websocket": ignore_disconnect_websocket,
            "disconnect_wait_timeout": disconnect_wait_timeout,
            "metadata": metadata,
            "extra_args": extra_args,
        }

        # モード固有オプションの検証を実行
        self._validate_mode_options(mode, self.kwargs)

    def _get_momo_executable_path(self) -> str:
        """ビルド済みの momo 実行ファイルのパスを自動検出"""
        project_root = Path(__file__).parent.parent
        build_dir = project_root / "_build"

        # _build ディレクトリ内の実際のビルドターゲットを検出
        if not build_dir.exists():
            raise RuntimeError(
                f"Build directory {build_dir} does not exist. "
                f"Please build with: python3 run.py build <target>"
            )

        available_targets = [
            d.name
            for d in build_dir.iterdir()
            if d.is_dir() and (d / "release" / "momo" / "momo").exists()
        ]

        if not available_targets:
            raise RuntimeError(
                f"No built momo executables found in {build_dir}. "
                f"Please build with: python3 run.py build <target>"
            )

        if len(available_targets) == 1:
            # ビルドが1つだけの場合は自動選択
            target = available_targets[0]
            print(f"Auto-detected momo target: {target}")
        else:
            # 複数ビルドがある場合は、プラットフォームに応じて優先順位を決める
            system = platform.system().lower()
            machine = platform.machine().lower()

            # プラットフォームに応じた優先順位リスト
            if system == "darwin":
                if machine == "arm64" or machine == "aarch64":
                    preferred = ["macos_arm64", "macos_x86_64"]
                else:
                    preferred = ["macos_x86_64", "macos_arm64"]
            elif system == "linux":
                if machine == "aarch64":
                    preferred = ["ubuntu-24.04_armv8", "ubuntu-22.04_armv8", "ubuntu-20.04_armv8"]
                else:
                    preferred = [
                        "ubuntu-24.04_x86_64",
                        "ubuntu-22.04_x86_64",
                        "ubuntu-20.04_x86_64",
                    ]
            else:
                preferred = []

            # 優先順位に従って選択
            target = None
            for pref in preferred:
                if pref in available_targets:
                    target = pref
                    print(
                        f"Auto-detected momo target: {target} (from {len(available_targets)} available)"
                    )
                    break

            if not target:
                # 優先順位で見つからない場合は最初のものを使用
                target = available_targets[0]
                print(
                    f"Using first available target: {target} (available: {', '.join(available_targets)})"
                )

        # momo のパスを構築
        assert target is not None
        momo_path = project_root / "_build" / target / "release" / "momo" / "momo"

        if not momo_path.exists():
            raise RuntimeError(
                f"momo executable not found at {momo_path}. "
                f"Please build with: python3 run.py build {target}"
            )

        return str(momo_path)

    def __enter__(self) -> Self:
        """コンテキストマネージャーの開始"""
        # コマンドライン引数を構築
        args = self._build_args(**self.kwargs)

        # 起動コマンドを表示
        cmd = [self.executable_path] + args
        # JSON を含む引数を適切に表示するため、shlex.quote を使用
        quoted_cmd = " ".join(shlex.quote(arg) for arg in cmd)
        print(f"Starting momo with command: {quoted_cmd}")

        # プロセスを起動
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # プロセスが起動してメトリクスが利用可能になるまで待機
        self._wait_for_startup(self.metrics_port)
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_val: BaseException | None,
        _exc_tb: TracebackType | None,
    ) -> Literal[False]:
        """コンテキストマネージャーの終了"""
        self._cleanup()
        return False

    def _build_args(self, mode: MomoMode, **kwargs: Any) -> list[str]:
        """コマンドライン引数を構築"""
        # モード固有オプションの検証
        self._validate_mode_options(mode, kwargs)

        args = []

        # === 共通オプション（モードの前に配置） ===

        # 基本フラグ
        if kwargs.get("no_google_stun"):
            args.append("--no-google-stun")
        if kwargs.get("no_video_device"):
            args.append("--no-video-device")
        if kwargs.get("no_audio_device"):
            args.append("--no-audio-device")
        if kwargs.get("fake_capture_device"):
            args.append("--fake-capture-device")
        if kwargs.get("force_i420"):
            args.append("--force-i420")
        if kwargs.get("hw_mjpeg_decoder") is not None:
            args.extend(["--hw-mjpeg-decoder", str(int(kwargs["hw_mjpeg_decoder"]))])
        if kwargs.get("use_libcamera"):
            args.append("--use-libcamera")
        if kwargs.get("use_libcamera_native"):
            args.append("--use-libcamera-native")

        # libcamera control
        if kwargs.get("libcamera_control"):
            for key, value in kwargs["libcamera_control"]:
                args.extend(["--libcamera-control", key, value])

        # ビデオ設定
        if kwargs.get("video_device"):
            args.extend(["--video-device", kwargs["video_device"]])
        if kwargs.get("resolution"):
            args.extend(["--resolution", kwargs["resolution"]])
        if kwargs.get("framerate") is not None:
            args.extend(["--framerate", str(kwargs["framerate"])])
        if kwargs.get("fixed_resolution"):
            args.append("--fixed-resolution")
        if kwargs.get("priority"):
            args.extend(["--priority", kwargs["priority"]])

        # SDL設定
        if kwargs.get("use_sdl"):
            args.append("--use-sdl")
        if kwargs.get("window_width") is not None:
            args.extend(["--window-width", str(kwargs["window_width"])])
        if kwargs.get("window_height") is not None:
            args.extend(["--window-height", str(kwargs["window_height"])])
        if kwargs.get("fullscreen"):
            args.append("--fullscreen")

        # その他の基本設定
        if kwargs.get("version"):
            args.append("--version")
        if kwargs.get("insecure"):
            args.append("--insecure")
        if kwargs.get("log_level"):
            args.extend(["--log-level", kwargs["log_level"]])
        if kwargs.get("screen_capture"):
            args.append("--screen-capture")

        # オーディオ設定
        if kwargs.get("disable_echo_cancellation"):
            args.append("--disable-echo-cancellation")
        if kwargs.get("disable_auto_gain_control"):
            args.append("--disable-auto-gain-control")
        if kwargs.get("disable_noise_suppression"):
            args.append("--disable-noise-suppression")
        if kwargs.get("disable_highpass_filter"):
            args.append("--disable-highpass-filter")

        # コーデック設定
        if kwargs.get("video_codec_engines"):
            args.append("--video-codec-engines")
        if kwargs.get("vp8_encoder"):
            args.extend(["--vp8-encoder", kwargs["vp8_encoder"]])
        if kwargs.get("vp8_decoder"):
            args.extend(["--vp8-decoder", kwargs["vp8_decoder"]])
        if kwargs.get("vp9_encoder"):
            args.extend(["--vp9-encoder", kwargs["vp9_encoder"]])
        if kwargs.get("vp9_decoder"):
            args.extend(["--vp9-decoder", kwargs["vp9_decoder"]])
        if kwargs.get("av1_encoder"):
            args.extend(["--av1-encoder", kwargs["av1_encoder"]])
        if kwargs.get("av1_decoder"):
            args.extend(["--av1-decoder", kwargs["av1_decoder"]])
        if kwargs.get("h264_encoder"):
            args.extend(["--h264-encoder", kwargs["h264_encoder"]])
        if kwargs.get("h264_decoder"):
            args.extend(["--h264-decoder", kwargs["h264_decoder"]])
        if kwargs.get("h265_encoder"):
            args.extend(["--h265-encoder", kwargs["h265_encoder"]])
        if kwargs.get("h265_decoder"):
            args.extend(["--h265-decoder", kwargs["h265_decoder"]])
        if kwargs.get("openh264"):
            args.extend(["--openh264", kwargs["openh264"]])

        # その他の共通設定
        if kwargs.get("serial"):
            args.extend(["--serial", kwargs["serial"]])
        if kwargs.get("metrics_port") is not None and kwargs["metrics_port"] != -1:
            args.extend(["--metrics-port", str(kwargs["metrics_port"])])
        if kwargs.get("metrics_allow_external_ip"):
            args.append("--metrics-allow-external-ip")
        if kwargs.get("client_cert"):
            args.extend(["--client-cert", kwargs["client_cert"]])
        if kwargs.get("client_key"):
            args.extend(["--client-key", kwargs["client_key"]])
        if kwargs.get("proxy_url"):
            args.extend(["--proxy-url", kwargs["proxy_url"]])
        if kwargs.get("proxy_username"):
            args.extend(["--proxy-username", kwargs["proxy_username"]])
        if kwargs.get("proxy_password"):
            args.extend(["--proxy-password", kwargs["proxy_password"]])

        # === モード指定とモード固有オプション ===

        if mode == MomoMode.TEST:
            args.append(mode.value)
            if kwargs.get("document_root"):
                args.extend(["--document-root", kwargs["document_root"]])
            # test モードのデフォルトポートは 8080
            port = kwargs.get("port") if kwargs.get("port") is not None else 8080
            args.extend(["--port", str(port)])

        elif mode == MomoMode.AYAME:
            args.append(mode.value)
            if kwargs.get("ayame_signaling_url"):
                args.extend(["--signaling-url", kwargs["ayame_signaling_url"]])
            if kwargs.get("room_id"):
                args.extend(["--room-id", kwargs["room_id"]])
            if kwargs.get("client_id"):
                args.extend(["--client-id", kwargs["client_id"]])
            if kwargs.get("signaling_key"):
                args.extend(["--signaling-key", kwargs["signaling_key"]])

        elif mode == MomoMode.SORA:
            args.append(mode.value)
            if kwargs.get("signaling_urls"):
                # 複数URL対応（文字列の場合はスペース区切りで分割）
                if isinstance(kwargs["signaling_urls"], list):
                    args.extend(["--signaling-urls"] + kwargs["signaling_urls"])
                else:
                    # 文字列の場合はスペース区切りで分割して複数の引数として渡す
                    urls = kwargs["signaling_urls"].split()
                    args.extend(["--signaling-urls"] + urls)
            if kwargs.get("channel_id"):
                args.extend(["--channel-id", kwargs["channel_id"]])
            if kwargs.get("auto"):
                args.append("--auto")
            # sora モードのデフォルトは video/audio ともに true
            video = kwargs.get("video") if kwargs.get("video") is not None else True
            audio = kwargs.get("audio") if kwargs.get("audio") is not None else True
            args.extend(["--video", str(video).lower()])
            args.extend(["--audio", str(audio).lower()])
            if kwargs.get("video_codec_type"):
                args.extend(["--video-codec-type", kwargs["video_codec_type"]])
            if kwargs.get("audio_codec_type"):
                args.extend(["--audio-codec-type", kwargs["audio_codec_type"]])
            if kwargs.get("video_bit_rate") is not None:
                args.extend(["--video-bit-rate", str(kwargs["video_bit_rate"])])
            if kwargs.get("audio_bit_rate") is not None:
                args.extend(["--audio-bit-rate", str(kwargs["audio_bit_rate"])])
            if kwargs.get("role"):
                args.extend(["--role", kwargs["role"]])
            if kwargs.get("spotlight") is not None:
                args.extend(["--spotlight", str(int(kwargs["spotlight"]))])
            if kwargs.get("spotlight_number") is not None:
                args.extend(["--spotlight-number", str(kwargs["spotlight_number"])])
            if kwargs.get("sora_port") is not None:
                args.extend(["--port", str(kwargs["sora_port"])])
            if kwargs.get("simulcast") is not None:
                args.extend(["--simulcast", str(kwargs["simulcast"]).lower()])
            if kwargs.get("data_channel_signaling"):
                args.extend(["--data-channel-signaling", kwargs["data_channel_signaling"]])
            if kwargs.get("data_channel_signaling_timeout") is not None:
                args.extend(
                    [
                        "--data-channel-signaling-timeout",
                        str(kwargs["data_channel_signaling_timeout"]),
                    ]
                )
            if kwargs.get("ignore_disconnect_websocket"):
                args.extend(
                    ["--ignore-disconnect-websocket", kwargs["ignore_disconnect_websocket"]]
                )
            if kwargs.get("disconnect_wait_timeout") is not None:
                args.extend(["--disconnect-wait-timeout", str(kwargs["disconnect_wait_timeout"])])
            if kwargs.get("metadata"):
                args.extend(["--metadata", json.dumps(kwargs["metadata"])])

        # その他のカスタム引数
        if kwargs.get("extra_args"):
            args.extend(kwargs["extra_args"])

        return args

    def _validate_mode_options(self, mode: MomoMode, kwargs: dict[str, Any]) -> None:
        """モード固有オプションの検証"""
        # test モード固有オプション
        test_only_options = {"document_root"}

        # ayame モード固有オプション
        ayame_only_options = {"ayame_signaling_url", "room_id", "client_id", "signaling_key"}

        # sora モード固有オプション
        sora_only_options = {
            "signaling_urls",
            "channel_id",
            "auto",
            "video",
            "audio",
            "video_codec_type",
            "audio_codec_type",
            "video_bit_rate",
            "audio_bit_rate",
            "role",
            "spotlight",
            "spotlight_number",
            "sora_port",
            "simulcast",
            "data_channel_signaling",
            "data_channel_signaling_timeout",
            "ignore_disconnect_websocket",
            "disconnect_wait_timeout",
            "metadata",
        }

        # 指定されたオプションのキーを取得（None でないもの）
        specified_options = {k for k, v in kwargs.items() if v is not None}

        # モードごとの検証
        if mode == MomoMode.TEST:
            # test モードで ayame/sora オプションが指定されていたらエラー
            invalid_options = specified_options & (ayame_only_options | sora_only_options)
            if invalid_options:
                # どのモードのオプションか判定
                modes = []
                if invalid_options & ayame_only_options:
                    modes.append("ayame")
                if invalid_options & sora_only_options:
                    modes.append("sora")
                raise ValueError(
                    f"Invalid options specified for Test mode: {', '.join(sorted(invalid_options))}\n"
                    f"These options are only for {'/'.join(modes)} mode"
                )

        elif mode == MomoMode.AYAME:
            # ayame モードで test/sora オプションが指定されていたらエラー
            invalid_options = specified_options & (test_only_options | sora_only_options)
            if invalid_options:
                # どのモードのオプションか判定
                modes = []
                if invalid_options & test_only_options:
                    modes.append("test")
                if invalid_options & sora_only_options:
                    modes.append("sora")
                raise ValueError(
                    f"Invalid options specified for Ayame mode: {', '.join(sorted(invalid_options))}\n"
                    f"These options are only for {'/'.join(modes)} mode"
                )

        elif mode == MomoMode.SORA:
            # sora モードで test/ayame オプションが指定されていたらエラー
            invalid_options = specified_options & (test_only_options | ayame_only_options)
            if invalid_options:
                # どのモードのオプションか判定
                modes = []
                if invalid_options & test_only_options:
                    modes.append("test")
                if invalid_options & ayame_only_options:
                    modes.append("ayame")
                raise ValueError(
                    f"Invalid options specified for Sora mode: {', '.join(sorted(invalid_options))}\n"
                    f"These options are only for {'/'.join(modes)} mode"
                )

    def _wait_for_startup(
        self, metrics_port: int, timeout: int = 10, initial_wait: int = 2
    ) -> None:
        """プロセスが起動してメトリクスが利用可能になるまで待機"""
        if not self.process:
            raise RuntimeError("Process not started")

        # プロセスが完全に起動するまで少し待機
        # 即座にメトリクスエンドポイントをチェックすると失敗することがあるため
        if initial_wait > 0:
            time.sleep(initial_wait)

        print(f"Waiting for metrics endpoint to be ready (timeout: {timeout}s)...")
        start_time = time.time()

        with httpx.Client() as client:
            while time.time() - start_time < timeout:
                # プロセスの状態を確認
                if self.process.poll() is not None:
                    # プロセスが終了していたらエラー
                    raise RuntimeError(
                        f"momo process exited unexpectedly with code {self.process.returncode}"
                    )

                # メトリクスエンドポイントをチェック
                try:
                    response = client.get(f"http://localhost:{metrics_port}/metrics", timeout=1)
                    if response.status_code == 200:
                        # Soraモードの場合はstatsが空でないことを確認
                        if self.kwargs["mode"] == MomoMode.SORA:
                            data = response.json()
                            stats = data.get("stats", [])
                            if stats and len(stats) > 0:
                                print(
                                    f"Momo started successfully after {time.time() - start_time:.1f}s (stats count: {len(stats)})"
                                )
                                return
                            else:
                                # statsが空の場合は待機を続ける
                                elapsed = time.time() - start_time
                                print(
                                    f"  Metrics endpoint is up but stats is empty, waiting for connection... ({elapsed:.1f}s elapsed)"
                                )
                        else:
                            # test/ayameモードは200応答で成功（statsのチェック不要）
                            print(
                                f"Momo started successfully after {time.time() - start_time:.1f}s"
                            )
                            return
                    else:
                        print(f"  Got status code: {response.status_code}")
                except httpx.ConnectError:
                    # 接続エラーは無視して次の試行へ
                    pass
                except httpx.ConnectTimeout:
                    pass
                except httpx.HTTPStatusError as e:
                    print(f"  HTTP error: {e}")

                # 次の試行まで1秒待機
                time.sleep(1)

            # タイムアウト
            self._cleanup()
            raise RuntimeError(f"momo process failed to start within {timeout} seconds")

    def _cleanup(self) -> None:
        """プロセスをクリーンアップ"""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
            self.process = None

    def get_metrics(self, client: httpx.Client) -> dict[str, Any]:
        """メトリクスを取得"""
        if not self.metrics_port:
            raise RuntimeError("Process not started")
        response = client.get(f"http://localhost:{self.metrics_port}/metrics")
        response.raise_for_status()
        return response.json()
