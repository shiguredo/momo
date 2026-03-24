"""Momo プロセスを管理するためのクラス"""

import os
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

    P2P = "p2p"
    AYAME = "ayame"


class Momo:
    """Momo プロセスを管理するクラス"""

    def __init__(
        self,
        # モード設定（必須）
        mode: MomoMode,
        # === 共通オプション ===
        no_google_stun: bool = False,
        no_video_input_device: bool = False,
        no_audio_device: bool = False,
        fake_capture_device: bool = True,
        resolution: str | None = None,
        framerate: int | None = None,
        fixed_resolution: bool = False,
        priority: Literal["BALANCE", "FRAMERATE", "RESOLUTION"] | None = None,
        insecure: bool = False,
        log_level: Literal["verbose", "info", "warning", "error", "none"] | None = None,
        disable_echo_cancellation: bool = False,
        disable_auto_gain_control: bool = False,
        disable_noise_suppression: bool = False,
        disable_highpass_filter: bool = False,
        # コーデック設定
        vp8_encoder: str | None = None,
        vp8_decoder: str | None = None,
        vp9_encoder: str | None = None,
        vp9_decoder: str | None = None,
        av1_encoder: str | None = None,
        av1_decoder: str | None = None,
        h264_encoder: str | None = None,
        h264_decoder: str | None = None,
        h265_encoder: str | None = None,
        h265_decoder: str | None = None,
        # その他の共通設定
        metrics_port: int = 9090,
        metrics_allow_external_ip: bool = False,
        # === p2p モード固有 ===
        document_root: str | None = None,
        port: int | None = None,
        # === ayame モード固有 ===
        ayame_signaling_url: str | None = None,
        room_id: str | None = None,
        client_id: str | None = None,
        signaling_key: str | None = None,
        direction: Literal["sendrecv", "sendonly", "recvonly"] | None = None,
        ayame_video_codec_type: Literal["VP8", "VP9", "AV1", "H264", "H265"] | None = None,
        ayame_audio_codec_type: Literal["OPUS", "PCMU", "PCMA"] | None = None,
        # 起動待機時間
        initial_wait: int | None = None,
    ) -> None:
        self.executable_path = self._get_momo_executable_path()
        self.process: subprocess.Popen[Any] | None = None
        self.metrics_port = metrics_port
        self.initial_wait = initial_wait if initial_wait is not None else 2

        self.kwargs: dict[str, Any] = {
            "mode": mode,
            "no_google_stun": no_google_stun,
            "no_video_input_device": no_video_input_device,
            "no_audio_device": no_audio_device,
            "fake_capture_device": fake_capture_device,
            "resolution": resolution,
            "framerate": framerate,
            "fixed_resolution": fixed_resolution,
            "priority": priority,
            "insecure": insecure,
            "log_level": log_level,
            "disable_echo_cancellation": disable_echo_cancellation,
            "disable_auto_gain_control": disable_auto_gain_control,
            "disable_noise_suppression": disable_noise_suppression,
            "disable_highpass_filter": disable_highpass_filter,
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
            "metrics_port": metrics_port,
            "metrics_allow_external_ip": metrics_allow_external_ip,
            "document_root": document_root,
            "port": port,
            "ayame_signaling_url": ayame_signaling_url,
            "room_id": room_id,
            "client_id": client_id,
            "signaling_key": signaling_key,
            "direction": direction,
            "ayame_video_codec_type": ayame_video_codec_type,
            "ayame_audio_codec_type": ayame_audio_codec_type,
        }

        # モード固有オプションの検証を実行
        self._validate_mode_options(mode, self.kwargs)

        # HTTP クライアントの初期化（None で初期化）
        self._http_client: httpx.Client | None = None

    def _get_momo_executable_path(self) -> str:
        """momo-rs 実行ファイルのパスを検出"""
        # 環境変数を最優先
        env_path = os.environ.get("MOMO_BINARY")
        if env_path:
            path = Path(env_path)
            if path.exists():
                return str(path)
            raise RuntimeError(f"MOMO_BINARY={env_path} does not exist")

        # プロジェクトルートから target ディレクトリを検索
        project_root = Path(__file__).parent.parent

        # debug ビルドを優先
        for profile in ["debug", "release"]:
            momo_path = project_root / "target" / profile / "momo"
            if momo_path.exists():
                print(f"Auto-detected momo binary: {momo_path}")
                return str(momo_path)

        raise RuntimeError(
            "momo binary not found. Set MOMO_BINARY environment variable or build with: cargo build"
        )

    def __enter__(self) -> Self:
        """コンテキストマネージャーの開始"""
        try:
            args = self._build_args(**self.kwargs)

            cmd = [self.executable_path] + args
            quoted_cmd = " ".join(shlex.quote(arg) for arg in cmd)
            print(f"Starting momo with command: {quoted_cmd}")

            # プロジェクトルートを cwd にして html ディレクトリを解決する
            project_root = Path(__file__).parent.parent
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                cwd=project_root,
            )
            print(f"Started momo process with PID: {self.process.pid}")

            self._wait_for_startup(self.metrics_port, timeout=30, initial_wait=self.initial_wait)

            self._http_client = httpx.Client(timeout=10.0)

            return self
        except Exception as e:
            if self.process:
                print(f"Cleaning up momo process (PID: {self.process.pid}) due to exception: {e}")
            self._cleanup()
            raise

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_val: BaseException | None,
        _exc_tb: TracebackType | None,
    ) -> Literal[False]:
        """コンテキストマネージャーの終了"""
        if self._http_client:
            self._http_client.close()
            self._http_client = None

        self._cleanup()
        return False

    def _build_args(self, mode: MomoMode, **kwargs: Any) -> list[str]:
        """コマンドライン引数を構築"""
        self._validate_mode_options(mode, kwargs)

        args = []

        # === 共通オプション（モードの前に配置） ===
        if kwargs.get("no_google_stun"):
            args.append("--no-google-stun")
        if kwargs.get("no_video_input_device"):
            args.append("--no-video-input-device")
        if kwargs.get("no_audio_device"):
            args.append("--no-audio-device")
        if kwargs.get("fake_capture_device"):
            args.append("--fake-capture-device")

        if kwargs.get("resolution"):
            args.extend(["--resolution", kwargs["resolution"]])
        if kwargs.get("framerate") is not None:
            args.extend(["--framerate", str(kwargs["framerate"])])
        if kwargs.get("fixed_resolution"):
            args.append("--fixed-resolution")
        if kwargs.get("priority"):
            args.extend(["--priority", kwargs["priority"]])

        if kwargs.get("insecure"):
            args.append("--insecure")
        if kwargs.get("log_level"):
            args.extend(["--log-level", kwargs["log_level"]])

        if kwargs.get("disable_echo_cancellation"):
            args.append("--disable-echo-cancellation")
        if kwargs.get("disable_auto_gain_control"):
            args.append("--disable-auto-gain-control")
        if kwargs.get("disable_noise_suppression"):
            args.append("--disable-noise-suppression")
        if kwargs.get("disable_highpass_filter"):
            args.append("--disable-highpass-filter")

        # コーデック設定
        for codec in ["vp8", "vp9", "av1", "h264", "h265"]:
            for role in ["encoder", "decoder"]:
                key = f"{codec}_{role}"
                if kwargs.get(key):
                    args.extend([f"--{codec}-{role}", kwargs[key]])

        # メトリクス
        if kwargs.get("metrics_port") is not None and kwargs["metrics_port"] != -1:
            args.extend(["--metrics-port", str(kwargs["metrics_port"])])
        if kwargs.get("metrics_allow_external_ip"):
            args.append("--metrics-allow-external-ip")

        # === モード指定とモード固有オプション ===
        if mode == MomoMode.P2P:
            args.append(mode.value)
            if kwargs.get("document_root"):
                args.extend(["--document-root", kwargs["document_root"]])
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
            if kwargs.get("direction"):
                args.extend(["--direction", kwargs["direction"]])
            if kwargs.get("ayame_video_codec_type"):
                args.extend(["--video-codec-type", kwargs["ayame_video_codec_type"]])
            if kwargs.get("ayame_audio_codec_type"):
                args.extend(["--audio-codec-type", kwargs["ayame_audio_codec_type"]])

        return args

    def _validate_mode_options(self, mode: MomoMode, kwargs: dict[str, Any]) -> None:
        """モード固有オプションの検証"""
        p2p_only_options = {"document_root"}

        ayame_only_options = {
            "ayame_signaling_url",
            "room_id",
            "client_id",
            "signaling_key",
            "direction",
            "ayame_video_codec_type",
            "ayame_audio_codec_type",
        }

        specified_options = {k for k, v in kwargs.items() if v is not None}

        if mode == MomoMode.P2P:
            invalid_options = specified_options & ayame_only_options
            if invalid_options:
                raise ValueError(
                    f"Invalid options specified for P2P mode: {', '.join(sorted(invalid_options))}\n"
                    f"These options are only for ayame mode"
                )

        elif mode == MomoMode.AYAME:
            invalid_options = specified_options & p2p_only_options
            if invalid_options:
                raise ValueError(
                    f"Invalid options specified for Ayame mode: "
                    f"{', '.join(sorted(invalid_options))}\n"
                    f"These options are only for p2p mode"
                )

    def _wait_for_startup(
        self, metrics_port: int, timeout: int = 30, initial_wait: int = 2
    ) -> None:
        """プロセスが起動してメトリクスが利用可能になるまで待機"""
        if not self.process:
            raise RuntimeError("Process not started")

        if initial_wait > 0:
            time.sleep(initial_wait)

        print(
            f"Waiting for metrics endpoint to be ready on port {metrics_port} (timeout: {timeout}s)..."
        )
        start_time = time.time()

        with httpx.Client() as client:
            while time.time() - start_time < timeout:
                if self.process.poll() is not None:
                    error_msg = (
                        f"momo process exited unexpectedly with code {self.process.returncode}"
                    )
                    if hasattr(self.process, "stderr") and self.process.stderr:
                        stderr_output = self.process.stderr.read()
                        if stderr_output:
                            error_msg += f"\nStderr output:\n{stderr_output}"
                    raise RuntimeError(error_msg)

                try:
                    url = f"http://localhost:{metrics_port}/metrics"
                    response = client.get(url, timeout=5)
                    if response.status_code == 200:
                        print(f"Momo started successfully after {time.time() - start_time:.1f}s")
                        return
                    else:
                        print(f"  Got status code: {response.status_code}")
                except httpx.ConnectError:
                    elapsed = time.time() - start_time
                    if elapsed > 5 and int(elapsed) % 5 == 0:
                        print(
                            f"  Still waiting for metrics on port {metrics_port} "
                            f"({elapsed:.1f}s elapsed)"
                        )
                except httpx.ConnectTimeout:
                    pass
                except httpx.HTTPStatusError as e:
                    print(f"  HTTP error: {e}")

                time.sleep(1)

            if self.process:
                print(f"Timeout waiting for momo process (PID: {self.process.pid}) to start")
                if hasattr(self.process, "stderr") and self.process.stderr:
                    stderr_output = self.process.stderr.read()
                    if stderr_output:
                        print(f"Stderr output:\n{stderr_output}")
            self._cleanup()
            raise RuntimeError(f"momo process failed to start within {timeout} seconds")

    def _cleanup(self) -> None:
        """プロセスをクリーンアップ"""
        if self.process:
            pid = self.process.pid
            print(f"Terminating momo process (PID: {pid})")
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
                print(f"Momo process (PID: {pid}) terminated gracefully")
            except subprocess.TimeoutExpired:
                print(f"Force killing momo process (PID: {pid})")
                self.process.kill()
                self.process.wait()
                print(f"Momo process (PID: {pid}) killed")
            self.process = None
            time.sleep(0.2)

    def get_metrics(
        self,
        wait_stats: list[dict[str, Any]] | None = None,
        wait_stats_timeout: int = 5,
        wait_after_stats: float = 0,
        interval: float = 0.5,
    ) -> dict[str, Any]:
        """メトリクスを取得"""
        if not self._http_client:
            raise RuntimeError("HTTP client not initialized")
        if not self.metrics_port:
            raise RuntimeError("Process not started")

        if not wait_stats:
            response = self._http_client.get(f"http://localhost:{self.metrics_port}/metrics")
            response.raise_for_status()
            return response.json()

        start_time = time.time()
        while time.time() - start_time < wait_stats_timeout:
            try:
                response = self._http_client.get(f"http://localhost:{self.metrics_port}/metrics")
                if response.status_code == 200:
                    data = response.json()
                    stats = data.get("stats", [])

                    all_conditions_met = True
                    for expected_dict in wait_stats:
                        found = False
                        for stat in stats:
                            if stat.get("type") != expected_dict.get("type"):
                                continue

                            all_items_match = True
                            for key, expected_value in expected_dict.items():
                                if stat.get(key) != expected_value:
                                    all_items_match = False
                                    break

                            if all_items_match:
                                found = True
                                break

                        if not found:
                            all_conditions_met = False
                            break

                    if all_conditions_met:
                        if wait_after_stats > 0:
                            time.sleep(wait_after_stats)
                        return data

            except (httpx.ConnectError, httpx.HTTPStatusError):
                pass

            time.sleep(interval)

        raise RuntimeError(
            f"Timeout waiting for expected stats within {wait_stats_timeout} seconds. "
            f"Expected stats: {wait_stats}"
        )

    def wait_for_connection(
        self,
        timeout: int = 10,
        interval: float = 0.5,
    ) -> bool:
        """接続が確立されるまで待機"""
        if not self._http_client:
            raise RuntimeError("HTTP client not initialized")

        default_conditions = [
            {"type": "transport", "key": "dtlsState", "value": "connected"},
            {"type": "transport", "key": "iceState", "value": "connected"},
        ]

        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                response = self._http_client.get(f"http://localhost:{self.metrics_port}/metrics")
                if response.status_code == 200:
                    data = response.json()
                    stats = data.get("stats", [])

                    all_conditions_met = True
                    for expected in default_conditions:
                        expected_type = expected.get("type")
                        expected_key = expected.get("key")
                        expected_value = expected.get("value")

                        found = False
                        for stat in stats:
                            if stat.get("type") == expected_type:
                                if stat.get(expected_key) == expected_value:
                                    found = True
                                    break

                        if not found:
                            all_conditions_met = False
                            break

                    if all_conditions_met:
                        return True

            except (httpx.ConnectError, httpx.HTTPStatusError):
                pass

            time.sleep(interval)

        return False
