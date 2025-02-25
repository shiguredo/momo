import json
import signal
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path


class Momo:
    signaling_urls: list[str]
    channel_id_prefix: str
    metadata: dict[str, str]
    port: int

    def __init__(
        self,
        signaling_urls: list[str],
        channel_id_prefix: str,
        metadata: dict[str, str],
        momo_path: str,
    ):
        self.signaling_urls = signaling_urls
        self.channel_id_prefix = channel_id_prefix
        self.metadata = metadata

        self.channel_id = f"{self.channel_id_prefix}_{uuid.uuid4()}"

        self.port = 5000

        self.executable = Path(momo_path)

        assert self.executable.exists()
        self.process = None
        self.thread = None
        self.is_running = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def run_app(self):
        print(self.executable)
        args = [
            str(self.executable),
            # audio device は作れてないので掴まないようにしてみる
            "--no-audio-device",
            "--log-level",
            "0",
            "sora",
            "--signaling-urls",
            ",".join(self.signaling_urls),
            "--channel-id",
            self.channel_id_prefix,
            # "--video-device",
            # これは GitHub Actions 用
            # "VCamera",
            "--metadata",
            json.dumps(self.metadata),
        ]
        try:
            self.process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.process.wait()
        except Exception as e:
            print(f"Error running momo: {e}", file=sys.stderr)
        finally:
            self.is_running = False

    def start(self):
        if not self.executable.exists():
            raise FileNotFoundError(f"Executable file not found: {self.executable}")

        self.thread = threading.Thread(target=self.run_app)
        self.thread.start()
        self.is_running = True

        # momoの起動を確認
        start_time = time.time()
        while time.time() - start_time < 10:  # 10秒のタイムアウト
            if self.process and self.process.poll() is None:
                print("Momo started")
                return
            time.sleep(0.1)

        raise TimeoutError("Momo failed to start within the timeout period")

    def stop(self):
        if self.is_running and self.process:
            # SIGINT を送信 (Ctrl+C と同等)
            self.process.send_signal(signal.SIGINT)
            try:
                # プロセスが終了するのを最大5秒間待つ
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # タイムアウトした場合、強制終了
                print("Momo did not terminate gracefully. Forcing termination.", file=sys.stderr)
                self.process.kill()

            self.thread.join(timeout=5)
            self.is_running = False
            print("Momo stopped")
