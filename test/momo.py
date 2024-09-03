import subprocess
import threading
import time
from pathlib import Path

RELEASE_DIR = Path(__file__).resolve().parent.parent / Path("_build/")


class Momo:
    def __init__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def run_app(self, args):
        self.process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.process.wait()
        self.is_running = False

    def start(self):
        self.thread = threading.Thread(target=self.run_app, args=())
        self.thread.start()
        self.is_running = True
        time.sleep(1)  # アプリケーションの起動を待つ

    def stop(self):
        pass
