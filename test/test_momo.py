import time

from momo import Momo


def test_start_and_stop():
    with Momo() as momo:
        momo.start()

        time.sleep(3)

        momo.stop()
