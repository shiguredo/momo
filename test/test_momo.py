import time

from momo_sora import Momo


def test_start_and_stop(setup):
    with Momo(**setup) as momo:
        momo.start()

        time.sleep(3)

        momo.stop()
