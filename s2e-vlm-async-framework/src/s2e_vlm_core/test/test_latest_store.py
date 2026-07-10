import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from s2e_vlm_core.latest_store import LatestStore


class LatestStoreTest(unittest.TestCase):
    def test_put_get_and_pop_latest_value(self):
        store = LatestStore()
        store.put("first")
        store.put("second")

        self.assertEqual(store.get(), "second")
        self.assertEqual(store.pop(), "second")
        self.assertIsNone(store.get())

    def test_wait_for_returns_when_predicate_matches(self):
        store = LatestStore()

        def delayed_put():
            time.sleep(0.02)
            store.put({"state": "ready"})

        thread = threading.Thread(target=delayed_put)
        thread.start()

        value = store.wait_for(lambda item: bool(item and item["state"] == "ready"), timeout=0.5)
        thread.join()

        self.assertEqual(value, {"state": "ready"})

    def test_wait_for_times_out_without_match(self):
        started = time.monotonic()
        value = LatestStore().wait_for(lambda item: bool(item is not None), timeout=0.02)

        self.assertIsNone(value)
        self.assertGreaterEqual(time.monotonic() - started, 0.015)


if __name__ == "__main__":
    unittest.main()
