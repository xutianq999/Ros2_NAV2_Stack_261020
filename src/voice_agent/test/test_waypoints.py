import os
import tempfile
import unittest

from voice_agent.waypoints import WaypointStore


CONTENT = """\
frame: map
points:
- id: 1
  name: point_1
  x: 0.5
  y: 0.2
  yaw: 0.0
- id: 2
  name: kitchen
  aliases: ["厨房", "茶水间"]
  x: 9.4
  y: 0.7
  yaw: 1.57
- id: 10
  name: point_10
  x: 3.0
  y: 4.0
  yaw: -1.0
"""


class WaypointStoreTest(unittest.TestCase):
    def setUp(self):
        handle, self.path = tempfile.mkstemp(suffix=".yaml")
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(CONTENT)
        self.store = WaypointStore(self.path)

    def tearDown(self):
        os.unlink(self.path)

    def test_load(self):
        self.assertEqual(len(self.store.points), 3)
        self.assertEqual(self.store.frame, "map")
        self.assertIsNone(self.store.load_error)

    def test_resolve_variants(self):
        cases = {
            "1": 1,
            "1号点位": 1,
            "一号点": 1,
            "第1个点": 1,
            "point_1": 1,
            "point 1": 1,
            "厨房": 2,
            "茶水间": 2,
            "2号点位": 2,
            "十号点": 10,
            "point_10": 10,
        }
        for query, expected in cases.items():
            point = self.store.resolve(query)
            self.assertIsNotNone(point, query)
            self.assertEqual(point.id, expected, query)

    def test_unknown_point(self):
        self.assertIsNone(self.store.resolve("火星基地"))

    def test_list_text(self):
        text = self.store.list_text()
        self.assertIn("1号点位", text)
        self.assertIn("厨房", text)

    def test_missing_file_is_reported(self):
        store = WaypointStore("/nonexistent/waypoints.yaml")
        self.assertEqual(store.points, [])
        self.assertIn("不存在", store.load_error)


if __name__ == "__main__":
    unittest.main()
