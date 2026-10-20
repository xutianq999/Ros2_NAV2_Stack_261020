import unittest

from voice_agent.intent_parser import parse_command


class IntentParserTest(unittest.TestCase):
    def test_forward_defaults(self):
        command = parse_command("前进")
        self.assertIsNotNone(command)
        self.assertEqual(command.action, "move")
        self.assertEqual(command.direction, "forward")
        self.assertIsNone(command.distance_m)

    def test_forward_distance_chinese(self):
        command = parse_command("往前走两米")
        self.assertEqual(command.direction, "forward")
        self.assertAlmostEqual(command.distance_m, 2.0)

    def test_forward_spoken_decimal(self):
        command = parse_command("后退一米五")
        self.assertEqual(command.direction, "backward")
        self.assertAlmostEqual(command.distance_m, 1.5)

    def test_forward_centimeters_and_half(self):
        self.assertAlmostEqual(parse_command("后退50厘米").distance_m, 0.5)
        self.assertAlmostEqual(parse_command("前进半米").distance_m, 0.5)
        self.assertAlmostEqual(parse_command("前进两米五").distance_m, 2.5)

    def test_turn_angle(self):
        command = parse_command("左转九十度")
        self.assertEqual(command.direction, "left")
        self.assertAlmostEqual(command.angle_deg, 90.0)

    def test_turn_default_angle(self):
        command = parse_command("右转")
        self.assertEqual(command.direction, "right")
        self.assertIsNone(command.angle_deg)

    def test_speed_modifier(self):
        command = parse_command("快一点前进")
        self.assertEqual(command.direction, "forward")
        self.assertGreater(command.speed_scale, 1.0)

    def test_strafe_is_explicitly_unsupported(self):
        command = parse_command("向左平移")
        self.assertEqual(command.action, "unsupported")

    def test_stop_variants(self):
        for text in ("停止", "请停下来", "停一下", "停止前进", "别右转", "刹车",
                     "麻烦停一下", "把车停下"):
            self.assertEqual(parse_command(text).action, "stop", text)

    def test_estop_and_release(self):
        self.assertEqual(parse_command("急停").action, "estop")
        self.assertEqual(parse_command("解除急停").action, "release_estop")
        self.assertEqual(parse_command("取消导航").action, "cancel_nav")

    def test_navigate_number(self):
        command = parse_command("导航到1号点位")
        self.assertEqual(command.action, "navigate")
        self.assertEqual(command.point, "1号点位")

    def test_navigate_chinese_number(self):
        command = parse_command("去三号点")
        self.assertEqual(command.action, "navigate")
        self.assertEqual(command.point, "3号点")

    def test_navigate_prefixed(self):
        command = parse_command("带我去point_1")
        self.assertEqual(command.action, "navigate")
        self.assertEqual(command.point, "point_1")

    def test_queries(self):
        self.assertEqual(parse_command("有哪些点位").action, "list_waypoints")
        self.assertEqual(parse_command("状态").action, "status")
        self.assertEqual(parse_command("帮助").action, "help")

    def test_chat_is_not_a_command(self):
        for text in ("你好", "停车场怎么走", "前进是什么意思", "前进还是后退",
                     "到底怎么回事", "到哪里"):
            self.assertIsNone(parse_command(text), text)


if __name__ == "__main__":
    unittest.main()
