"""执行边界回归：只用模拟传输，不连接模型服务或家电。

运行：server/.venv/bin/python -m unittest discover -s server -p test_control_safety.py -v
"""
import unittest
from unittest.mock import AsyncMock, patch

import intent
from executor import LightExecutor, Router
from llm import LocalLLM
from tivoli import TivoliExecutor


class ModelSlotsTests(unittest.IsolatedAsyncioTestCase):
    async def classify(self, data, raw="模拟语句"):
        model = LocalLLM.__new__(LocalLLM)
        model._ask = AsyncMock(return_value=data)
        return await model.classify(raw)

    async def test_valid_parameters_reach_light_hardware_adapter(self):
        class Lamp:
            P_ON, P_BRIGHT, P_CCT = 1, 2, 3

            def get(self):
                return {"brightness": 50}

            def set_many(self, props):
                self.props = props

        # 绝对值槽位都要原话依据（见 llm._SLOT_EVIDENCE），所以每条都把值说出来
        for action, fields, expected, said in [
            ("brightness", {"pct": 30}, {1: True, 2: 30}, "亮度调到30"),
            ("brightness", {"pct": 0}, {1: True, 2: 1}, "亮度调到0"),  # 同规则路径：最低亮度 1
            ("brightness_step", {"step": -20}, {1: True, 2: 30}, "暗一点"),  # step 不要依据
            ("color_temp", {"kelvin": 3000}, {1: True, 3: 3000}, "色温3000"),
            ("on", {"pct": 40, "kelvin": 4000}, {1: True, 2: 40, 3: 4000}, "把灯开到40%，色温4000"),
        ]:
            with self.subTest(action=action, fields=fields):
                guess = await self.classify({"domain": "light", "action": action, **fields},
                                            raw=said)
                lamp = Lamp()
                executor = LightExecutor(None)
                executor._local = lamp
                router = Router(executor, llm=AsyncMock())
                router.llm.classify.return_value = guess
                with patch("executor.intent_mod.parse", return_value=intent.Intent()):
                    parsed, ok, _ = await router.parse_and_execute("模拟语句")
                self.assertTrue(ok)
                self.assertEqual(lamp.props, expected)
                self.assertEqual(parsed.slots, fields)

    async def test_missing_wrong_type_or_out_of_range_never_becomes_action(self):
        for domain, action, fields in [
            ([], "brightness", {"pct": 30}),
            ("light", {}, {"pct": 30}),
            ("light", "brightness", {"pct": True}),
            ("light", "brightness", {"pct": "30"}),
            ("light", "brightness", {"pct": 30.5}),
            ("light", "brightness", {"pct": 101}),
            ("light", "color_temp", {"kelvin": -1}),
            ("light", "brightness_step", {"step": 0}),
            ("tivoli", "preset_save", {"preset": 7}),
            ("tivoli", "volume_step", {"step": 999}),
            ("aircon", "set_temp", {"temp": 100}),
            ("aircon", "temp_step", {}),
            ("music", "play", {"query": " "}),
        ]:
            with self.subTest(domain=domain, action=action, fields=fields):
                self.assertIsNone(await self.classify({"domain": domain, "action": action, **fields}))

    async def test_missing_askable_slot_asks_instead_of_giving_up(self):
        """缺参和乱编必须走不同的路 —— 前者我们知道他要干什么，只是不知道干到几。"""
        for domain, action, key, ask in [
            ("light", "brightness", "pct", "亮度调到多少？"),
            ("tivoli", "preset_recall", "preset", "第几个台？"),
            ("aircon", "set_temp", "temp", "空调调到多少度？"),
            ("music", "play", "query", "想听什么？"),
        ]:
            with self.subTest(domain=domain, action=action):
                result = await self.classify({"domain": domain, "action": action})
                self.assertEqual(result["missing"], key)
                self.assertEqual(result["ask"], ask)
                self.assertEqual(result["slots"], {})

    async def test_missing_step_is_dropped_not_asked(self):
        """方向就是这条指令的全部内容。给不出方向 = 没听懂，该丢不该问。"""
        for domain, action in [("light", "brightness_step"), ("tivoli", "volume_step"),
                               ("tivoli", "station_step"), ("aircon", "temp_step")]:
            with self.subTest(domain=domain, action=action):
                self.assertIsNone(await self.classify({"domain": domain, "action": action}))

    async def test_plausible_filler_value_is_asked_about_not_executed(self):
        """实测过的真事：「太亮了受不了」-> light.brightness pct=0。

        0 落在 [0,100] 里，范围检查一个字都挑不出来，灯就被设成最低亮度 ——
        一个用户从没说过的绝对值。编的动作有白名单挡着，编的数值长得和真的一样。
        """
        result = await self.classify({"domain": "light", "action": "brightness", "pct": 0},
                                     raw="太亮了受不了")
        self.assertEqual(result["missing"], "pct")
        self.assertEqual(result["slots"], {})
        # 同一个值，用户真说了，就照做
        result = await self.classify({"domain": "light", "action": "brightness", "pct": 0},
                                     raw="亮度调到0")
        self.assertEqual(result["slots"], {"pct": 0})

    async def test_invalid_value_is_dropped_never_asked(self):
        """乱编的值和没给的值必须走不同的路 —— 前者问也问不出个所以然。"""
        for fields in ({"pct": 101}, {"pct": "30"}, {"pct": True}, {"pct": 30.5}):
            with self.subTest(fields=fields):
                self.assertIsNone(await self.classify(
                    {"domain": "light", "action": "brightness", **fields},
                    raw="亮度调到30"))

    async def test_optional_slot_without_evidence_is_ignored(self):
        """用户只说了「开灯」，schema 里摆着 pct，模型顺手填一个 —— 不能拿去动设备。"""
        result = await self.classify({"domain": "light", "action": "on",
                                      "pct": 50, "kelvin": 3000}, raw="开灯")
        self.assertEqual(result["slots"], {})          # 动作保留，槽位丢掉
        self.assertEqual(result["action"], "on")
        result = await self.classify({"domain": "light", "action": "on", "pct": 50},
                                     raw="把灯开到一半")   # "一半" 是依据，"一点" 不是
        self.assertEqual(result["slots"], {"pct": 50})  # 原话提了，就收

    async def test_domain_repair_respects_the_sentence(self):
        """原话佐证了它说的域，那幻觉的就是动作 —— 不能按动作改到另一台设备上。"""
        # 「这首」指着音乐。修成 light.brightness_step 就是去调台灯亮度。
        self.assertIsNone(await self.classify(
            {"domain": "music", "action": "brightness_step", "step": -20},
            raw="这首太吵了"))
        # 原话没有任何音乐词，域是瞎猜的 —— 照旧按动作修回 tivoli
        result = await self.classify(
            {"domain": "music", "action": "volume_step", "step": -3},
            raw="把收音机声音关小")
        self.assertEqual(result["domain"], "tivoli")
        # 两边都被提到时，动作才是分得开的那个信号 —— 不能因为"歌"在就丢掉
        result = await self.classify(
            {"domain": "music", "action": "volume_step", "step": -3},
            raw="这首歌音量小一点")
        self.assertEqual(result["domain"], "tivoli")

    async def test_optional_fields_and_unrelated_fields(self):
        self.assertEqual((await self.classify({"domain": "light", "action": "on"}))["slots"], {})
        result = await self.classify({"domain": "light", "action": "off", "pct": 80})
        self.assertEqual(result["slots"], {})
        result = await self.classify({"domain": "aircon", "action": "set_temp", "temp": 26},
                                     raw="空调调到26度")
        self.assertEqual(result["slots"], {"temp": 26})

    async def test_action_domain_repair_still_validates_parameters(self):
        result = await self.classify({"domain": "music", "action": "volume_step", "step": -3})
        self.assertEqual(result["domain"], "tivoli")
        self.assertIsNone(await self.classify({"domain": "music", "action": "volume_step"}))


class PowerTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.air = AsyncMock()
        self.ha = AsyncMock()
        self.target = TivoliExecutor(self.ha, self.air,
                                    conf={"power_settle_ms": 0, "ir_gap_ms": 0})

    async def test_unreachable_never_toggles_power_on(self):
        self.air.anchor.return_value = False
        self.air.reachable.return_value = False
        self.target.shadow.set(powered=False, source="fm", anchored_at=1)
        ok, _ = await self.target.ensure_wifi()
        self.assertFalse(ok)
        self.ha.call.assert_not_awaited()
        self.assertIsNone(self.target.shadow.powered)
        self.assertIsNone(self.target.shadow.source)

    async def test_online_relink_failure_never_toggles(self):
        self.air.anchor.return_value = False
        self.air.reachable.return_value = True
        ok, _ = await self.target.ensure_wifi()
        self.assertFalse(ok)
        self.assertTrue(self.target.shadow.powered)
        self.ha.call.assert_not_awaited()

    async def test_successful_anchor_still_works(self):
        self.air.anchor.return_value = True
        ok, _ = await self.target.ensure_wifi()
        self.assertTrue(ok)
        self.assertEqual(self.target.shadow.source, "wifi")
        self.ha.call.assert_not_awaited()

    async def test_unknown_or_stale_power_never_toggles_off(self):
        self.air.reachable.return_value = False
        self.target.shadow.set(powered=True)
        ok, _ = await self.target.power_off()
        self.assertFalse(ok)
        self.ha.call.assert_not_awaited()
        self.assertIsNone(self.target.shadow.powered)

    async def test_network_loss_after_send_is_not_confirmed_off(self):
        self.air.reachable.side_effect = [True, False]
        ok, reply = await self.target.power_off()
        self.assertFalse(ok)
        self.assertIn("未确认", reply)
        self.assertIsNone(self.target.shadow.powered)
        self.ha.call.assert_awaited_once_with("button", "press", entity_id="button.tivoli_ir_power")
        self.air.anchor.assert_not_awaited()

    async def test_still_online_never_sends_second_toggle(self):
        self.air.reachable.side_effect = [True, True]
        ok, _ = await self.target.power_off()
        self.assertFalse(ok)
        self.assertTrue(self.target.shadow.powered)
        self.assertEqual(self.ha.call.await_count, 1)

    async def test_send_failure_leaves_unknown_state(self):
        self.air.reachable.return_value = True
        self.ha.call.side_effect = RuntimeError("mock transport timeout")
        ok, _ = await self.target.power_off()
        self.assertFalse(ok)
        self.assertIsNone(self.target.shadow.powered)
        self.assertEqual(self.ha.call.await_count, 1)


class AskBackRoundTripTests(unittest.IsolatedAsyncioTestCase):
    """缺参 -> 问一句 -> 用户补一个裸值 -> 真的执行。

    裸值这条路必须单独接：实测 parse("26度") 解不出任何意图（"调到26度"才行），
    所以不能像 _answer_slot 那样"把域顶到最前面再重解一遍"。
    """

    def setUp(self):
        self.aircon = AsyncMock()
        self.aircon.believed_on = None
        self.aircon.execute.return_value = (True, "调到26度")
        self.llm = AsyncMock()
        self.router = Router(LightExecutor(None), aircon=self.aircon, llm=self.llm)

    async def ask(self, text="空调调一下"):
        self.llm.classify.return_value = {
            "domain": "aircon", "action": "set_temp", "slots": {},
            "missing": "temp", "ask": "空调调到多少度？", "reply": ""}
        with patch("executor.intent_mod.parse", return_value=intent.Intent()):
            return await self.router.parse_and_execute(text)

    async def test_asks_then_executes_with_the_answer(self):
        parsed, ok, reply = await self.ask()
        self.assertEqual(reply, "空调调到多少度？")
        self.assertEqual(parsed.rule, "ask_param")
        self.assertTrue(self.router.asking)      # 板子要靠它决定追问窗口里出不出声
        self.aircon.execute.assert_not_awaited()  # 还没问到值，什么都不许发生

        parsed, ok, reply = await self.router.parse_and_execute("26度")
        self.assertTrue(ok)
        self.assertEqual(parsed.domain, "aircon")
        self.assertEqual(parsed.action, "set_temp")
        self.assertEqual(parsed.slots, {"temp": 26})
        self.assertIsNone(self.router.ctx.question)

    async def test_chinese_numerals_in_the_answer(self):
        await self.ask()
        parsed, ok, _ = await self.router.parse_and_execute("二十六度吧")
        self.assertEqual(parsed.slots, {"temp": 26})

    async def test_out_of_range_answer_never_reaches_hardware(self):
        await self.ask()
        parsed, ok, reply = await self.router.parse_and_execute("一百度")
        self.assertFalse(ok)
        self.assertIn("17", reply)
        self.aircon.execute.assert_not_awaited()
        self.assertIsNone(self.router.ctx.question)

    async def test_cancel_drops_the_question(self):
        await self.ask()
        parsed, ok, reply = await self.router.parse_and_execute("算了")
        self.assertTrue(ok)
        self.assertEqual(reply, "好")
        self.aircon.execute.assert_not_awaited()
        self.assertIsNone(self.router.ctx.question)

    async def test_unrelated_answer_releases_the_question(self):
        """答非所问就撤掉问题 —— 留着的话用户下一句会被这个焦点带偏。"""
        await self.ask()
        self.llm.classify.return_value = None
        with patch("executor.intent_mod.parse", return_value=intent.Intent()):
            parsed, ok, _ = await self.router.parse_and_execute("今天天气怎么样")
        self.assertIsNone(self.router.ctx.question)
        self.aircon.execute.assert_not_awaited()


if __name__ == "__main__":
    unittest.main()
