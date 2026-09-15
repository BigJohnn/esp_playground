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
    async def classify(self, data):
        model = LocalLLM.__new__(LocalLLM)
        model._ask = AsyncMock(return_value=data)
        return await model.classify("模拟语句")

    async def test_valid_parameters_reach_light_hardware_adapter(self):
        class Lamp:
            P_ON, P_BRIGHT, P_CCT = 1, 2, 3

            def get(self):
                return {"brightness": 50}

            def set_many(self, props):
                self.props = props

        for action, fields, expected in [
            ("brightness", {"pct": 30}, {1: True, 2: 30}),
            ("brightness", {"pct": 0}, {1: True, 2: 1}),  # 同规则路径：最低亮度 1
            ("brightness_step", {"step": -20}, {1: True, 2: 30}),
            ("color_temp", {"kelvin": 3000}, {1: True, 3: 3000}),
            ("on", {"pct": 40, "kelvin": 4000}, {1: True, 2: 40, 3: 4000}),
        ]:
            with self.subTest(action=action, fields=fields):
                guess = await self.classify({"domain": "light", "action": action, **fields})
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
            ("light", "brightness", {}),
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

    async def test_optional_fields_and_unrelated_fields(self):
        self.assertEqual((await self.classify({"domain": "light", "action": "on"}))["slots"], {})
        result = await self.classify({"domain": "light", "action": "off", "pct": 80})
        self.assertEqual(result["slots"], {})
        result = await self.classify({"domain": "aircon", "action": "set_temp", "temp": 26})
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


if __name__ == "__main__":
    unittest.main()
