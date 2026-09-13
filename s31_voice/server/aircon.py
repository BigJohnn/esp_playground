"""空调。这是这套系统里**唯一一个不开环的设备**。

Tivoli 的全部麻烦来自「只能发相对命令、读不回状态」：POWER 是 toggle、
SOURCE 是 cycle，按之前不知道在哪、按之后也不知道到了哪。空调正好相反 ——
Coolix 协议每一帧都把**整机状态**（开关＋模式＋温度＋风速）打包重发，
所以每条命令都是绝对的：说 26 度就是 26 度，不用记账、不会跟实体遥控器走偏。

（2026-09-12 从遥控器实测解出来的：按「温度+」再按「温度−」之后，
第三帧和第一帧**逐位相同** —— 因为温度回到了原值，状态帧当然重复。
这就是"它发的是状态不是动作"的直接证据。）

所以这里没有影子状态，也不需要。ESPHome 的 coolix 组件还开了接收，
实体遥控器一按 HA 就跟着更新 —— 状态是真的读得回来的。
"""
from __future__ import annotations

import logging
import time

from ha import HomeAssistant
from intent import Intent

_LOG = logging.getLogger("aircon")

ENTITY = "climate.tivoli_ir_air_conditioner"

# 温度范围按遥控器的物理能力来，不是按我们想当然
MIN_TEMP, MAX_TEMP = 17, 30

_MODE_CN = {"cool": "制冷", "heat": "制热", "dry": "除湿",
            "fan_only": "送风", "heat_cool": "自动", "off": "关"}


class AirconExecutor:
    def __init__(self, ha: HomeAssistant, entity: str = ENTITY) -> None:
        self.ha = ha
        self.entity = entity
        # 我们自己刚下发的温度 + 时间戳。
        # 需要它是因为 HA 的状态是**滞后的** —— 命令经 HA -> ESPHome -> 红外发出去，
        # 再由 ESPHome 回报新状态给 HA，这一圈要一两秒。连着说「太热了」「太冷了」时，
        # 第二句读回的还是第一句之前的温度，于是从错的基准上加减（实测：
        # 26 -> 说"太热" -> 读到 24(旧) -> 设 23 -> 说"太冷" -> 又读到 24 -> 设 25）。
        # 这不是 Tivoli 那种"影子状态可能骗人"：Coolix 是绝对状态帧，
        # 我们下发什么它就是什么，所以这个值在短时间内比 HA 更可信。
        self._last_set: tuple[int, float] | None = None
        # 我们以为它开着没有。给对话层的消歧用（「大一点」是温度还是音量），
        # 不是给控制用 —— 控制走的是绝对状态帧，从来不需要知道当前状态。
        # None = 不知道（刚启动、或只被实体遥控器动过）。
        self.believed_on: bool | None = None
        # 上一次**开关机**的时刻。温度调节不算 —— 伤机器的是压缩机短周期启停，
        # 不是改目标温度。
        self._last_power_at: float = 0.0

    async def _state(self) -> dict:
        """读回当前状态。**这一步在 Tivoli 上是做不到的**，在这儿可以。"""
        try:
            r = await self.ha._client.get(f"/api/states/{self.entity}")
            r.raise_for_status()
            return r.json()
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("读空调状态失败：%s", exc)
            return {}

    async def _current_temp(self) -> int | None:
        """当前目标温度。短时间内优先用我们自己刚下发的值（见 _last_set 的注释）。"""
        if self._last_set is not None:
            temp, at = self._last_set
            if time.time() - at < 60:
                return temp
        st = await self._state()
        cur = (st.get("attributes") or {}).get("temperature")
        return int(cur) if cur is not None else None

    async def _call(self, service: str, **data) -> bool:
        try:
            await self.ha.call("climate", service, entity_id=self.entity, **data)
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("climate.%s 失败：%s", service, exc)
            return False
        return True

    # 两次开关机之间至少隔这么久。
    #
    # 这不是软件洁癖，是硬件约束：压缩机短周期启停（short cycling）会真的把它做坏 ——
    # 刚停机时冷媒两侧压力还没平衡，这时候重启，压缩机是在带着压差硬启动。
    # 整机厂通常在机内做 3 分钟延时保护，但我们是**绕过面板直接打红外**的，
    # 不能假定那层保护一定挡得住我们发的每一帧。
    #
    # 加这条是因为一次自己造成的事故：三台设备的体检脚本里连着跑了两轮
    # 「打开空调 … 关闭空调」，几十秒内开关了四次。用户当场喊停。
    # 光记住"以后别这么干"是不够的 —— 约束要写进代码，否则下一个脚本还会犯。
    #
    # 只挡**反向**的那次：连说两次「关空调」是无害的（Coolix 发的是绝对状态帧，
    # 已经关着再关一次还是关着），挡它只会让人觉得系统在闹脾气。
    _POWER_GUARD_S = 180.0

    def _power_guard(self, want_on: bool) -> str | None:
        """这次开关机该不该拦下来。返回拦下来的说辞，放行就返回 None。"""
        if self.believed_on is None or self.believed_on == want_on:
            return None            # 不知道，或者本来就是这个状态 —— 不是一次翻转
        left = self._POWER_GUARD_S - (time.time() - self._last_power_at)
        if left <= 0:
            return None
        return (f"空调{int(left)}秒前刚{'关' if want_on else '开'}过，"
                f"这么快反过来会伤压缩机，等{int(left)}秒再说")

    async def execute(self, intent: Intent) -> tuple[bool, str]:
        a = intent.action
        slots = intent.slots

        if a == "off":
            blocked = self._power_guard(False)
            if blocked:
                return False, blocked
            # 注意这不是"按一下电源键"，是把状态设成 off。
            # 已经关着的时候再说一次"关空调"，结果还是关着 —— 不会反过来打开。
            # 这正是状态帧相对 toggle 的好处，也是用户特意提过的那个顾虑。
            if not await self._call("set_hvac_mode", hvac_mode="off"):
                return False, "空调没反应，检查一下红外板"
            self._last_set = None      # 关机之后温度记账作废
            if self.believed_on is not False:
                self._last_power_at = time.time()
            self.believed_on = False
            return True, "空调关了"

        if a == "on":
            blocked = self._power_guard(True)
            if blocked:
                return False, blocked
            mode = slots.get("mode") or "cool"
            if not await self._call("set_hvac_mode", hvac_mode=mode):
                return False, "空调没反应，检查一下红外板"
            if self.believed_on is not True:
                self._last_power_at = time.time()
            self.believed_on = True
            temp = slots.get("temp")
            if temp is not None:
                await self._call("set_temperature", temperature=int(temp))
                self._last_set = (int(temp), time.time())
                return True, f"空调{_MODE_CN.get(mode, mode)}{int(temp)}度"
            return True, f"空调开了，{_MODE_CN.get(mode, mode)}"

        if a == "set_temp":
            temp = slots.get("temp")
            if temp is None:
                return False, "要调到多少度？"
            temp = max(MIN_TEMP, min(MAX_TEMP, int(temp)))
            st = await self._state()
            # 关着的时候直接设温度，机器收不到 —— 先开起来。
            if (st.get("state") or "off") == "off":
                # 这里也是一次开机，同样要过压缩机保护那道闸
                blocked = self._power_guard(True)
                if blocked:
                    return False, blocked
                await self._call("set_hvac_mode", hvac_mode="cool")
                self._last_power_at = time.time()
                self.believed_on = True
            if not await self._call("set_temperature", temperature=temp):
                return False, "空调没反应"
            self._last_set = (temp, time.time())
            return True, f"空调调到{temp}度"

        if a == "temp_step":
            step = int(slots.get("step", 1))
            cur = await self._current_temp()
            if cur is None:
                return False, "读不到空调现在几度"
            target = max(MIN_TEMP, min(MAX_TEMP, cur + step))
            if target == cur:
                return True, f"已经是{cur}度了，{'调不上去' if step > 0 else '调不下来'}"
            if not await self._call("set_temperature", temperature=target):
                return False, "空调没反应"
            self._last_set = (target, time.time())
            return True, f"{'调高' if step > 0 else '调低'}到{target}度"

        if a == "status":
            st = await self._state()
            mode = st.get("state") or "unknown"
            if mode == "off":
                return True, "空调关着"
            temp = (st.get("attributes") or {}).get("temperature")
            return True, f"空调{_MODE_CN.get(mode, mode)}，{int(temp) if temp else '?'}度"

        return False, intent.reply or "这个我还不会"
