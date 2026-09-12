"""Tivoli Audio Model One Digital Gen 2 —— 红外那条链，以及影子状态。

红外是开环的，这里做的全部事情就是**用锚定把开环重新闭上**。见 airplay.py 的开头。

一条硬规矩写在最前面：这个文件里没有一个常数是猜出来然后当真的。
源循环有几档、调谐是 ▲▼ 还是 ◀▶、长按要连发多少帧 —— 全在 tivoli.json 里，
而且带一个 measured 开关。没量过就是没量过，不许假装知道 ——
猜错的后果不是报错，是每次"打开收音机"都安静地落到隔壁源上，
而我们连"落错了"都发现不了。
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from dataclasses import dataclass, field

from airplay import AirPlay
from ha import HomeAssistant
from intent import Intent, station_name

_LOG = logging.getLogger("tivoli")

_CONF_PATH = os.environ.get(
    "TIVOLI_CONF_PATH", os.path.join(os.path.dirname(__file__), "tivoli.json"))

_DEFAULTS = {
    # 源循环的顺序。SOURCE 键是**循环**不是选择 —— 想到 FM 只能从当前位置数过去，
    # 而"当前位置"只有锚定成功那一刻才是已知的。
    # 本机到底有没有 DAB 决定了 wifi->fm 是 1 步还是 2 步（R3）。
    "source_cycle": ["wifi", "bt", "dab", "fm", "aux"],
    # FM 下调谐用哪一对方向键。M6 之前这是未知的。
    "tune_keys": "up_down",              # up_down | left_right
    # 两次红外之间等多久。设备要时间处理，连发太快会被吞。
    "ir_gap_ms": 600,
    # 切源之后等多久才认为它稳定了
    "source_settle_ms": 1500,
    # 长按存台：连发多少个 NEC 帧（板子侧按 108ms 周期发）。R4。
    "hold_frames": 18,
    # 按完 POWER 等多久再去反向锚定确认
    "power_settle_ms": 3000,
    # 所有常数都量过了吗？没有的话 FM 那条链会拒绝动作，而不是瞎按。
    "measured": False,
}


def load_conf(path: str | None = None) -> dict:
    conf = dict(_DEFAULTS)
    try:
        with open(path or _CONF_PATH, encoding="utf-8") as f:
            conf.update(json.load(f))
    except FileNotFoundError:
        pass
    except Exception as exc:  # noqa: BLE001
        _LOG.warning("tivoli.json 读不了（%s），用默认值 —— 但 measured 仍然是 False", exc)
    return conf


@dataclass
class Shadow:
    """我们**以为**设备处在什么状态。每一个字段都可能是错的，所以每个都带时间戳。

    影子状态不是真相，是"上一次我们确知的真相 + 之后我们做过什么"。
    锚定成功是唯一能把它归零的事件；实体遥控器一按，它立刻就开始腐化。
    """
    source: str | None = None          # "wifi" / "fm" / ... / None=不知道
    anchored_at: float | None = None   # 上次锚定成功
    preset: int | None = None          # 上次我们切到的预设位
    powered: bool | None = None
    updated_at: float = field(default_factory=time.time)

    def set(self, **kw) -> None:
        for k, v in kw.items():
            setattr(self, k, v)
        self.updated_at = time.time()

    def stale_after(self, seconds: float) -> bool:
        return self.anchored_at is None or (time.time() - self.anchored_at) > seconds

    def as_dict(self) -> dict:
        return {"source": self.source, "preset": self.preset, "powered": self.powered,
                "anchored_s_ago": None if self.anchored_at is None
                else round(time.time() - self.anchored_at, 1)}


class TivoliExecutor:
    """红外 + 锚定。音乐那条链在 player.py，两者共用这里的 shadow 和 airplay。"""

    def __init__(self, ha: HomeAssistant, air: AirPlay, conf: dict | None = None) -> None:
        self.ha = ha
        self.air = air
        self.conf = conf or load_conf()
        self.shadow = Shadow()
        self._lock = asyncio.Lock()

    # ---------- 红外 ----------

    async def press(self, button: str, times: int = 1) -> bool:
        """按一下红外键。发出去就返回 —— 没有回执，这是红外的本性。"""
        entity = f"button.tivoli_ir_{button}"
        gap = self.conf["ir_gap_ms"] / 1000.0
        for i in range(times):
            try:
                await self.ha.call("button", "press", entity_id=entity)
            except Exception as exc:  # noqa: BLE001
                _LOG.warning("红外 %s 发不出去：%s", entity, exc)
                return False
            if i + 1 < times:
                await asyncio.sleep(gap)
        return True

    # ---------- 锚定 ----------

    @property
    def source_gap(self) -> int | None:
        """从 wifi 数到 fm 要按几次 SOURCE。数不出来就返回 None。"""
        cycle = self.conf["source_cycle"]
        if "wifi" not in cycle or "fm" not in cycle:
            return None
        return (cycle.index("fm") - cycle.index("wifi")) % len(cycle)

    async def anchor(self) -> bool:
        ok = await self.air.anchor()
        if ok:
            self.shadow.set(source="wifi", powered=True, anchored_at=time.time())
        else:
            # 锚定失败**不代表关机** —— 也可能是它开着但停在 FM 源上（那时 RAOP 接收端
            # 没在跑）。所以这里只把 source 标成未知，不敢断言 powered=False。
            self.shadow.set(source=None)
        return ok

    async def ensure_wifi(self) -> tuple[bool, str]:
        """把设备弄到"开着 + 链路通"这个已知状态上。

        **按 POWER 之前必须先确认设备真的不可达。** 这一条是踩出来的，而且踩得很准：
        早先这里的逻辑是"锚定失败 -> 按 POWER 试试看"，看起来合理 ——
        但 anchor() 失败有两个完全不同的原因，一个是设备关着，另一个是
        macOS 这头的 AirPlay 链路断了（**只要听一次收音机就会断**，是主路径不是异常）。
        两者混在一起的后果是：用户切去听 FM，再说一句"打开收音机"，
        代码读到锚定失败、按下 POWER，**把一台开着的收音机关掉了**。

        POWER 是 toggle，在错误前提下按它是破坏性的，而且没有回执能发现按错了。
        所以判据必须是"网络层面真的够不着"（reachable），不是"推不了流"（anchor）。
        """
        if await self.anchor():
            return True, ""

        # 锚定失败了，先分清是哪一种失败
        if await self.air.reachable():
            # 设备在电、在网 —— 只是链路没挂上，而 anchor() 里已经试过重挂并且失败了。
            # 这种情况**绝不能**碰 POWER：它是好的，按下去只会把它弄坏。
            return False, "接不上音响的 AirPlay，检查一下辅助功能权限给了没有"

        # 到这儿意味着要走"按 POWER + 等它重新入网"这条路，而那是**半分钟起步**的
        # （实测开机到重新上网 28 秒）。这条路不能同步做完再回话 ——
        # 用户说完「打开收音机」之后会盯着一分钟的沉默，然后认定坏了（实测 57 秒）。
        # 所以调用方要能提前知道"这次会很久"，先说一句话安抚，剩下的后台做。
        _LOG.info("设备网络层面不可达，按 POWER 唤醒（要等它重新入网，约半分钟）")
        if not await self.press("power"):
            return False, "红外发不出去，检查一下 tivoli-ir 那块板子在不在线"
        # 开机到重新入网实测要 28 秒，power_settle_ms 那三秒远远不够，所以这里轮询等
        deadline = time.time() + 40
        while time.time() < deadline:
            await asyncio.sleep(3)
            if await self.air.reachable():
                break
        else:
            # 一直没上线：可能它本来就是开着但掉网了，而我们刚把它关了。按回去。
            await self.press("power")
            return False, "音响连不上，可能是掉网了"
        if await self.anchor():
            return True, ""
        return False, "音响开起来了，但 AirPlay 链路挂不上"

    async def ensure_fm(self) -> tuple[bool, str]:
        """锚定 -> 从 WiFi 数 k 次 SOURCE 落到 FM。整套方案的核心动作。"""
        if not self.conf.get("measured"):
            return False, ("源循环还没实测过，我不想瞎按 —— "
                           "先做一遍出声实测，把常数量出来")
        k = self.source_gap
        if k is None:
            return False, "源循环配错了，找不到 wifi 或 fm"
        if self.shadow.source == "fm" and not self.shadow.stale_after(300):
            return True, ""      # 五分钟内锚过并且我们没动过源，认账
        ok, why = await self.ensure_wifi()
        if not ok:
            return False, why
        if k and not await self.press("source", times=k):
            return False, "红外发不出去"
        await asyncio.sleep(self.conf["source_settle_ms"] / 1000.0)
        self.shadow.set(source="fm")
        return True, ""

    async def power_off(self) -> tuple[bool, str]:
        """关机。POWER 是 toggle，所以必须验 —— 验的办法是**反向**用锚定。

        按完之后锚定失败 = 关掉了。锚定成功 = 原来它是关着的，我们刚把它打开了，
        再按一次关回去。这是白捡的一个确认通道：同一个原语，成功和失败都有意义。
        """
        if not await self.press("power"):
            return False, "红外发不出去"
        await asyncio.sleep(self.conf["power_settle_ms"] / 1000.0)
        if not await self.air.anchor():
            self.shadow.set(source=None, powered=False, anchored_at=None)
            return True, "音响关了"
        # 锚上了，说明这一下是把它**打开**了
        _LOG.info("按完 POWER 反而锚上了 —— 原来它是关着的，再按一次关回去")
        await self.press("power")
        self.shadow.set(source=None, powered=False, anchored_at=None)
        return True, "本来就是关的，我给你关回去了"

    # ---------- 动作 ----------

    async def needs_cold_start(self) -> bool:
        """这次操作要不要走"开机 + 等入网"那条慢路。"""
        return not await self.air.reachable()

    async def execute(self, intent: Intent) -> tuple[bool, str]:
        async with self._lock:
            return await self._execute(intent)

    async def _execute(self, intent: Intent) -> tuple[bool, str]:
        a = intent.action

        if a == "power_on":
            ok, why = await self.ensure_wifi()
            return (True, "音响开了") if ok else (False, why)

        if a == "power_off":
            return await self.power_off()

        if a == "radio_on":
            ok, why = await self.ensure_fm()
            if not ok:
                return False, why
            # 上次听的那个预设。第一次用没有记录，就停在设备自己记得的地方 ——
            # 那也是个合理的默认，用户上次就是听的它。
            if self.shadow.preset:
                await self.press(f"preset_{self.shadow.preset}")
                return True, f"收音机开了，还是{station_name(self.shadow.preset)}"
            return True, "收音机开了"

        if a == "preset_recall":
            n = intent.slots.get("preset")
            if not n:
                return False, "第几个预设？"
            ok, why = await self.ensure_fm()
            if not ok:
                return False, why
            if not await self.press(f"preset_{n}"):
                return False, "红外发不出去"
            self.shadow.set(preset=n)
            return True, f"切到{station_name(n)}"

        if a == "preset_save":
            n = intent.slots.get("preset")
            if not n:
                return False, "存到第几个预设？"
            ok, why = await self.ensure_fm()
            if not ok:
                return False, why
            if not await self._set_hold_frames():
                return False, "设不了长按帧数"
            # 长按是 **toggle**，不是覆盖（2026-09-11 实测：位子空则存、位子满则删）。
            # 所以要按原来的占用状态决定按几下：满的先删一次再存，空的直接存。
            # 这笔账只有我们自己操作时才准 —— 有人拿实体遥控器动过预设，它就开始腐化，
            # 而腐化的后果是"说存台结果把台删了"。所以宁可多问一句也不要猜错。
            if self.occupied(n):
                if not await self.press(f"preset_{n}_hold"):
                    return False, "红外发不出去"
                self.set_occupied(n, False)
                await asyncio.sleep(self.conf["ir_gap_ms"] / 1000.0)
            if not await self.press(f"preset_{n}_hold"):
                return False, "红外发不出去"
            self.set_occupied(n, True)
            self.shadow.set(preset=n)
            return True, f"存到预设{n}了"

        if a == "station_step":
            # 说明书查过了：自动搜台（Autoscan）**只能双击机身旋钮**，遥控器上没有这个键
            # （遥控器按键全集：MUTE/ALARM/方向键+SELECT/音量±/预设1-6/SETTINGS/SOURCE/SLEEP/POWER）。
            # 所以红外做不到"下一个电台"，方向键只是 ±tune_step_mhz 的微调。
            # 与其假装能换台、实际只挪了半格，不如说实话，并把人引到真正能用的办法上。
            step = intent.slots.get("step", 1)
            ok, why = await self.ensure_fm()
            if not ok:
                return False, why
            up, down = ("up", "down") if self.conf["tune_keys"] == "up_down" else ("right", "left")
            # 走一个频道位要按几下：国内台间隔 0.1MHz，而出厂步进是 0.05
            per = max(1, round(0.1 / float(self.conf.get("tune_step_mhz") or 0.05)))
            if not await self.press(up if step > 0 else down, times=per * abs(step)):
                return False, "红外发不出去"
            self.shadow.set(preset=None)   # 挪过频率之后，预设的记账作废
            return True, ("往上挪了一格，遥控器上没有搜台键，只能微调"
                          if step > 0 else "往下挪了一格")

        if a == "volume_step":
            step = int(intent.slots.get("step", 1))
            # AirPlay 链路在的时候走网络，不走红外。两个理由：
            #   快 —— 红外要连按 3 下、每下之间还得留 600ms 给设备反应，实测 1370ms；
            #         网络设一次绝对值，200ms 不到。
            #   准 —— 红外的 VOL± 是相对的、开环的，按了几次全靠记账，
            #         而记账一定会跟实体遥控器的操作对不上。
            # 而且 AirPlay 会把设备音量**同步回** macOS 的系统音量（实测：
            # 发了 3 下红外 VOL-，系统音量跟着从 18 掉到 12），
            # 所以读回来的当前值是真的，不是我们自己记的账。
            if await self.air.linked():
                cur = await self.air.volume()
                if cur is not None:
                    # intent 里的 step 是"按几下红外"的意思（±3）。
                    # 换算到绝对音量不能用固定步长：实测音量 13 时降一档变成 4，
                    # 相对降了 70%，几乎等于静音。人对响度的感知是对数的，
                    # 所以低音量时要走小步。按当前值的比例调，再兜一个最小步长。
                    delta = max(3.0, abs(cur) * 0.25) * (1 if step > 0 else -1)
                    target = max(0.0, min(100.0, cur + delta))
                    if await self.air.set_volume(target):
                        return True, "调大了" if step > 0 else "调小了"
            key = "volume_up" if step > 0 else "volume_down"
            if not await self.press(key, times=abs(step)):
                return False, "红外发不出去"
            return True, "调大了" if step > 0 else "调小了"

        if a == "mute":
            if not await self.press("mute"):
                return False, "红外发不出去"
            return True, "静音了"

        return False, intent.reply or "这个我还不会"

    # ---------- 预设占用记账 ----------

    def occupied(self, n: int) -> bool:
        return bool((self.conf.get("preset_occupied") or {}).get(str(n), True))

    def set_occupied(self, n: int, value: bool) -> None:
        """记账要落盘 —— 进程重启之后如果忘了哪些位子是满的，
        下一次"存台"就会变成"删台"，而这个错用户是看不见的（屏幕我们读不到）。"""
        table = dict(self.conf.get("preset_occupied") or {})
        table[str(n)] = bool(value)
        self.conf["preset_occupied"] = table
        try:
            with open(_CONF_PATH, encoding="utf-8") as f:
                raw = json.load(f)
            raw["preset_occupied"] = table
            with open(_CONF_PATH, "w", encoding="utf-8") as f:
                json.dump(raw, f, ensure_ascii=False, indent=2)
                f.write("\n")
        except Exception as exc:  # noqa: BLE001 - 落盘失败不该让这次操作算失败
            _LOG.warning("预设占用表存不下去：%s", exc)

    async def _set_hold_frames(self) -> bool:
        """把长按的帧数推给板子。板上是个 template number，改它不用重烧。"""
        try:
            await self.ha.call("number", "set_value",
                               entity_id="number.tivoli_ir_hold_frames",
                               value=int(self.conf["hold_frames"]))
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("设 hold_frames 失败：%s", exc)
            return False
        return True
