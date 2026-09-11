"""把 Intent 落到实际设备上：灯的执行器，以及三台设备的分发器（Router）。

灯的部分（LightExecutor）

两条路径，优先本地：
  1. miIO 局域网直连  —— 实测 ~300ms，不依赖外网、不受 Docker 网络模式影响
  2. Home Assistant   —— 兜底。经小米云，实测 ~1000ms（首次冷启动 ~6s）

之所以还留着 HA 这条路：token 会因为重新配网而失效，灯泡也可能被换到别的网段；
这时候有个能用但慢的通道，比整套哑掉强。
"""
from __future__ import annotations

import asyncio
import logging

from config import CONFIG
from ha import HomeAssistant
import intent as intent_mod
from intent import Intent

_LOG = logging.getLogger("exec")


class LightExecutor:
    def __init__(self, ha: HomeAssistant) -> None:
        self.ha = ha
        self._entity: str | None = None
        self._local = None
        self._local_failures = 0

    # ---------- 本地路径 ----------

    def _ensure_local(self):
        if self._local is not None:
            return self._local
        if not (CONFIG.miio_did and CONFIG.miio_token):
            return None
        from miio import MiioDevice, MiotLight

        self._local = MiotLight(MiioDevice(did=CONFIG.miio_did, token=CONFIG.miio_token))
        return self._local

    async def keep_warm(self, period_s: int = 40) -> None:
        """后台把 miIO 握手一直保持有效。

        实测：一次控灯里，如果握手过期了要先重握，那一下就是几百毫秒 ——
        而它完全落在用户等待的关键路径上。提前在后台握好，控灯就只剩一个 UDP 往返。
        周期取小于 miio.HANDSHAKE_TTL，代价是每 40 秒一个几十字节的包。
        """
        while True:
            light = self._ensure_local()
            if light is not None:
                await asyncio.to_thread(light.dev.warm)
            await asyncio.sleep(period_s)

    async def _run_local(self, intent: Intent) -> bool:
        light = self._ensure_local()
        if light is None:
            return False
        try:
            await asyncio.to_thread(self._apply_local, light, intent)
        except Exception as exc:  # noqa: BLE001 - 本地失败就退回 HA，不该让整条链断掉
            self._local_failures += 1
            _LOG.warning("miIO 直控失败(%d 次): %s，回退到 Home Assistant",
                         self._local_failures, exc)
            return False
        self._local_failures = 0
        return True

    @staticmethod
    def _apply_local(light, intent: Intent) -> None:
        if intent.action == "off":
            light.turn_off()
            return
        if intent.action == "toggle":
            light.turn_off() if light.get().get("on") else light.turn_on()
            return

        # 其余动作都归结为"开灯 + 可选的亮度/色温"，合并成一次 set_properties 发出去，
        # 否则每多一个属性就多 ~300ms 的 UDP 往返。
        props: dict[int, object] = {light.P_ON: True}

        if intent.action == "brightness_step":
            # 相对调节绕不开一次读取，这是唯一需要两次往返的情况
            cur = light.get().get("brightness") or 50
            props[light.P_BRIGHT] = max(1, min(100, cur + (intent.brightness_step_pct or 0)))
        elif intent.brightness_pct is not None:
            props[light.P_BRIGHT] = max(1, min(100, intent.brightness_pct))

        if intent.color_temp_kelvin is not None:
            props[light.P_CCT] = max(2700, min(6500, intent.color_temp_kelvin))

        light.set_many(props)

    # ---------- HA 路径 ----------

    async def entity(self) -> str | None:
        if self._entity is None:
            self._entity = await self.ha.resolve_light()
            if self._entity:
                _LOG.info("HA 兜底目标: %s", self._entity)
        return self._entity

    async def _run_ha(self, intent: Intent) -> tuple[bool, str]:
        entity = await self.entity()
        if not entity:
            return False, "我还没找到灯"

        data: dict = {"entity_id": entity}
        service = "turn_on"
        if intent.action == "off":
            service = "turn_off"
        elif intent.action == "toggle":
            service = "toggle"
        else:
            if intent.brightness_pct is not None:
                data["brightness_pct"] = intent.brightness_pct
            if intent.brightness_step_pct is not None:
                data["brightness_step_pct"] = intent.brightness_step_pct
            if intent.color_temp_kelvin is not None:
                data["color_temp_kelvin"] = intent.color_temp_kelvin

        try:
            await self.ha.call("light", service, **data)
        except Exception as exc:  # noqa: BLE001
            _LOG.exception("light.%s 失败", service)
            return False, f"控制失败：{exc}"
        return True, intent.reply

    # ---------- 对外 ----------

    async def execute(self, intent: Intent) -> tuple[bool, str]:
        """返回 (是否执行了动作, 给用户的回话)。"""
        if intent.action == "none":
            return False, intent.reply

        if await self._run_local(intent):
            return True, intent.reply
        return await self._run_ha(intent)


# ---------------------------------------------------------------------------
# 设备注册表
#
# 从"只有一盏灯"扩到三台设备之后，多出来的两件事都不属于任何单个执行器：
#   1. 泛化词消歧 —— "关掉"是关灯还是关音响，得记着上一次操作的是谁
#   2. 第三层兜底 —— 规则和拼音都没中的时候才问本地 LLM，而且带硬超时
# 所以有了 Router。它不碰任何硬件，只做分发和这两件事。
# ---------------------------------------------------------------------------

class Router:
    def __init__(self, light: "LightExecutor", tivoli=None, music=None, llm=None) -> None:
        self.light = light
        self.tivoli = tivoli
        self.music = music
        self.llm = llm
        # 上一次**成功**操作的设备。失败的不算 —— 一条没执行成的命令不该改变
        # 后面那句"关掉"的含义。
        self.last_domain: str | None = None

    def _for(self, domain: str):
        return {"light": self.light, "tivoli": self.tivoli, "music": self.music}.get(domain)

    async def parse_and_execute(self, text: str) -> tuple[Intent, bool, str]:
        """一句话进来，走完三层意图 + 执行。返回 (最终意图, 是否执行了, 回话)。"""
        parsed = intent_mod.parse(text, self.last_domain)

        if parsed.domain == "none" and self.llm is not None:
            # 规则层和拼音层都放弃了，才轮到模型。硬超时在 llm.py 里，
            # 超时就当没有这一层 —— 回"这个我还不会"，跟以前一样。
            guess = await self.llm.classify(text)
            if guess:
                parsed = Intent(domain=guess["domain"], action=guess["action"],
                                slots=guess.get("slots") or {},
                                reply=guess.get("reply", ""), raw=text, rule="llm")
                _LOG.info("规则没中，LLM 判成 %s.%s", parsed.domain, parsed.action)

        ok, reply = await self.execute(parsed)
        return parsed, ok, reply

    async def execute(self, intent: Intent) -> tuple[bool, str]:
        if intent.domain == "none" or intent.action == "none":
            return False, intent.reply or "这个我还不会"

        target = self._for(intent.domain)
        if target is None:
            return False, "这台设备还没接上"

        ok, reply = await target.execute(intent)
        if ok:
            self.last_domain = intent.domain
        return ok, reply

    def status(self) -> dict:
        out: dict = {"last_domain": self.last_domain}
        if self.tivoli is not None:
            out["tivoli"] = self.tivoli.shadow.as_dict()
            out["tivoli"]["measured"] = bool(self.tivoli.conf.get("measured"))
        if self.music is not None:
            out["music"] = self.music.status()
        return out
