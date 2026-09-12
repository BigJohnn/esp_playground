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
import os
import time

from config import CONFIG
from ha import HomeAssistant
import intent as intent_mod
from context import Context, Question
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

# 说完这些之后，自然没有下文 —— 开追问窗口是白开，只是平白多几秒麦克风暴露。
# 判据很朴素：这句话本身就是一个终点（关掉、停止），而不是一个中间步骤。
# 反问出去的问题能等多久。和选歌那边是同一个道理：一个悬着的问题会改写
# 后面每一句话的含义，所以必须会过期。
_QUESTION_TTL = 45.0

# 置信度低于这个值就值得花 1.5 秒问一下模型。
# 0.6 这个位置是照着 context.confidence_for 的分档挑的：
#   0.9 它确实活着            -> 不问，规则就是对的
#   0.7 不知道，但刚聊过它     -> 不问，话题是真信号
#   0.5 不知道，也没聊过       -> 问
#   0.4 确定关着，纯属兜底     -> 问
# 也就是"只在完全没有现场证据支撑时才问"，而不是"不确定就问"。
_ASK_LLM_BELOW = 0.6

_TERMINAL = {
    ("light", "off"),
    ("tivoli", "power_off"),
    ("aircon", "off"),
    ("music", "stop"),
}


class Router:
    def __init__(self, light: "LightExecutor", tivoli=None, music=None,
                 aircon=None, llm=None) -> None:
        self.light = light
        self.tivoli = tivoli
        self.music = music
        self.aircon = aircon
        self.llm = llm
        # 对话状态。替掉了原来的 last_domain —— 它记的是"我们说过什么"，
        # 而省略主语的话要靠"现在正在发生什么"补全。见 context.py 开头那段。
        self.ctx = Context()

    @property
    def last_domain(self) -> str | None:
        """老名字，留给 app.py / 日志 / 测试。现在只是 ctx.focus 的别名。"""
        return self.ctx.focus

    # 所有可能被泛化词命中的域。顺序无所谓 —— rank() 会重排。
    _DOMAINS = ("light", "music", "tivoli", "aircon")

    def _refresh_world(self) -> None:
        """把各执行器**进程内**已经知道的状态收进 Context。

        刻意只读进程内的东西，一次网络都不发：这个函数在每条命令的关键路径上，
        而控灯的预算总共才 300ms。读得到的就读（音乐队列在我们自己手里，
        Tivoli 有影子状态），读不到的留 None —— None 是"不知道"，
        rank() 会把它排在"确定关着"前面而不是后面。
        """
        w = self.ctx.world
        if self.music is not None:
            st = self.music.status()
            w.music_playing = bool(st.get("playing"))
            w.music_paused = bool(st.get("paused"))
            if w.music_playing or w.music_paused:
                w.touch("music")
        if self.tivoli is not None:
            sh = self.tivoli.shadow
            w.tivoli_powered = sh.powered
            w.tivoli_source = sh.source
        if self.aircon is not None:
            w.aircon_on = getattr(self.aircon, "believed_on", None)

    def _note_effect(self, intent: Intent) -> None:
        """一条命令做成之后，它自己就是关于世界的最新消息。

        比任何回读都可靠也便宜：我们刚把灯关了，就不必再去问灯亮不亮。
        （前提是执行器报告成功 —— 而 README §4.1.17 讲的正是"成功"不总是可信。
        所以这里只记那些**本地可验证**的动作，红外那种没有回执的不记。）
        """
        w, d, a = self.ctx.world, intent.domain, intent.action
        if d == "light":
            if a == "off":
                w.light_on = False
            elif a in ("on", "brightness", "brightness_step", "color_temp"):
                w.light_on = True
        elif d == "aircon" and a in ("on", "off"):
            w.aircon_on = (a == "on")

    def _for(self, domain: str):
        return {"light": self.light, "tivoli": self.tivoli,
                "music": self.music, "aircon": self.aircon}.get(domain)

    @property
    def asking(self) -> bool:
        """我们这一轮是不是**真的问了用户一个问题**，正等着回答。

        板子要靠它决定"追问窗口里没听懂"该不该出声：平时该闭嘴（窗口是我们
        自己开的，屋里一点动静就会走到那儿），但问了问题之后闭嘴是错的 ——
        用户答了一句，系统一声不吭，他不知道是没听见还是答错了。
        """
        if self.ctx.question is not None and not self.ctx.question.expired:
            return True
        return bool(self.music is not None and self.music.pending is not None
                    and not self.music.pending.expired)

    async def parse_and_execute(self, text: str) -> tuple[Intent, bool, str]:
        """一句话进来，走完三层意图 + 执行。返回 (最终意图, 是否执行了, 回话)。"""
        # 有问题悬着的话，先按"这是在回答"试一次。**必须在意图解析之前** ——
        # 「第二个」在普通意图里什么都不是，走到 LLM 那层还可能被瞎猜成别的。
        # answer() 认不出来会还回 None，那时候再原样往下走，一个字都不会被吞掉。
        answered = await self._answer_domain(text)
        if answered is not None:
            return answered

        if self.music is not None and self.music.pending is not None:
            answered = await self.music.answer(text)
            if answered is not None:
                ok, reply = answered
                self.ctx.record(text, "music", "choose", ok, reply)
                return (Intent(domain="music", action="choose", slots={},
                               reply=reply, raw=text, rule="choice"), ok, reply)

        self._refresh_world()
        # 候选顺序按"此刻谁活着"排，而不是按"上次说的是谁"。
        order = self.ctx.rank(self._DOMAINS)
        parsed = intent_mod.parse(text, order)

        # 选完之后回头看一眼：刚才真的分得清吗？
        # 灯开着、音乐也放着、话题又帮不上忙的时候说「关掉」，
        # rank() 还是会排出一个第一名，但那只是排序，不是把握。
        if parsed.via:
            parsed.confidence = self.ctx.confidence_for(parsed.via)
        amb = self.ctx.ambiguous(intent_mod.ambiguous_domains(parsed.rule))
        if amb:
            return self._ask_domain(text, parsed, amb)

        # 什么时候问模型：从"完全没命中"放宽到"**没把握**"。
        #
        # 原来的条件是 domain=="none"。但规则层还有一种更坏的失败：命中了，
        # 而且是错的 —— 开机就说「大一点」会稳稳地落到灯上（conf=0.5），
        # 屋里可能根本没开灯。这种"自信的错"用户看到的是设备乱动，
        # 比"这个我还不会"难受得多，却从来没机会走到模型那一层。
        #
        # 顺带纠正设计时的一个想当然：本来打算让规则和模型**并行**发车，
        # 省掉串行的等待。实现时才发现没有可并行的东西 —— 规则层是微秒级的，
        # 命中与否瞬间就知道，根本不存在"等规则的同时先让模型跑起来"这个窗口。
        # 真正省时间的是不问（高置信直接执行），而不是早问。
        if self.llm is not None and (parsed.domain == "none"
                                     or parsed.confidence < _ASK_LLM_BELOW):
            # 硬超时在 llm.py 里，超时就当没有这一层 —— 规则给的那个照常用。
            guess = await self.llm.classify(text, self.ctx)
            if guess and guess["domain"] != "none":
                _LOG.info("规则%s（%s conf=%.1f），LLM 判成 %s.%s",
                          "没中" if parsed.domain == "none" else "没把握",
                          parsed.rule or "-", parsed.confidence,
                          guess["domain"], guess["action"])
                parsed = Intent(domain=guess["domain"], action=guess["action"],
                                slots=guess.get("slots") or {},
                                reply=guess.get("reply", ""), raw=text, rule="llm",
                                confidence=0.6)

        ok, reply = await self.execute(parsed)
        self.ctx.record(text, parsed.domain, parsed.action, ok, reply)
        if ok:
            self._note_effect(parsed)
        return parsed, ok, reply

    # ------------------------------------------------------------ 反问

    # 反问时怎么称呼每个域。用用户会说的词，不是内部域名。
    _CALL = {"light": "灯", "music": "音乐", "tivoli": "音响", "aircon": "空调"}

    def _ask_domain(self, text: str, parsed: Intent, amb: list[str]) -> tuple[Intent, bool, str]:
        """真分不清是哪台设备，就问一句。

        只在**两台以上确定活着**时才走到这儿（见 Context.ambiguous），
        所以这不是"没把握就问" —— 那样会烦死人。这是"猜错的代价高于问一句"：
        灯和音乐都开着时把「关掉」猜错，用户得再说一次，还得先反应过来发生了什么。
        """
        names = [self._CALL.get(d, d) for d in amb[:3]]
        q = Question(kind="domain",
                     options=[{"domain": d, "name": n} for d, n in zip(amb, names)],
                     text=f"{'还是'.join(names)}？",
                     deadline=time.time() + _QUESTION_TTL)
        self.ctx.question = q
        # 原话留着：用户答「音乐」之后，要拿它重解一遍，而不是让用户重说一整句。
        q.options.append({"pending_text": text})
        self.ctx.record(text, parsed.domain, parsed.action, False, q.text)
        _LOG.info("「%s」在 %s 之间分不清，反问", text, "/".join(names))
        return Intent(domain="none", action="ask", slots={}, reply=q.text,
                      raw=text, rule="ask_domain"), True, q.text

    async def _answer_domain(self, text: str) -> tuple[Intent, bool, str] | None:
        """把一句话当成对"哪台设备"的回答。不是就还回 None。"""
        q = self.ctx.question
        if q is None or q.kind != "domain" or q.expired:
            return None
        t = text.strip()
        picked = None
        for opt in q.options:
            name, dom = opt.get("name"), opt.get("domain")
            if name and dom and name in t:
                picked = dom
                break
        if picked is None:
            return None
        original = next((o["pending_text"] for o in q.options if "pending_text" in o), "")
        self.ctx.question = None
        # 拿**原话**重解一遍，只是这次把答案顶到候选顺序最前面。
        # 不是直接执行"那个域的默认动作"—— 原话里可能还带着别的信息
        # （「小一点」的幅度、「下一个」的方向），重解一次比自己拼一个意图可靠。
        order = [picked] + [d for d in self.ctx.rank(self._DOMAINS) if d != picked]
        parsed = intent_mod.parse(original or text, order)
        ok, reply = await self.execute(parsed)
        self.ctx.record(text, parsed.domain, parsed.action, ok, reply)
        if ok:
            self._note_effect(parsed)
        return parsed, ok, reply

    def wants_followup(self, intent: Intent, ok: bool) -> bool:
        """这一句之后该不该不用唤醒词就继续听。

        只在**成功**之后开窗。失败意味着我们本来就没听懂，这时候开窗
        是在邀请更多混乱 —— 用户多半会重复一遍或者换个说法，
        而那恰恰是最该走完整唤醒流程、让识别从干净状态重来的时候。
        """
        if not ok or intent.domain == "none":
            return False
        return (intent.domain, intent.action) not in _TERMINAL

    async def execute(self, intent: Intent) -> tuple[bool, str]:
        if intent.domain == "none" or intent.action == "none":
            return False, intent.reply or "这个我还不会"

        # 只识别、不执行。声学回归要用：那个测试量的是"板子听没听对"，
        # 而真去执行会把一堆副作用混进来 —— 实测踩过一次：词表里有
        # 「播放我的收藏」，它执行时发现 AirPlay 链路断了就去重挂，
        # 把系统音频输出从内置喇叭切到了 Tivoli，于是后面每一条测试词
        # 都在往一个没人听的地方放，整轮唤醒 1/7，看起来像是板子坏了。
        if os.environ.get("DRY_RUN", "").strip() in ("1", "true", "yes"):
            _LOG.info("[DRY_RUN] 只解析不执行：%s.%s %s",
                      intent.domain, intent.action, intent.slots or "")
            self.ctx.world.touch(intent.domain)
            return True, intent.reply or f"[{intent.domain}.{intent.action}]"

        target = self._for(intent.domain)
        if target is None:
            return False, "这台设备还没接上"

        # 注意这里**不**更新 focus。记账统一在 parse_and_execute 的 ctx.record()
        # 里做 —— execute() 还有别的调用方（消歧答完之后的重解），
        # 两处都记就会把同一轮对话记两遍。
        return await target.execute(intent)

    def status(self) -> dict:
        out: dict = {"last_domain": self.last_domain, "context": self.ctx.as_dict()}
        if self.tivoli is not None:
            out["tivoli"] = self.tivoli.shadow.as_dict()
            out["tivoli"]["measured"] = bool(self.tivoli.conf.get("measured"))
        if self.music is not None:
            out["music"] = self.music.status()
        if self.aircon is not None:
            # 空调是唯一一个状态**读得回来**的设备，所以这里给的是真值不是影子
            out["aircon"] = {"entity": self.aircon.entity}
        return out
