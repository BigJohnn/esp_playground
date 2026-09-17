"""本地 LLM，作意图层的第三层兜底。Ollama + Qwen3-4B(4bit)。

它的位置**只在规则层和拼音层都没命中之后**，而且带硬超时。理由：
家里的指令空间小到规则能覆盖九成，规则的延迟是微秒级、结果可预测、断网可用；
把每句话都过一遍模型，等于给九成能秒回的请求平白加一秒延迟，
还引入一个"同一句话今天这么解明天那么解"的不确定性。

选 4B 不选 7B 是被内存逼的（roadmap R6）：
    SenseVoice ≈1.2G + Kokoro ≈0.5G + Qwen3-4B ≈3G + HA 容器 ≈1G + 系统 ≈4G ≈ 10G / 16G
7B 会把余量吃光，而这台机器还要开 Chrome。

两个用途，都很窄：
    keywords()  —— "放点适合睡觉的歌" -> 一个能拿去搜歌单的词
    classify()  —— 规则没认出来的句子，看看能不能归到某个域的某个动作上
窄是有意的。让模型做开放式对话，它就会开始编设备、编动作，而下游全是真会动的硬件。
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import time

import httpx

_LOG = logging.getLogger("llm")


def _ground(ctx) -> str:
    """把 Context 压成几行给模型看的现场说明。

    只给**和这句话可能相关**的东西，不倒整个状态树：提示词每长一点，
    首 token 就慢一点，而这一层总共只有 1.5 秒预算。
    """
    if ctx is None:
        return "（不知道）"
    w, lines = ctx.world, []
    if w.music_playing or w.music_paused:
        song = ""
        for t in reversed(ctx.turns):
            if t.domain == "music" and t.ok:
                song = f"（刚才：{t.reply}）"
                break
        lines.append(("音乐正在播放" if w.music_playing else "音乐暂停中") + song)
    else:
        lines.append("没有在放音乐")
    if w.light_on is not None:
        lines.append("灯开着" if w.light_on else "灯关着")
    if w.aircon_on is not None:
        lines.append("空调开着" if w.aircon_on else "空调关着")
    if w.tivoli_source:
        lines.append(f"音响在{w.tivoli_source}源上")
    recent = [f"用户说「{t.text}」，我们{t.reply or t.action}"
              for t in list(ctx.turns)[-3:] if t.text]
    if recent:
        lines.append("最近几句：" + "；".join(recent))
    return "\n".join("- " + x for x in lines)

_HOST = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434").rstrip("/")
_MODEL = os.environ.get("LLM_MODEL", "qwen3:4b")
# roadmap 里定的硬超时。超了就当没有这一层 —— 它是兜底，不是依赖。
# 原来是 1.5s。放宽到 2.6s，因为这一层的**职责变了**：
# 以前它只在"完全没听懂"时兜底，超时的代价是一句"这个我还不会" —— 很便宜。
# 现在它还兼管"规则命中了但没有任何现场证据支撑"的情况，超时的代价是
# **照着一个没根据的猜测去动真设备**。为后者多等一秒是划算的。
# 实测带上现场信息后是 1.46~1.81s，1.5s 会砍掉一半的正确答案。
_TIMEOUT = float(os.environ.get("LLM_TIMEOUT", "2.6"))
# 让模型常驻多久。设 0 的话每次都要重新加载权重（首 token 要好几秒），
# 设 -1 就永远不卸载（16G 的机器上不合适）。15 分钟是个折中。
_KEEP_ALIVE = os.environ.get("LLM_KEEP_ALIVE", "15m")

# 输出用 Ollama 的**结构化输出**（format=JSON schema）来约束，不是靠提示词求它。
# 这不是锦上添花，是这一层能不能存在的前提：Qwen3 是思考型模型，
# 放开了让它自由生成，它一定先写一段"首先，用户说……"，120 个 token 都花在
# 推理上，正文一个字都没吐出来 —— 实测三种写法（think:false、/no_think、默认）
# 全都这样，2.9 秒还没进入正题。给了 schema 之后它直接吐 JSON：
#     太亮了受不了            0.98s  -> light.brightness_step
#     换个别的歌吧            0.57s  -> music.next
# 稳稳落在 1.5 秒预算里。
_SCHEMA = {
    "type": "object",
    "properties": {
        "domain": {"type": "string",
                   "enum": ["light", "tivoli", "music", "aircon", "none"]},
        "action": {"type": "string"},
        "preset": {"type": "integer"},     # 预设位 1-6
        "step": {"type": "integer"},       # 相对增减
        "pct": {"type": "integer"},        # 亮度百分比
        "kelvin": {"type": "integer"},     # 色温
        "query": {"type": "string"},       # 要搜的歌/歌手
        "temp": {"type": "integer"},       # 空调目标温度
        "reply": {"type": "string"},
    },
    "required": ["domain", "action"],
}

_KW_SCHEMA = {"type": "object", "properties": {"keyword": {"type": "string"}},
              "required": ["keyword"]}

# 提示词里带**现场情况**，这是这一层从"同义句改写器"变成"能解指代"的关键。
# 在此之前模型只看得到孤零零一句话，于是「再来一个」「不是这首」「对就是它」
# 这类句子它结构上就不可能答对 —— 那些话的意思根本不在话里面。
_CLASSIFY_PROMPT = """把用户这句话归到下面某一条动作上。

action 只能从下面这些词里原样挑一个，不许改写、不许加括号：
light   on  off  brightness  brightness_step  color_temp
tivoli  power_on  power_off  radio_on  preset_recall  preset_save  station_step  volume_step  set_volume  mute
music   play  play_favorites  next  prev  pause  resume  stop  now_playing
aircon  on  off  set_temp  temp_step  status
none    none

需要的话另外填这几个字段：pct(亮度 0-100) kelvin(色温 2700-6500) step(增减，如 20/-20/1/-1/3/-3)
preset(预设位 1-6) query(要搜的歌名或歌手) temp(空调温度 17-30)。
reply 是给用户听的中文回话，不超过 12 个字。

现在的情况：
%s

用户说：%s

只有当这句话**和这几台设备都无关**时（比如问天气、闲聊）才答 domain=none。
句子含糊但看得出是在说其中某台设备的，就照现在的情况挑最合理的那一条。

（写过一版"拿不准就答 none"，实测反了：模型明明听懂了 ——
「这首太吵了」它回的 reply 是"太吵了，调低点？"—— 却照着那句话答 none，
五句里五句作废。编造不存在的动作有白名单挡着，那才是真风险；
让一个答得对的模型闭嘴不是安全，只是失败。）"""

_KEYWORDS_PROMPT = """用户想听歌，但没说具体歌名。把这句话变成一个用来搜网易云歌单的关键词，
2 到 8 个汉字。

用户说：%s"""

_ALLOWED = {
    "light": {"on", "off", "toggle", "brightness", "brightness_step", "color_temp"},
    "tivoli": {"power_on", "power_off", "radio_on", "preset_recall", "preset_save",
               "station_step", "volume_step", "set_volume", "mute"},
    "music": {"play", "play_favorites", "next", "prev", "pause", "resume", "stop",
              "now_playing"},
    "aircon": {"on", "off", "set_temp", "temp_step", "status"},
}
# action -> 唯一属于哪个域。模型经常把域搞错但动作说对（"把收音机声音关小"
# 它答 music.volume_step）——  volume_step 只有 tivoli 有，那就是 tivoli。
# 与其因为域错了整条丢掉，不如按动作把它修回来。
_ACTION_HOME = {a: d for d, acts in _ALLOWED.items() for a in acts
                if sum(a in x for x in _ALLOWED.values()) == 1}

# 但这个修复**必须看原话**，否则它会往错的设备上发指令。
#
# 模型答错的时候，域和动作总有一个是幻觉，而上面那行无条件地认定"幻觉的是域"。
# 反例：对着「这首太吵了」它吐 music.brightness_step，brightness_step 只有 light 有，
# 于是被修成 light.brightness_step —— **去调台灯亮度**。可原话里"这首"明明白白
# 指着音乐，句子里的域信号是强的，幻觉的是动作。
#
# 所以判据是：**它说的那个域，原话佐证了没有**。
#   佐证了，而且**要修去的那个域没被佐证** -> 域是对的，动作是幻觉 -> 丢掉
#   其余情况                                -> 按动作修回来（原来那条路）
#
# 后半个条件是拿反例逼出来的。只判"原域被佐证就丢"会误伤：
# 「这首歌音量小一点」里 music 被"歌/这首"佐证，但 tivoli 也被"音量"佐证，
# 而 volume_step 确实只有 tivoli 有 —— 这时候丢掉等于回一句"我不会"，
# 用户明明说得清清楚楚。两边都被提到时，动作才是那个分得开的信号。
#「把收音机声音关小」里没有"音乐/歌/这首"，music 得不到佐证，照旧修成 tivoli。
_DOMAIN_WORDS = {
    "light": ("台灯", "吊灯", "壁灯", "灯"),
    "tivoli": ("音响", "收音机", "电台", "广播", "音量", "声音"),
    "music": ("音乐", "歌", "这首", "那首", "这曲", "专辑", "歌单"),
    # 不收单字"度"：角度、程度、再调亮一度都会误伤
    "aircon": ("空调", "冷气", "暖气", "温度"),
}


def _corroborated(raw: str, domain: str) -> bool:
    """原话里有没有指向这个域的设备词。"""
    return any(w in (raw or "") for w in _DOMAIN_WORDS.get(domain, ()))


# 每个动作只接收它用得到的槽位；类型/范围错误时放弃模型结果。
# JSON schema 是生成约束，不能替代执行前校验（尤其 bool 在 Python 中也是 int）。
_SLOT_LIMITS = {
    "pct": (0, 100), "kelvin": (2700, 6500), "preset": (1, 6), "temp": (17, 30),
}
# **绝对值槽位必须在原话里找得到依据**，不管它是必填的还是可选的。
#
# schema 里摆着 pct/kelvin/temp 这些字段，模型就会顺手把它们填满。
# 实测（2026-09-16，qwen3:4b 真跑）十句话里有七句带着 "pct":0,"kelvin":0,"temp":0
# 这样的填充值，而其中一条是会出事的：
#
#     「太亮了受不了」 -> light.brightness pct=0
#
# 0 落在 [0,100] 之内，范围检查一个字都挑不出来，于是灯被设成最低亮度 ——
# 一个用户从没说过的**绝对值**。这比编造动作危险：编的动作有白名单挡着，
# 编的数值长得和真数值一模一样。
#
# 判据不是"值合不合法"，是"用户到底说没说过这件事"。
# 刻意不收裸中文数词："调暗一点""开一下"里的"一"会让任何填充值都过关。
_SLOT_EVIDENCE = {
    "pct": re.compile(r"[0-9]|百分之|亮度|一半|最亮|最暗"),
    "kelvin": re.compile(r"[0-9]|色温|暖光|冷光|暖色|冷色|白光|黄光"),
    "temp": re.compile(r"[0-9]|[零一二两三四五六七八九十]+\s*度|多少度"),
    "preset": re.compile(r"[0-9]|第[零一二两三四五六七八九十]|[零一二两三四五六七八九十]\s*[个台号]"),
}
_ACTION_SLOTS = {
    ("light", "on"): ("pct", "kelvin"),
    ("light", "brightness"): ("pct",),
    ("light", "brightness_step"): ("step",),
    ("light", "color_temp"): ("kelvin",),
    ("tivoli", "preset_recall"): ("preset",),
    ("tivoli", "preset_save"): ("preset",),
    ("tivoli", "set_volume"): ("pct",),
    ("tivoli", "volume_step"): ("step",),
    ("tivoli", "station_step"): ("step",),
    ("music", "play"): ("query",),
    ("aircon", "on"): ("temp",),
    ("aircon", "set_temp"): ("temp",),
    ("aircon", "temp_step"): ("step",),
}


# 缺了可以问回去的槽位。问句要能用一句人话问出来、答案是一个**裸值**，
# 这两条同时成立才放进来。
#
# step 刻意不在里面：「调高还是调低」本身就是这条指令的全部内容，
# 模型给出 brightness_step 却给不出方向，说明它压根没听懂 —— 那是该丢的，
# 不是该问的。问"要调亮还是调暗？"只是把一次失败包装成一次对话。
_ASKABLE = {
    ("light", "brightness"): ("pct", "亮度调到多少？"),
    ("light", "color_temp"): ("kelvin", "色温要多少？"),
    ("tivoli", "set_volume"): ("pct", "音量设成多少？"),
    ("tivoli", "preset_recall"): ("preset", "第几个台？"),
    ("tivoli", "preset_save"): ("preset", "存到第几个？"),
    ("music", "play"): ("query", "想听什么？"),
    ("aircon", "set_temp"): ("temp", "空调调到多少度？"),
}


def _validated_slots(data: dict, domain: str, action: str,
                     raw: str = "") -> tuple[dict | None, str]:
    """校验模型给的槽位。返回 (槽位, 缺的那个键)。

    三种结果要分得开，因为**该做的事不一样**：
        ({...}, "")      拿到了，去执行
        (None,  "temp")  该给的没给 -> 问回去（调用方决定问不问）
        (None,  "")      给了但不合法/编的 -> 丢掉，这是幻觉信号

    原来这里只有"要么槽位要么 None"，于是缺参和乱编走同一条路，
    最后都变成一句「这个我还不会」—— 而那句话在缺参时是**假的**：
    我们明明知道他在说空调、要设温度，只是不知道设到几度。

    三道关的**顺序是有讲究的**，排错了会把"乱编"也变成"问回去"：
        1. 没给      -> 问（我们知道他要干什么）
        2. 类型/范围 -> 丢（pct=101、pct="30" 是模型在胡说，问也问不出个所以然）
        3. 原话依据  -> 问（值本身合法，但用户压根没说过这个数）
    """
    slots: dict = {}
    askable = _ASKABLE.get((domain, action))

    def _ask_for(key: str) -> tuple[None, str]:
        """能问的才回 key；不能问的（比如 step）回空串 = 丢掉。"""
        return None, (key if askable and askable[0] == key else "")

    for key in _ACTION_SLOTS.get((domain, action), ()):
        value = data.get(key)
        if value is None:
            if action == "on":       # on 的参数可选；其余动作必须给齐
                continue
            return _ask_for(key)

        # --- 第 2 关：类型和范围。不合法就是幻觉，丢掉，不问 ---
        if key == "query":
            if not isinstance(value, str) or not value.strip() or len(value) > 200:
                return None, ""
            slots[key] = value.strip()
            continue
        if type(value) is not int:   # bool 在 Python 里也是 int，得用 type 不能用 isinstance
            return None, ""
        if key == "step":
            limit = {"light": 100, "tivoli": 10, "aircon": 13}[domain]
            if value == 0 or not -limit <= value <= limit:
                return None, ""
            slots[key] = value
            continue
        low, high = _SLOT_LIMITS[key]
        if not low <= value <= high:
            return None, ""

        # --- 第 3 关：合法，但用户说过吗 ---
        ev = _SLOT_EVIDENCE.get(key)
        if ev and not ev.search(raw or ""):
            _LOG.info("LLM 给 %s.%s 填了 %s=%r，但原话里没提，不采信",
                      domain, action, key, value)
            if action == "on":
                continue             # 可选槽，丢槽位不丢动作：「开灯」本身是对的
            return _ask_for(key)

        slots[key] = value
    return slots, ""


class LocalLLM:
    def __init__(self, host: str = _HOST, model: str = _MODEL,
                 timeout: float = _TIMEOUT) -> None:
        self.host = host
        self.model = model
        self.timeout = timeout
        self._client = httpx.AsyncClient(base_url=host, timeout=timeout + 1.0,
                                         trust_env=False)
        self._ok: bool | None = None

    async def close(self) -> None:
        await self._client.aclose()

    async def available(self) -> bool:
        """模型在不在。查一次就缓存 —— 这个判断落在兜底路径上，不该每次都问一遍。"""
        if self._ok is not None:
            return self._ok
        try:
            r = await self._client.get("/api/tags", timeout=2.0)
            names = [m.get("name", "") for m in r.json().get("models", [])]
            self._ok = any(n == self.model or n.startswith(self.model.split(":")[0] + ":")
                           for n in names)
            if not self._ok:
                _LOG.warning("ollama 在，但没有 %s。跑 `ollama pull %s`", self.model, self.model)
        except Exception:  # noqa: BLE001
            self._ok = False
            _LOG.info("ollama 连不上，第三层兜底关掉 —— 规则层照常工作")
        return self._ok

    async def warm(self) -> None:
        """把权重预加载进内存。不做的话第一次兜底要等好几秒装模型，
        而那一下必然发生在用户面前。"""
        if not await self.available():
            return
        try:
            await self._client.post("/api/generate", timeout=180.0, json={
                "model": self.model, "prompt": "", "keep_alive": _KEEP_ALIVE})
            _LOG.info("%s 预热完成", self.model)
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("预热失败：%s", exc)

    async def _ask(self, prompt: str, schema: dict, timeout: float | None = None,
                   max_tokens: int = 80) -> dict | None:
        if not await self.available():
            return None
        budget = self.timeout if timeout is None else timeout
        t0 = time.perf_counter()
        try:
            r = await asyncio.wait_for(self._client.post("/api/chat", json={
                "model": self.model,
                "messages": [{"role": "user", "content": prompt}],
                "stream": False,
                "think": False,
                "format": schema,
                "keep_alive": _KEEP_ALIVE,
                "options": {"temperature": 0.1, "num_predict": max_tokens},
            }, timeout=budget + 1.0), timeout=budget)
            out = (r.json().get("message") or {}).get("content", "")
        except asyncio.TimeoutError:
            _LOG.info("LLM 超过 %.1fs 硬超时，放弃这一层", budget)
            return None
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("LLM 出错：%s", exc)
            return None
        _LOG.info("LLM %.0fms -> %s", (time.perf_counter() - t0) * 1000,
                  " ".join(out.split())[:90])
        try:
            data = json.loads(out)
        except ValueError:
            return None
        return data if isinstance(data, dict) else None

    async def classify(self, text: str, ctx=None) -> dict | None:
        """规则层拿不准的那句话。返回 {domain, action, slots, reply} 或 None。

        输出**必须**过白名单。模型很乐意发明一个 "tivoli/set_frequency"，
        而下游是真会动的硬件 —— 没在 _ALLOWED 里的一律丢掉，宁可回"我不会"。
        """
        data = await self._ask(_CLASSIFY_PROMPT % (_ground(ctx), text), _SCHEMA)
        if not data:
            return None
        domain, action = data.get("domain"), data.get("action")
        if not isinstance(domain, str) or not isinstance(action, str):
            return None
        if domain not in _ALLOWED or action not in _ALLOWED[domain]:
            home = _ACTION_HOME.get(action)
            if home is None:
                if domain != "none":
                    _LOG.info("LLM 编了个不存在的动作 %s.%s，丢掉", domain, action)
                return None
            if _corroborated(text, domain) and not _corroborated(text, home):
                # 原话只佐证了它说的那个域，那幻觉的就是动作。按动作修回来会把
                # 指令发到另一台设备上（「这首太吵了」-> 去调台灯亮度），宁可不做。
                _LOG.info("LLM 答 %s.%s，原话里只有 %s 的设备词、没有 %s 的 —— "
                          "错的是动作不是域，丢掉而不是改域", domain, action, domain, home)
                return None
            _LOG.info("LLM 把域说成了 %s，但 %s 只有 %s 有 —— 按动作修回来",
                      domain, action, home)
            domain = home
        slots, missing = _validated_slots(data, domain, action, text)
        if slots is None:
            if missing:
                _LOG.info("LLM %s.%s 缺 %s，交给上层问回去", domain, action, missing)
                return {"domain": domain, "action": action, "slots": {},
                        "missing": missing, "ask": _ASKABLE[(domain, action)][1],
                        "reply": str(data.get("reply", ""))[:24]}
            _LOG.info("LLM %s.%s 参数无效，放弃该结果", domain, action)
            return None
        return {"domain": domain, "action": action, "slots": slots,
                "reply": str(data.get("reply", ""))[:24]}

    async def keywords(self, text: str) -> str | None:
        """氛围类点歌 -> 一个搜歌单的关键词。这一路可以放宽超时：
        用户已经知道自己没点具体的歌，多等一下是合理的，而且这条路上
        没有"快速失败"的替代品 —— 拿不到关键词就只能拿原话去搜。"""
        data = await self._ask(_KEYWORDS_PROMPT % text, _KW_SCHEMA,
                               timeout=max(self.timeout, 3.0), max_tokens=32)
        if not data:
            return None
        kw = re.sub(r"[\s\"'“”‘’《》。，,.!?！？]", "", str(data.get("keyword", "")))
        return kw[:12] or None
