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

_HOST = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434").rstrip("/")
_MODEL = os.environ.get("LLM_MODEL", "qwen3:4b")
# roadmap 里定的硬超时。超了就当没有这一层 —— 它是兜底，不是依赖。
_TIMEOUT = float(os.environ.get("LLM_TIMEOUT", "1.5"))
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
        "domain": {"type": "string", "enum": ["light", "tivoli", "music", "none"]},
        "action": {"type": "string"},
        "preset": {"type": "integer"},     # 预设位 1-6
        "step": {"type": "integer"},       # 相对增减
        "pct": {"type": "integer"},        # 亮度百分比
        "kelvin": {"type": "integer"},     # 色温
        "query": {"type": "string"},       # 要搜的歌/歌手
        "reply": {"type": "string"},
    },
    "required": ["domain", "action"],
}

_KW_SCHEMA = {"type": "object", "properties": {"keyword": {"type": "string"}},
              "required": ["keyword"]}

_CLASSIFY_PROMPT = """把用户这句话归到下面某一条动作上。

action 只能从下面这些词里原样挑一个，不许改写、不许加括号：
light   on  off  brightness  brightness_step  color_temp
tivoli  power_on  power_off  radio_on  preset_recall  preset_save  station_step  volume_step  mute
music   play  play_favorites  next  prev  pause  resume  stop  now_playing
none    none

需要的话另外填这几个字段：pct(亮度 0-100) kelvin(色温 2700-6500) step(增减，如 20/-20/1/-1/3/-3)
preset(预设位 1-6) query(要搜的歌名或歌手)。
reply 是给用户听的中文回话，不超过 12 个字。

用户说：%s"""

_KEYWORDS_PROMPT = """用户想听歌，但没说具体歌名。把这句话变成一个用来搜网易云歌单的关键词，
2 到 8 个汉字。

用户说：%s"""

_ALLOWED = {
    "light": {"on", "off", "toggle", "brightness", "brightness_step", "color_temp"},
    "tivoli": {"power_on", "power_off", "radio_on", "preset_recall", "preset_save",
               "station_step", "volume_step", "mute"},
    "music": {"play", "play_favorites", "next", "prev", "pause", "resume", "stop",
              "now_playing"},
}
# action -> 唯一属于哪个域。模型经常把域搞错但动作说对（"把收音机声音关小"
# 它答 music.volume_step）——  volume_step 只有 tivoli 有，那就是 tivoli。
# 与其因为域错了整条丢掉，不如按动作把它修回来。
_ACTION_HOME = {a: d for d, acts in _ALLOWED.items() for a in acts
                if sum(a in x for x in _ALLOWED.values()) == 1}
# 槽位名 -> Intent 上的字段。扁平的 schema 比嵌套的 slots 好填得多。
_SLOT_KEYS = ("preset", "step", "query", "pct", "kelvin")


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

    async def classify(self, text: str) -> dict | None:
        """规则层没认出来的那句话。返回 {domain, action, slots, reply} 或 None。

        输出**必须**过白名单。模型很乐意发明一个 "tivoli/set_frequency"，
        而下游是真会动的硬件 —— 没在 _ALLOWED 里的一律丢掉，宁可回"我不会"。
        """
        data = await self._ask(_CLASSIFY_PROMPT % text, _SCHEMA)
        if not data:
            return None
        domain, action = data.get("domain"), str(data.get("action") or "")
        if domain not in _ALLOWED or action not in _ALLOWED[domain]:
            home = _ACTION_HOME.get(action)
            if home is None:
                if domain != "none":
                    _LOG.info("LLM 编了个不存在的动作 %s.%s，丢掉", domain, action)
                return None
            _LOG.info("LLM 把域说成了 %s，但 %s 只有 %s 有 —— 按动作修回来",
                      domain, action, home)
            domain = home
        slots = {k: data[k] for k in _SLOT_KEYS if data.get(k) not in (None, "", 0)}
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
