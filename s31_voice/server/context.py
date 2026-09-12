"""对话状态：一句省略了主语的话，到底在说哪台设备。

为什么需要这个模块 —— 它替掉的是一个字符串
--------------------------------------------
在它之前，消歧全靠 `Router.last_domain`，也就是**上一条成功命令操作了谁**。
实测下来「大一点」因此有四个意思：

    last_domain=None    大一点 -> 灯亮度      关掉 -> 关灯
    last_domain=music   大一点 -> 音响音量    关掉 -> 停止播放
    last_domain=aircon  大一点 -> 空调温度    关掉 -> 关空调
    last_domain=light   大一点 -> 灯亮度      关掉 -> 关灯

问题不在这张表错了，在于**它查的东西不对**。省略和指代要靠"当前情境"补全，
而情境 ≠ 上一条命令：

  - 音乐正放着，但你上一句调的是空调 -> 说「大一点」，房间变热。
    用户一个字都没说错。
  - 刚开机 last_domain=None，所有省略句一律归给灯 —— 哪怕灯根本没开。
  - 用手机/实体按钮开的音乐，系统完全不知道，因为那不是"一条成功命令"。

根子上：`last_domain` 记的是**我们说过什么**，而人省略主语时，
心里想的是**现在正在发生什么**。这两件事经常不是一回事。

所以这里换成先查世界、再查话题：

    候选域（「大一点」= 灯亮度 / 音量 / 空调温度）
      -> 其中**此刻活着**的有哪些
      -> 只有一个       -> 就是它
      -> 有好几个       -> 上次说的那个优先（话题连续性是真实的信号，只是次要）
      -> 一个都没活着   -> 才退回默认

注意一个不对称：「活着」是把候选**筛掉**，不是**加权**。关着的空调不可能是
「大一点」的对象，这是硬约束；而"上次聊的是空调"只是个偏好。
把硬约束和偏好混在一个打分函数里，是这类消歧最常见的做法，也最容易出
"分数刚好压过去了"的怪结果。

诚实性
------
`WorldState` 每个字段都是**我们以为的**，不是测回来的（Tivoli 那边压根没有
回执，见 README §4.1.17）。所以字段一律可以是 None = 不知道，而"不知道"
**不等于**"关着"：不知道的设备留在候选里，交给后面的反问，而不是在这儿
被悄悄筛掉。这是这个模块最容易写错的一行。
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Sequence

Domain = str

# 一个域"最近动过"算活着的时长。超过就不再当作正在进行的事情。
# 取 10 分钟：一首歌大约 4 分钟，两三首之内说「大一点」显然还在说音乐；
# 而半小时前放过歌这件事，不该再影响现在这句话。
_ACTIVE_TTL = 600.0

# 一个域被说过之后，还算不算"当前话题"。比 _ACTIVE_TTL 短得多 ——
# 话题的时效本来就短，隔了五分钟再说一句「关掉」，指的多半是眼前正响着的东西，
# 而不是五分钟前提过的那台。
_TOPIC_TTL = 90.0


@dataclass
class WorldState:
    """此刻我们**以为**每台设备在干什么。None = 不知道，不是"否"。"""
    light_on: bool | None = None
    music_playing: bool = False          # 这条是真的（队列在我们自己手里）
    music_paused: bool = False
    tivoli_powered: bool | None = None
    tivoli_source: str | None = None
    aircon_on: bool | None = None
    # 各域最后一次"确实在动"的时刻。用来给 _ACTIVE_TTL 计时。
    touched: dict[Domain, float] = field(default_factory=dict)

    def touch(self, domain: Domain) -> None:
        self.touched[domain] = time.time()

    def fresh(self, domain: Domain) -> bool:
        t = self.touched.get(domain)
        return t is not None and (time.time() - t) < _ACTIVE_TTL

    def active(self, domain: Domain) -> bool | None:
        """这台设备此刻是不是正在做事。None = 不知道。

        三态而不是两态是有意的：不知道的设备**不该被筛掉**。
        Tivoli 尤其如此 —— 它的状态全是影子，而影子一开始就是 None。
        """
        if domain == "music":
            return self.music_playing or self.music_paused
        if domain == "light":
            return self.light_on
        if domain == "aircon":
            return self.aircon_on
        if domain == "tivoli":
            if self.tivoli_powered is False:
                return False
            # 开着、或者不知道开没开。在 WiFi 源上放着歌的时候
            # 音量归 music 管更自然，但两者都指向同一个物理旋钮，无所谓。
            if self.tivoli_powered or self.fresh("tivoli"):
                return True
            return None
        return None


@dataclass
class Turn:
    """一轮对话。留着是为了给 LLM 当上下文，以及排查时看得见来龙去脉。"""
    text: str
    domain: str
    action: str
    ok: bool
    reply: str
    at: float = field(default_factory=time.time)


@dataclass
class Question:
    """系统问出去、还没得到回答的一个问题。

    它是**会改写下一句话含义**的东西，所以必须有有效期：
    用户半分钟后随口说的「第二个」不该命中一个早就过时的问题。
    """
    kind: str                  # "song" / "domain" / ...
    options: list              # 选项，语义由 kind 决定
    text: str                  # 问出去的原话
    deadline: float

    @property
    def expired(self) -> bool:
        return time.time() > self.deadline


class Context:
    def __init__(self, history: int = 6) -> None:
        self.world = WorldState()
        self.turns: deque[Turn] = deque(maxlen=history)
        self.question: Question | None = None
        # 上一次成功操作的设备 + 时刻。就是原来的 last_domain，
        # 但降级成了**次要**信号：只在世界状态分不出来的时候才用得上。
        self.focus: Domain | None = None
        self.focus_at: float = 0.0

    # ------------------------------------------------------------ 记账

    def record(self, text: str, domain: str, action: str, ok: bool, reply: str) -> None:
        self.turns.append(Turn(text, domain, action, ok, reply))
        if ok and domain not in ("none", ""):
            self.focus = domain
            self.focus_at = time.time()
            self.world.touch(domain)

    @property
    def topic(self) -> Domain | None:
        """还算数的"当前话题"。过了 _TOPIC_TTL 就不算了。"""
        if self.focus is None or (time.time() - self.focus_at) > _TOPIC_TTL:
            return None
        return self.focus

    # ------------------------------------------------------------ 消歧

    def rank(self, candidates: Sequence[Domain]) -> list[Domain]:
        """把候选域按"此刻多像是在说它"排序。返回的顺序就是取用顺序。

        分三档，档与档之间不可换算（不是打分求和）：

          1. 确定活着的
          2. 不知道活不活（None）—— 影子状态天然如此，不能当成"关着"
          3. 确定关着的

        档内再按话题连续性排：上次说的那个排前面。
        """
        topic = self.topic

        def tier(d: Domain) -> int:
            a = self.world.active(d)
            return 0 if a else (1 if a is None else 2)

        return sorted(candidates, key=lambda d: (tier(d), 0 if d == topic else 1,
                                                 -self.world.touched.get(d, 0.0)))

    def ambiguous(self, candidates: Sequence[Domain]) -> list[Domain]:
        """真正分不清的那几个。空列表 = 分得清，直接用 rank() 的第一个。

        判据：**有两个以上的域确定活着，而且话题帮不上忙**。
        只有这时候反问才是对的 —— 灯开着、音乐也放着，用户说「关掉」，
        我们确实不知道他要关哪个，猜一个的代价（关错了）比问一句高。

        反过来，只有一个活着、或者话题指向其中之一，就别问。
        问多了比猜错还烦，这是这套交互能不能用的分界线。
        """
        live = [d for d in candidates if self.world.active(d)]
        if len(live) < 2:
            return []
        if self.topic in live:
            return []
        return live

    def confidence_for(self, via: Domain) -> float:
        """按 via 这个候选域选出来的结论，有多可信。

        这个函数必须住在 Context 里，不能住在 intent.py 里 —— 只有这边
        知道谁活着。曾经试过在 intent.py 里按"它在候选顺序里排第几"反推，
        那是错的：排第一也可能只是"一堆不知道里的第一个"，
        和"唯一一台确实在响的"分数不该一样。
        """
        a = self.world.active(via)
        if a:
            return 0.9          # 它确实活着
        if via == self.topic:
            return 0.7          # 不知道活没活，但刚聊过它
        if a is None:
            return 0.5          # 不知道，也没聊过
        return 0.4              # 确定关着，纯属没得选的兜底

    def as_dict(self) -> dict:
        return {
            "focus": self.focus, "topic": self.topic,
            "world": {"light_on": self.world.light_on,
                      "music_playing": self.world.music_playing,
                      "music_paused": self.world.music_paused,
                      "tivoli_powered": self.world.tivoli_powered,
                      "tivoli_source": self.world.tivoli_source,
                      "aircon_on": self.world.aircon_on},
            "question": None if self.question is None or self.question.expired
                        else {"kind": self.question.kind, "text": self.question.text},
            "turns": [{"text": t.text, "domain": t.domain, "action": t.action,
                       "ok": t.ok, "reply": t.reply} for t in self.turns],
        }
