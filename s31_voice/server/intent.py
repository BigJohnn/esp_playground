"""中文意图解析：一句话 -> 对灯的一个动作。

刻意用规则而不是 LLM：控灯的指令空间很小，规则的延迟是微秒级、结果可预测、
断网可用。真正需要开放式理解时再在上层挂 LLM，而不是把简单事情复杂化。

这里的命令集同时是板子上 MultiNet 离线命令词表的来源（见 commands_for_multinet），
保证"板上识别"和"服务端识别"两条路的行为一致。
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Literal

Action = Literal["on", "off", "toggle", "brightness", "brightness_step", "color_temp", "none"]

# 中文数字 -> 阿拉伯数字，够用即可
_CN_DIGITS = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
              "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}


def _cn_number(text: str) -> int | None:
    """把 '八十'、'一百'、'五十五' 这类说法转成数字。"""
    if not text:
        return None
    if text.isdigit():
        return int(text)
    if text == "百" or text == "一百":
        return 100
    m = re.fullmatch(r"([一二两三四五六七八九])?十([一二三四五六七八九])?", text)
    if m:
        tens = _CN_DIGITS.get(m.group(1) or "一", 1)
        ones = _CN_DIGITS.get(m.group(2) or "零", 0)
        return tens * 10 + ones
    if len(text) == 1 and text in _CN_DIGITS:
        return _CN_DIGITS[text]
    return None


@dataclass
class Intent:
    action: Action
    brightness_pct: int | None = None
    brightness_step_pct: int | None = None
    color_temp_kelvin: int | None = None
    reply: str = ""
    raw: str = ""
    # 命中的规则名，方便排查
    rule: str = ""


# (规则名, 正则, 构造 Intent 的函数)
# 顺序有意义：先匹配更具体的（"调到八十"）再匹配更笼统的（"开灯"）
# 百分比必须带显式锚点（百分之 / % / 调到…），否则 "亮一点" 里的 "一" 会被当成 1%
_PCT_RE = re.compile(
    r"百分之\s*([0-9]+|[一二两三四五六七八九十百]+)"
    r"|([0-9]+)\s*(?:%|％)"
    r"|(?:调|设|开|降|升)到\s*(?:百分之)?\s*([0-9]+|[一二两三四五六七八九十百]+)\s*(?:%|％)?"
)

_RULES: list[tuple[str, re.Pattern[str]]] = [
    # 顺序有意义：先匹配更具体的档位词，再匹配百分比，最后才是笼统的开/关
    ("max",         re.compile(r"最亮|全亮|开到最大|亮度最大")),
    ("min",         re.compile(r"最暗|最小亮度|亮度最小|微光")),
    ("brighter",    re.compile(r"亮(?:一)?点|亮一些|调亮|再亮|更亮|太暗")),
    ("dimmer",      re.compile(r"暗(?:一)?点|暗一些|调暗|再暗|更暗|太亮|太刺眼")),
    ("set_pct",     _PCT_RE),
    ("warm",        re.compile(r"暖光|暖一点|暖色|黄光|暖白")),
    ("cool",        re.compile(r"冷光|冷一点|冷色|白光|冷白")),
    ("reading",     re.compile(r"看书|阅读|读书")),
    ("sleep",       re.compile(r"睡觉|睡眠|夜灯|月光")),
    ("off",         re.compile(r"关灯|关闭|关掉|把灯关|熄灯|灭了|不用灯|关上")),
    ("on",          re.compile(r"开灯|打开|把灯开|亮起来|开一下|来点光|太黑")),
    ("toggle",      re.compile(r"切换|反过来")),
]


def _to_pinyin(text: str) -> str:
    """转成无声调拼音，用来兜同音字。识别不出声调的差别正是我们要的模糊度。"""
    import pinyin_fix
    from pypinyin import Style, lazy_pinyin

    pinyin_fix.apply()
    return "".join(lazy_pinyin(text, style=Style.NORMAL, errors=lambda x: x))


# 拼音层规则：只覆盖那些"听错了但发音一样"的高频档位词。
# 数字/百分比不放进来 —— 拼音里数字歧义太大，宁可漏也不要误触发。
_PINYIN_RULES: list[tuple[str, re.Pattern[str]]] = [
    ("max",      re.compile(r"zuiliang|quanliang|liangdu?zuida")),
    ("min",      re.compile(r"zuian|weiguang")),
    ("brighter", re.compile(r"liangyi?dian|liangyixie|tiaoliang|zailiang|gengliang|taian")),
    ("dimmer",   re.compile(r"anyi?dian|anyixie|tiaoan|zaian|gengan|tailiang|taicijing?")),
    ("warm",     re.compile(r"nuanguang|nuanyidian|nuanse|huangguang|nuanbai")),
    ("cool",     re.compile(r"lengguang|lengyidian|lengse|baiguang|lengbai")),
    ("reading",  re.compile(r"kanshu|yuedu|dushu")),
    ("sleep",    re.compile(r"shuijiao|shuimian|yedeng|yueguang")),
    ("off",      re.compile(r"guandeng|guanbi|guandiao|badengguan|xideng|mieledeng|guanshang")),
    ("on",       re.compile(r"kaideng|dakai|badengkai|liangqilai|kaiyixia|taihei")),
]


def parse(text: str) -> Intent:
    t = re.sub(r"[\s，。！？、,.!?]", "", text or "")
    if not t:
        return Intent("none", reply="我没听清", raw=text)

    hit = _match(t, _RULES, text)
    if hit is not None:
        return hit

    # 字面没命中，退到拼音层。SenseVoice 把"台灯"听成"台等"、"暖光"听成"暖逛"
    # 这类同音错误在中文短指令里很常见，只在字面层做匹配会白白丢掉正确的意图。
    hit = _match(_to_pinyin(t), _PINYIN_RULES, text, pinyin=True)
    if hit is not None:
        return hit

    return Intent("none", reply="这个我还不会", raw=text)


def _match(t: str, rules: list[tuple[str, re.Pattern[str]]], raw: str,
           pinyin: bool = False) -> Intent | None:
    suffix = "-py" if pinyin else ""
    for name, pat in rules:
        m = pat.search(t)
        if not m:
            continue
        if name == "set_pct":
            group = next((g for g in m.groups() if g), None)
            pct = _cn_number(group)
            if pct is None or not (0 <= pct <= 100):
                continue
            return Intent("brightness", brightness_pct=pct,
                          reply=f"亮度调到百分之{pct}", raw=raw, rule=name + suffix)
        if name == "max":
            return Intent("brightness", brightness_pct=100, reply="已调到最亮", raw=raw, rule=name + suffix)
        if name == "min":
            return Intent("brightness", brightness_pct=1, reply="已调到最暗", raw=raw, rule=name + suffix)
        if name == "brighter":
            return Intent("brightness_step", brightness_step_pct=20, reply="调亮了", raw=raw, rule=name + suffix)
        if name == "dimmer":
            return Intent("brightness_step", brightness_step_pct=-20, reply="调暗了", raw=raw, rule=name + suffix)
        if name == "warm":
            return Intent("color_temp", color_temp_kelvin=2700, reply="换成暖光", raw=raw, rule=name + suffix)
        if name == "cool":
            return Intent("color_temp", color_temp_kelvin=6500, reply="换成冷光", raw=raw, rule=name + suffix)
        if name == "reading":
            return Intent("color_temp", color_temp_kelvin=4000, brightness_pct=100,
                          reply="阅读模式", raw=raw, rule=name + suffix)
        if name == "sleep":
            return Intent("color_temp", color_temp_kelvin=2700, brightness_pct=5,
                          reply="夜灯模式", raw=raw, rule=name + suffix)
        if name == "off":
            return Intent("off", reply="灯关了", raw=raw, rule=name + suffix)
        if name == "on":
            return Intent("on", reply="灯开了", raw=raw, rule=name + suffix)
        if name == "toggle":
            return Intent("toggle", reply="切换了", raw=raw, rule=name + suffix)
    return None


# 板上 MultiNet 的中文命令词表。每条都必须能被上面的规则解析出意图 ——
# 见 __main__ 里的自检，词表和规则脱节会当场报出来。
#
# 这份词表是**一条条对着板子念出来测出来的**，不是设计出来的（tools/speak.sh
# 用 TTS 合成后从物理扬声器放给板子听，tools/mn_regress.sh 跑整轮）。
# 因为 mn7_cn 有一类完全静默的失败：命令词注册成功、日志正常、
# 发音也没问题，就是永远不触发。实测被这样毙掉的有
#   最亮 / 最暗 / 亮度最大 / 亮度最小 / 最亮模式 / 最暗模式 / 开到最亮  （0/9）
#   把灯打开 / 把灯关掉                                                （0/2）
#   微光模式                                                          （0/2）
# 而同一轮里换成「全亮模式」立刻 2/2 —— 所以不是环境、不是音量、不是注音
# （TTS 念的和注册的拼音逐音节对得上，selftest 里验过）。
# 结论很朴素：**新词一律先过一遍声学回归再进词表**，别信推理。
#
# 另外两条已经站得住的规律：
#
# 1) 每条至少 3 个音节。第一版留了「暖光」「最暗」这种两音节词，唤醒 4/4 但命令词
#    只中 2/4，漏的正好都是两音节的。两个音节给不出足够的声学证据把自己和其他
#    候选拉开（中到的「冷光」置信度只有 0.17，四音节的「关灯」有 0.61）。
#
# 2) 躲开多音字。「调到最亮」被 pypinyin 注音成 "diao dao zui liang"，而人和 TTS
#    念的都是 "tiao ..."。注册不报错，就是永远不触发。真躲不开时在下面显式写注音。
#
# 元素是 (中文, 注音)；注音为 None 表示用 pypinyin 自动转。
_MULTINET_COMMANDS: list[tuple[str, str | None]] = [
    ("打开台灯", None),
    ("关闭台灯", None),
    ("亮一点", None),
    ("暗一点", None),
    ("全亮模式", None),
    # 不用「暖光模式 / 冷光模式」：只差第一个音节（nuan/leng），后面 "guang mo shi"
    # 三个音节完全相同，实测两轮里有一轮把「暖光」听成「冷光」。冷暖弄反是用户看得见的错。
    # 换成前两个音节都不同的说法之后就分得开了。
    ("黄光模式", None),      # huang …
    ("冷光模式", None),      # leng …
    ("阅读模式", None),
    ("夜灯模式", None),
]
# 没有单独的"最暗"命令：能表达它的说法（最暗/亮度最小/微光模式）在板上一条都不触发，
# 而「夜灯模式」本来就是 5% + 2700K，实际需求已经被它覆盖，「暗一点」还能连着说。
# 与其留一条按不动的命令，不如不留。


def _auto_phonemes(text: str) -> str:
    import pinyin_fix
    from pypinyin import Style, lazy_pinyin

    pinyin_fix.apply()

    return " ".join(lazy_pinyin(text, style=Style.NORMAL, errors=lambda x: x))


def commands_for_multinet() -> list[dict]:
    """给板子的命令词清单：[{id, text, phonemes}, ...]。

    mn7_cn 的命令词以**无声调拼音、按音节空格分隔**为单位（见 esp-sr/tool/README.md），
    例如 "打开台灯" -> "da kai tai deng"。这里直接复用意图解析已有的拼音层，
    保证"板上离线识别"和"服务端 ASR"两条路用的是同一份词表、同一套拼音规则。
    """
    return [
        {"id": i, "text": text, "phonemes": override or _auto_phonemes(text)}
        for i, (text, override) in enumerate(_MULTINET_COMMANDS)
    ]


if __name__ == "__main__":
    for s in ["打开台灯", "把灯关了", "亮一点", "调到百分之八十", "调到 30%",
              "有点太刺眼了", "换成暖光", "我要看书", "今天天气怎么样"]:
        i = parse(s)
        print(f"{s:<16} -> {i.action:<16} rule={i.rule:<12} "
              f"pct={i.brightness_pct} step={i.brightness_step_pct} k={i.color_temp_kelvin}")

    print("\n--- MultiNet 词表自检（每条都必须能解析出非 none 的意图）---")
    bad = 0
    for c in commands_for_multinet():
        i = parse(c["text"])
        flag = "" if i.action != "none" else "   <<< 解析不出来"
        bad += i.action == "none"
        print(f"{c['id']:>2}  {c['text']:<10} {c['phonemes']:<22} -> {i.action}{flag}")
    print("全部可解析" if not bad else f"{bad} 条词表和规则脱节了")

    print("\n注音对不对，靠 selftest.py 的闭环校验（TTS 念一遍再听回来比对），"
          "不靠这里的静态检查 —— 静态查多音字只会把「灯(deng/ding)」这种"
          "谁都不会念错的也报出来，噪声盖过信号。")
