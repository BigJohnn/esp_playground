"""中文意图解析：一句话 -> 对某台设备的一个动作。

刻意用规则而不是 LLM：家里的指令空间很小，规则的延迟是微秒级、结果可预测、
断网可用。真正开放式的那部分（"放点适合睡觉的歌"）才交给本地 LLM，
而且是在规则**没**命中之后，绝不挡在规则前面。见 llm.py。

三个域，对应三条执行链：
    light   台灯          -> executor.LightExecutor（miIO 直连）
    tivoli  Tivoli 的红外  -> tivoli.TivoliExecutor（HA -> ESPHome -> 红外 LED）
    music   Tivoli 的网络  -> player.MusicExecutor（网易云 -> AirPlay 推流）

tivoli 和 music 是同一台机器的两条**互斥**通路（红外那条在 FM 源上，
推流那条在 WiFi 源上），分开是因为它们的失败模式完全不同 ——
红外是开环发完就不知道了，推流有确切的成功/失败回执。

这里的命令集同时是板子上 MultiNet 离线命令词表的来源（见 commands_for_multinet），
保证"板上识别"和"服务端识别"两条路的行为一致。
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from typing import Callable, Literal

Domain = Literal["light", "tivoli", "music", "none"]

# 中文数字 -> 阿拉伯数字，够用即可
_CN_DIGITS = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5,
              "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}
# 预设序号的无声调拼音。只到六 —— 机器就六个预设位。
_PY_DIGITS = {"yi": 1, "er": 2, "liang": 2, "san": 3, "si": 4, "wu": 5, "liu": 6}


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
    domain: Domain = "none"
    action: str = "none"
    # 灯的槽位。留在顶层而不是塞进 slots，是因为 LightExecutor 已经在用了，
    # 而这次改动的目的不是重写能跑的东西。
    brightness_pct: int | None = None
    brightness_step_pct: int | None = None
    color_temp_kelvin: int | None = None
    # 其余域的槽位：preset / step / query / artist / title / vibe ...
    slots: dict = field(default_factory=dict)
    reply: str = ""
    raw: str = ""
    # 命中的规则名，方便排查
    rule: str = ""


# ---------------------------------------------------------------- FM 预设台名
#
# "切到预设三"能说，但没人这么说话；"放音乐之声"才是人话。
# 台名 -> 预设位的映射是**配置**不是代码：换个城市、换个台，改 json 不改 py。
# 文件不存在就只剩序号那条路，功能不缺，只是说法笨一点。
_PRESETS_PATH = os.environ.get(
    "FM_PRESETS_PATH", os.path.join(os.path.dirname(__file__), "fm_presets.json"))
_STATION_NAMES: dict[str, int] = {}
_STATION_RE: re.Pattern[str] | None = None
_STATION_PY: list[tuple[re.Pattern[str], int]] = []


def load_presets(path: str | None = None) -> dict[str, int]:
    """(重新)读台名表。返回 {台名: 预设位}。

    格式：{"1": {"name": "音乐之声", "alias": ["音乐台"], "freq": "90.0"}, ...}
    freq 只是给人看的备注 —— 我们**不能**按频率调台（读不回显示屏），见 roadmap 的"明确不做"。
    """
    global _STATION_NAMES, _STATION_RE, _STATION_PY
    p = path or _PRESETS_PATH
    names: dict[str, int] = {}
    try:
        with open(p, encoding="utf-8") as f:
            raw = json.load(f)
    except FileNotFoundError:
        raw = {}
    except Exception:  # noqa: BLE001 - 配置坏了不该让整个语音链起不来
        raw = {}
    for slot, meta in raw.items():
        try:
            n = int(slot)
        except (TypeError, ValueError):
            continue
        if not 1 <= n <= 6:
            continue
        if isinstance(meta, str):
            meta = {"name": meta}
        for name in [meta.get("name"), *(meta.get("alias") or [])]:
            if name:
                names[str(name)] = n

    _STATION_NAMES = names
    # 长名字优先，免得"音乐台"被"音乐"抢先匹配掉
    ordered = sorted(names, key=len, reverse=True)
    _STATION_RE = re.compile("|".join(re.escape(n) for n in ordered)) if ordered else None
    _STATION_PY = []
    if ordered:
        for name in ordered:
            _STATION_PY.append((re.compile(re.escape(_to_pinyin(name))), names[name]))
    return names


def station_name(slot: int) -> str:
    """预设位 -> 台名，没配就回"预设N"。回话里用。"""
    for name, n in _STATION_NAMES.items():
        if n == slot:
            return name
    return f"预设{slot}"


# ---------------------------------------------------------------- 规则表
#
# 顺序有意义，而且这次的顺序有一条硬约束：**带设备名的规则必须排在泛化词前面**。
# "打开收音机"和"打开台灯"共享"打开"，如果泛化的开/关排在前面，
# 每一句"打开收音机"都会去开灯。所以整张表是这么分层的：
#
#   1. tivoli 专有（收音机/电台/预设/音量）
#   2. music  专有（收藏/下一首/暂停/播放 X）
#   3. light  专有（灯/亮度/色温 —— 这些词只有灯有）
#   4. 泛化词（打开/关掉/下一个）—— 靠"上一次操作的是哪台设备"消歧
#
# 百分比必须带显式锚点（百分之 / % / 调到…），否则 "亮一点" 里的 "一" 会被当成 1%
_PCT_RE = re.compile(
    r"百分之\s*([0-9]+|[一二两三四五六七八九十百]+)"
    r"|([0-9]+)\s*(?:%|％)"
    # (?!半) 是踩出来的：「调到一半」会被抓成「一」，然后设成 1% ——
    # 一个安静的、看起来完全合理的错误。半档另有规则接。
    r"|(?:调|设|开|降|升)到\s*(?:百分之)?\s*([0-9]+|[一二两三四五六七八九十百]+)(?!半)\s*(?:%|％)?"
)

# 预设序号：中文或阿拉伯，1-6
_N = r"([一二两三四五六1-6])"

# "播放…"的动词前缀。单独拎出来是因为 music 的 catch-all 规则要用它，
# 而它极其贪婪 —— 必须排在所有 music 专有规则**之后**。
# (?!光) 因为"来点光"是灯 —— music 的规则排在 light 前面，不躲开就被它抢走了
_PLAY_VERB = (r"(?:播放|放一首|来一首|来首|放首|我想听|想听听|我要听|听一下|"
              r"来点(?!光)|来些|放点|放|听)")

_RULES: list[tuple[str, Domain, re.Pattern[str]]] = [
    # ---------------- 1. Tivoli 专有（红外 / FM）----------------
    # "存"必须排在"切"前面：'把这个台存到预设三'里两条都能匹配上"预设三"
    ("fm_preset_save", "tivoli", re.compile(
        r"(?:存|保存|记(?:住|下)|设为|设成)(?:到|成|为|进)?\s*(?:第)?\s*" + _N + r"\s*(?:号)?(?:预设|台|频道)"
        r"|(?:存|保存|记(?:住|下))(?:到|成|为|进)?\s*预设\s*(?:第)?\s*" + _N)),
    ("fm_preset_recall", "tivoli", re.compile(
        r"预设\s*(?:第)?\s*" + _N
        + r"|(?:切|换|跳|转|听)(?:换|听)?(?:到)?\s*第\s*" + _N + r"\s*(?:个|号)?\s*(?:台|频道|电台)"
        + r"|(?:切|换|跳|转)(?:换)?到\s*(?:第)?\s*" + _N + r"\s*(?:号)?(?:台|频道)")),
    ("fm_station_next", "tivoli", re.compile(
        r"下一?个?(?:电)?台|下个台|换一?个?台|切一?个?台|下一个频道|搜台")),
    ("fm_station_prev", "tivoli", re.compile(r"上一?个?(?:电)?台|上个台|上一个频道")),
    # 音量和"关"都必须排在 radio_on 前面。radio_on 的动词前缀是可选的
    # （"收音机"三个字本身就算数），所以「帮我把收音机声音关小一些」和
    # 「关掉收音机」里都含着它 —— 顺序反了，这两句都会变成"开收音机"。
    ("volume_up", "tivoli", re.compile(r"(?:声音|音量)(?:调)?大|大点声|大声(?:一)?点|太小声|听不(?:太)?清")),
    ("volume_down", "tivoli", re.compile(r"(?:声音|音量)(?:调|关|放)?小|小点声|小声(?:一)?点|太吵|(?:声音|音量)太大")),
    ("tivoli_off", "tivoli", re.compile(
        r"关(?:掉|闭|上|了)?(?:一下)?(?:音响|音箱|收音机|广播|电台|调频)"
        r"|(?:音响|音箱|收音机|广播|电台)(?:关(?:掉|了|上))")),
    ("tivoli_on",  "tivoli", re.compile(r"(?:打开|开启|开)(?:一下)?(?:音响|音箱)")),
    ("radio_on",  "tivoli", re.compile(r"(?:打开|开启|开|听|来点|放)?(?:收音机|广播|电台|调频)")),
    ("mute",      "tivoli", re.compile(r"静音|别出声|闭嘴|消音")),

    # ---------------- 2. 音乐专有（网络）----------------
    ("music_favorites", "music", re.compile(
        r"我(?:的)?收藏|我喜欢的(?:音乐|歌)?|我收藏的(?:歌|音乐)?|收藏(?:的)?(?:歌|列表)|红心歌曲")),
    ("music_next", "music", re.compile(r"下一首|下首|换一首|换首|切歌|跳过这首")),
    ("music_prev", "music", re.compile(r"上一首|上首|前一首|回上一首|重放这首")),
    ("music_pause", "music", re.compile(r"暂停|停一下|先停")),
    ("music_resume", "music", re.compile(r"继续(?:播放|放)?|接着放|接着听")),
    ("music_stop", "music", re.compile(r"停止播放|别放了|不听了|关掉音乐|关了音乐")),
    ("music_now", "music", re.compile(r"(?:现在|正在|这)(?:放|听|唱)的?是什么|这是什么歌|什么歌|"
                                      r"这首歌?叫什么|歌名(?:是什么)?")),
    ("music_play", "music", re.compile(_PLAY_VERB + r"(.{1,40})$")),

    # ---------------- 3. 灯专有 ----------------
    ("max",         "light", re.compile(r"最亮|全亮|开到最大|亮度最大")),
    ("min",         "light", re.compile(r"最暗|最小亮度|亮度最小|微光")),
    ("brighter",    "light", re.compile(r"亮(?:一)?点|亮一些|调亮|再亮|更亮|太暗")),
    ("dimmer",      "light", re.compile(r"暗(?:一)?点|暗一些|调暗|再暗|更暗|太亮|太刺眼")),
    ("half",        "light", re.compile(r"一半|百分之五十|(?:调|设|降)到一半")),
    ("set_pct",     "light", _PCT_RE),
    ("warm",        "light", re.compile(r"暖光|暖一点|暖色|黄光|暖白")),
    ("cool",        "light", re.compile(r"冷光|冷一点|冷色|白光|冷白")),
    ("reading",     "light", re.compile(r"看书|阅读|读书")),
    ("sleep",       "light", re.compile(r"夜灯|月光|睡眠模式")),
    # 「灯」这个字是灯的护照：带上它就不会和音响抢词，所以开关词可以放得宽一些
    ("off",         "light", re.compile(r"关(?:掉|闭|上|了)?(?:一下)?[台吊壁]?灯|把灯关|熄灯|灯灭了|不用灯")),
    ("on",          "light", re.compile(r"(?:打)?开(?:一下)?[台吊壁]?灯|把灯开|灯亮起来|来点光|太黑")),

    # ---------------- 4. 泛化词（靠上一次操作的设备消歧）----------------
    ("amb_off",    "none", re.compile(r"^(?:关(?:掉|闭|上|了)?|停|停下)$")),
    ("amb_on",     "none", re.compile(r"^(?:打开|开(?:一下)?|开始)$")),
    ("amb_next",   "none", re.compile(r"^(?:下一个|下个|换一个|换个)$")),
    ("amb_prev",   "none", re.compile(r"^(?:上一个|上个)$")),
    ("toggle",     "light", re.compile(r"切换|反过来")),
]


# ---------------------------------------------------------------- 构造器
#
# 一条规则一个构造器，而不是一条长 if-链：三个域二十多条规则的话，
# if-链会长到没人愿意在里面找东西。返回 None = 这条其实没匹配上，继续往下试。

def _preset_num(m: re.Match[str]) -> int | None:
    """从匹配里取预设位。字面层拿到的是「三」，拼音层拿到的是「san」，都得认。"""
    g = next((g for g in m.groups() if g), None)
    if not g:
        return None
    n = int(g) if g.isdigit() else (_CN_DIGITS.get(g) or _PY_DIGITS.get(g))
    return n if n and 1 <= n <= 6 else None


def _b_preset_save(m, raw, rule):
    n = _preset_num(m)
    if n is None:
        return None
    return Intent("tivoli", "preset_save", slots={"preset": n},
                  reply=f"存到预设{n}了", raw=raw, rule=rule)


def _b_preset_recall(m, raw, rule):
    n = _preset_num(m)
    if n is None:
        return None
    return Intent("tivoli", "preset_recall", slots={"preset": n},
                  reply=f"切到{station_name(n)}", raw=raw, rule=rule)


def _b_pct(m, raw, rule):
    group = next((g for g in m.groups() if g), None)
    pct = _cn_number(group)
    if pct is None or not (0 <= pct <= 100):
        return None
    return Intent("light", "brightness", brightness_pct=pct,
                  reply=f"亮度调到百分之{pct}", raw=raw, rule=rule)


# "播放王菲的红豆" -> artist=王菲 title=红豆
# "播放《红豆》"   -> title=红豆
# "放点适合睡觉的歌" -> vibe=True，交给 LLM 抽关键词
_BOOK_RE = re.compile(r"[《〈\"'\"']([^》〉\"'\"']{1,30})[》〉\"'\"']")
_ARTIST_RE = re.compile(r"^(.{1,10}?)的(.{1,30})$")
# 这些词跟在"的"后面时不是歌名，是"某人的作品"这个意思
_GENERIC_TAIL = {"歌", "音乐", "歌曲", "作品", "专辑", "新歌", "老歌"}
# 出现这些词说明用户在描述**氛围**而不是点具体的歌 —— 规则抽不出关键词，该 LLM 上了
_VIBE_RE = re.compile(r"适合|点儿?[^的]{0,4}的?歌|随便|一些|轻松|安静|舒缓|放松|睡觉|助眠|"
                      r"开心|伤感|难过|运动|跑步|工作|学习|下雨|发呆|怀旧|摇滚|爵士|古典|"
                      r"类似|风格|之类|那种|氛围")


def _b_music_play(m, raw, rule):
    q = re.sub(r"[吧呀啊呢嘛了]+$", "", m.group(1).strip())
    q = re.sub(r"(?:这|那)首歌$", "", q).strip()
    q = re.sub(r"^[点些]儿?", "", q).strip()
    if not q:
        return None
    slots: dict = {"query": q}

    if q in _GENERIC_TAIL:
        # 「来点音乐吧」——  没说听什么。当氛围类处理，让上层去搜个歌单，
        # 而不是拿"音乐"两个字当歌名去搜（那会搜出一堆真的叫《音乐》的歌）。
        slots["vibe"] = True
        return Intent("music", "play", slots=slots, raw=raw, rule=rule)

    bm = _BOOK_RE.search(q)
    if bm:
        # 书名号是用户自己给的边界，比任何启发式都准，优先级最高
        slots["title"] = bm.group(1)
        artist = q[:bm.start()].rstrip("的 ")
        if artist:
            slots["artist"] = artist
        slots["query"] = f"{slots.get('artist', '')} {slots['title']}".strip()
    elif _VIBE_RE.search(q):
        # 在抽歌手**之前**判氛围：'点适合睡觉的歌' 会被 (.+?)的(.+) 拆成
        # 歌手='点适合睡觉'、歌名='歌'，抽得有模有样，但全是错的。
        slots["vibe"] = True
    else:
        am = _ARTIST_RE.match(q)
        if am and am.group(2) in _GENERIC_TAIL:
            # 「周杰伦的歌」—— 「歌」不是歌名，是"某人的作品"这个意思
            slots["artist"] = slots["query"] = am.group(1)
        elif am:
            slots["artist"], slots["title"] = am.group(1), am.group(2)
        elif q.endswith("的"):
            slots["artist"] = slots["query"] = q[:-1]

    return Intent("music", "play", slots=slots, raw=raw, rule=rule,
                  reply="")  # 回话等查到歌名再说，这里说不出有意义的话


def _simple(domain: Domain, action: str, reply: str, **slots):
    def build(m, raw, rule):
        return Intent(domain, action, slots=dict(slots), reply=reply, raw=raw, rule=rule)
    return build


def _light(action: str, reply: str, **kw):
    def build(m, raw, rule):
        return Intent("light", action, reply=reply, raw=raw, rule=rule, **kw)
    return build


_BUILD: dict[str, Callable[[re.Match[str], str, str], Intent | None]] = {
    # tivoli
    "fm_preset_save":   _b_preset_save,
    "fm_preset_recall": _b_preset_recall,
    "fm_station_next":  _simple("tivoli", "station_step", "换个台", step=1),
    "fm_station_prev":  _simple("tivoli", "station_step", "退回上一个台", step=-1),
    "tivoli_on":        _simple("tivoli", "power_on", "音响开了"),
    "radio_on":         _simple("tivoli", "radio_on", "收音机开了"),
    "tivoli_off":       _simple("tivoli", "power_off", "音响关了"),
    "volume_up":        _simple("tivoli", "volume_step", "调大了", step=3),
    "volume_down":      _simple("tivoli", "volume_step", "调小了", step=-3),
    "mute":             _simple("tivoli", "mute", "静音了"),
    # music
    "music_favorites":  _simple("music", "play_favorites", "放你收藏的歌"),
    "music_next":       _simple("music", "next", "下一首"),
    "music_prev":       _simple("music", "prev", "上一首"),
    "music_pause":      _simple("music", "pause", "暂停了"),
    "music_resume":     _simple("music", "resume", "继续"),
    "music_stop":       _simple("music", "stop", "停了"),
    "music_now":        _simple("music", "now_playing", ""),
    "music_play":       _b_music_play,
    # light
    "max":      _light("brightness", "已调到最亮", brightness_pct=100),
    "min":      _light("brightness", "已调到最暗", brightness_pct=1),
    "brighter": _light("brightness_step", "调亮了", brightness_step_pct=20),
    "dimmer":   _light("brightness_step", "调暗了", brightness_step_pct=-20),
    "half":     _light("brightness", "亮度调到一半", brightness_pct=50),
    "set_pct":  _b_pct,
    "warm":     _light("color_temp", "换成暖光", color_temp_kelvin=2700),
    "cool":     _light("color_temp", "换成冷光", color_temp_kelvin=6500),
    "reading":  _light("color_temp", "阅读模式", color_temp_kelvin=4000, brightness_pct=100),
    "sleep":    _light("color_temp", "夜灯模式", color_temp_kelvin=2700, brightness_pct=5),
    "off":      _light("off", "灯关了"),
    "on":       _light("on", "灯开了"),
    "toggle":   _light("toggle", "切换了"),
}

# 泛化词按"上一次操作的设备"落地。没有上一次就默认灯 ——
# 灯是唯一一个"猜错了代价很小"的设备：开错了再关掉就行，
# 而把音响的源切错要重新走一遍锚定。
_AMBIGUOUS: dict[str, dict[Domain, Intent]] = {
    "amb_off": {
        "light":  Intent("light", "off", reply="灯关了"),
        "tivoli": Intent("tivoli", "power_off", reply="音响关了"),
        "music":  Intent("music", "stop", reply="停了"),
    },
    "amb_on": {
        "light":  Intent("light", "on", reply="灯开了"),
        "tivoli": Intent("tivoli", "radio_on", reply="收音机开了"),
        "music":  Intent("music", "resume", reply="继续"),
    },
    "amb_next": {
        "tivoli": Intent("tivoli", "station_step", slots={"step": 1}, reply="换个台"),
        "music":  Intent("music", "next", reply="下一首"),
    },
    "amb_prev": {
        "tivoli": Intent("tivoli", "station_step", slots={"step": -1}, reply="退回上一个台"),
        "music":  Intent("music", "prev", reply="上一首"),
    },
}


def _to_pinyin(text: str) -> str:
    """转成无声调拼音，用来兜同音字。识别不出声调的差别正是我们要的模糊度。"""
    import pinyin_fix
    from pypinyin import Style, lazy_pinyin

    pinyin_fix.apply()
    return "".join(lazy_pinyin(text, style=Style.NORMAL, errors=lambda x: x))


# 拼音层：只覆盖**固定说法**。
# 歌名歌手不放进来 —— 那部分本来就该交给网易云自己的模糊搜索去兜，
# 在这里拿拼音去猜"红豆"还是"洪都"只会把一个能查的词毁掉。
_PINYIN_RULES: list[tuple[str, Domain, re.Pattern[str]]] = [
    ("fm_preset_save",   "tivoli", re.compile(r"(?:cun|baocun|jizhu)(?:dao|cheng|wei)?yushe?(yi|er|san|si|wu|liu)"
                                              r"|(?:cun|baocun)(?:dao)?di?(yi|er|san|si|wu|liu)(?:hao)?(?:yushe|tai)")),
    ("fm_preset_recall", "tivoli", re.compile(r"yushe(?:di)?(yi|er|liang|san|si|wu|liu)"
                                              r"|(?:qie|huan|tiao)(?:huan)?dao(?:di)?(yi|er|liang|san|si|wu|liu)(?:hao)?tai")),
    ("fm_station_next",  "tivoli", re.compile(r"xiayi?ge?(?:dian)?tai|huanyi?ge?tai|qieyi?ge?tai|soutai")),
    ("fm_station_prev",  "tivoli", re.compile(r"shangyi?ge?(?:dian)?tai")),
    # 和字面层同样的坑：关必须排在开前面，否则 guandiaoshouyinji 里的 shouyinji 会先命中
    ("tivoli_off",       "tivoli", re.compile(r"guan(?:diao|bi|shang|le)?(?:yinxiang|shouyinji|guangbo|diantai)")),
    ("tivoli_on",        "tivoli", re.compile(r"(?:dakai|kaiqi)yinxiang")),
    ("radio_on",         "tivoli", re.compile(r"shouyinji|guangbo|diantai|tiaopin")),
    ("volume_up",        "tivoli", re.compile(r"(?:shengyin|yinliang)(?:tiao)?da|dadianshen|dashengyi?dian|taixiaosheng")),
    ("volume_down",      "tivoli", re.compile(r"(?:shengyin|yinliang)(?:tiao)?xiao|xiaodianshen|xiaoshengyi?dian|taichao")),
    ("mute",             "tivoli", re.compile(r"jingyin|xiaoyin|biechusheng")),
    ("music_favorites",  "music",  re.compile(r"wo(?:de)?shoucang|woxihuande(?:yinyue|ge)?|woshoucangde(?:ge|yinyue)?|hongxingequ")),
    ("music_next",       "music",  re.compile(r"xiayishou|xiashou|huanyishou|qiege")),
    ("music_prev",       "music",  re.compile(r"shangyishou|shangshou|qianyishou")),
    ("music_pause",      "music",  re.compile(r"zanting|tingyixia|xianting")),
    ("music_resume",     "music",  re.compile(r"jixu(?:bofang|fang)?|jiezhefang|jiezheting")),
    ("music_stop",       "music",  re.compile(r"tingzhibofang|biefangle|butingle|guandiaoyinyue")),
    ("music_now",        "music",  re.compile(r"shenmege|zhengzaifangde?shishenme|zheshishenmege")),
    ("max",      "light", re.compile(r"zuiliang|quanliang|liangdu?zuida")),
    ("min",      "light", re.compile(r"zuian|weiguang")),
    ("brighter", "light", re.compile(r"liangyi?dian|liangyixie|tiaoliang|zailiang|gengliang|taian")),
    ("dimmer",   "light", re.compile(r"anyi?dian|anyixie|tiaoan|zaian|gengan|tailiang|taicijing?")),
    ("warm",     "light", re.compile(r"nuanguang|nuanyidian|nuanse|huangguang|nuanbai")),
    ("cool",     "light", re.compile(r"lengguang|lengyidian|lengse|baiguang|lengbai")),
    ("reading",  "light", re.compile(r"kanshu|yuedu|dushu")),
    ("sleep",    "light", re.compile(r"yedeng|yueguang|shuimianmoshi")),
    ("off",      "light", re.compile(r"guan(?:diao|bi|shang|le)?(?:tai|diao|bi)?deng|badengguan|xideng|dengmiele")),
    ("on",       "light", re.compile(r"(?:da)?kai(?:yixia)?(?:tai|diao|bi)?deng|badengkai|dengliangqilai|taihei")),
]


def parse(text: str, last_domain: Domain | None = None) -> Intent:
    """解析一句话。last_domain 是上一次成功操作的设备，用来给泛化词消歧。"""
    t = re.sub(r"[\s，。！？、,.!?]", "", text or "")
    if not t:
        return Intent("none", reply="我没听清", raw=text)

    # 台名先于一切：用户说"放音乐之声"时，"放"会被 music_play 抢走
    if _STATION_RE is not None:
        m = _STATION_RE.search(t)
        if m:
            n = _STATION_NAMES[m.group(0)]
            return Intent("tivoli", "preset_recall", slots={"preset": n},
                          reply=f"切到{m.group(0)}", raw=text, rule="fm_station_name")

    hit = _match(t, _RULES, text, last_domain)
    if hit is not None:
        return hit

    # 字面没命中，退到拼音层。SenseVoice 把"台灯"听成"台等"、"暖光"听成"暖逛"
    # 这类同音错误在中文短指令里很常见，只在字面层做匹配会白白丢掉正确的意图。
    py = _to_pinyin(t)
    for pat, n in _STATION_PY:
        if pat.search(py):
            return Intent("tivoli", "preset_recall", slots={"preset": n},
                          reply=f"切到{station_name(n)}", raw=text, rule="fm_station_name-py")
    hit = _match(py, _PINYIN_RULES, text, last_domain, pinyin=True)
    if hit is not None:
        return hit

    # 到这里规则层已经放弃了。**不在这里调 LLM** —— parse 必须是同步、微秒级、
    # 无副作用的，调用方（executor.Router）才好决定值不值得为这一句等 1.5 秒。
    return Intent("none", reply="这个我还不会", raw=text)


def _match(t: str, rules, raw: str, last_domain: Domain | None,
           pinyin: bool = False) -> Intent | None:
    suffix = "-py" if pinyin else ""
    for name, _domain, pat in rules:
        m = pat.search(t)
        if not m:
            continue
        if name in _AMBIGUOUS:
            table = _AMBIGUOUS[name]
            choice = table.get(last_domain or "light") or table.get("light")
            if choice is None:
                continue
            return Intent(choice.domain, choice.action, slots=dict(choice.slots),
                          brightness_pct=choice.brightness_pct,
                          brightness_step_pct=choice.brightness_step_pct,
                          color_temp_kelvin=choice.color_temp_kelvin,
                          reply=choice.reply, raw=raw, rule=name + suffix)
        build = _BUILD.get(name)
        if build is None:
            continue
        # 拼音层的匹配对象跟字面层的分组结构不一样（歌名那类根本没法拼音匹配），
        # 所以拼音层只走"无参数"的构造器 —— 除了预设序号，它的分组是特意对齐的。
        intent = build(m, raw, name + suffix)
        if intent is not None:
            return intent
    return None


# ---------------------------------------------------------------- MultiNet 词表
#
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

# Tivoli 的候选词。**默认不发给板子** —— 上面那段教训说得很清楚，
# 没过声学回归的词进了表，表现是"注册成功、日志正常、永远不触发"，
# 而板子这时已经把兜底路径让给了这条按不动的命令词。
#
#   tools/mn_regress.sh 打开收音机 换一个台 …      # 先单独验
#   MULTINET_INCLUDE_CANDIDATES=1 重启服务端       # 过了再放进正式表
#
# 选词时已经按上面两条规律避了坑：
#   - 全部 ≥3 音节
#   - 「预设一…六」六条互相只差最后一个音节，正是规律 2 明令禁止的形状，
#     所以预设**不进板上词表**，只走服务端 ASR 兜底那条路。
#   - 「换一个台 / 上一个台」差两个音节（huan/shang + yi ge tai），勉强够开；
#     真验不过就砍掉「上一个台」，往回退的需求本来就低频。
_MULTINET_CANDIDATES: list[tuple[str, str | None]] = [
    ("打开收音机", None),
    ("关掉收音机", None),
    ("换一个台", None),
    ("上一个台", None),
    ("播放我的收藏", None),
    ("下一首歌", None),
    ("暂停播放", None),
    ("继续播放", None),
    ("声音大一点", None),
    ("声音小一点", None),
]


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
    table = list(_MULTINET_COMMANDS)
    if os.environ.get("MULTINET_INCLUDE_CANDIDATES", "").strip() in ("1", "true", "yes"):
        table += _MULTINET_CANDIDATES
    return [
        {"id": i, "text": text, "phonemes": override or _auto_phonemes(text)}
        for i, (text, override) in enumerate(table)
    ]


load_presets()


if __name__ == "__main__":
    cases = [
        ("打开台灯", None), ("把灯关了", None), ("亮一点", None), ("调到百分之八十", None),
        ("有点太刺眼了", None), ("换成暖光", None), ("我要看书", None),
        ("打开收音机", None), ("听广播", None), ("换个台", None), ("上一个台", None),
        ("切到预设三", None), ("预设5", None), ("把这个台存到预设二", None),
        ("声音大一点", None), ("太吵了", None), ("静音", None), ("关掉音响", None),
        ("播放王菲的红豆", None), ("播放《稻香》", None), ("放周杰伦的歌", None),
        ("播放我的收藏", None), ("下一首", None), ("暂停", None), ("继续播放", None),
        ("现在放的是什么", None), ("放点适合睡觉的歌", None),
        ("关掉", "music"), ("关掉", "tivoli"), ("关掉", "light"),
        ("下一个", "music"), ("下一个", "tivoli"),
        ("今天天气怎么样", None),
    ]
    print(f"{'说的话':<18}{'上次设备':<9}{'域':<8}{'动作':<16}{'规则':<22}槽位")
    for s, last in cases:
        i = parse(s, last)
        slots = {k: v for k, v in i.slots.items()}
        extra = {k: v for k, v in (("pct", i.brightness_pct), ("step", i.brightness_step_pct),
                                   ("k", i.color_temp_kelvin)) if v is not None}
        print(f"{s:<18}{str(last or '-'):<11}{i.domain:<9}{i.action:<17}{i.rule:<23}"
              f"{slots or ''}{extra or ''}")

    print("\n--- MultiNet 词表自检（每条都必须能解析出非 none 的意图）---")
    bad = 0
    os.environ["MULTINET_INCLUDE_CANDIDATES"] = "1"   # 候选词也一起查
    for c in commands_for_multinet():
        i = parse(c["text"])
        flag = "" if i.action != "none" else "   <<< 解析不出来"
        bad += i.action == "none"
        print(f"{c['id']:>2}  {c['text']:<12} {c['phonemes']:<26} -> {i.domain}.{i.action}{flag}")
    print("全部可解析" if not bad else f"{bad} 条词表和规则脱节了")

    print("\n注音对不对，靠 selftest.py 的闭环校验（TTS 念一遍再听回来比对），"
          "不靠这里的静态检查 —— 静态查多音字只会把「灯(deng/ding)」这种"
          "谁都不会念错的也报出来，噪声盖过信号。")
