"""对话消歧的回归测试。不碰网络、不碰设备。

为什么值得单独一个文件：这一层的 bug 全都是"看起来执行成功了，只是做在了
别的设备上" —— 日志绿着，用户房间变热。这种 bug 不会自己冒出来，
只能靠把场景摆出来一条条断言。

跑：  server/.venv/bin/python -m pytest test_dialogue.py -q
或：  server/.venv/bin/python test_dialogue.py
"""
from __future__ import annotations

import asyncio

from context import Context
import intent as intent_mod


def _ctx(**world) -> Context:
    c = Context()
    for k, v in world.items():
        setattr(c.world, k, v)
    for d in ("light", "music", "tivoli", "aircon"):
        if c.world.active(d):
            c.world.touch(d)
    return c


DOMAINS = ("light", "music", "tivoli", "aircon")


def parse(text: str, ctx: Context):
    return intent_mod.parse(text, ctx.rank(DOMAINS))


# ---------------------------------------------------------------- 世界优先

def test_音乐在放时大一点是音量_不是灯():
    """改造前：last_domain=None -> 灯亮度。这正是用户报的那一类误伤。"""
    i = parse("大一点", _ctx(music_playing=True))
    assert (i.domain, i.action) == ("tivoli", "volume_step"), i


def test_只有灯开着时大一点是亮度():
    i = parse("大一点", _ctx(light_on=True))
    assert (i.domain, i.action) == ("light", "brightness_step"), i


def test_只有空调开着时大一点是温度():
    i = parse("大一点", _ctx(aircon_on=True))
    assert (i.domain, i.action) == ("aircon", "temp_step"), i


def test_话题不能压过世界():
    """上一句聊的是空调，但音乐正放着 —— 说「大一点」该调音量。

    改造前这里必然错：last_domain=aircon，房间变热。
    这是整个改动最核心的一条断言。
    """
    c = _ctx(music_playing=True, aircon_on=False)
    c.record("空调关掉", "aircon", "off", True, "空调关了")
    i = parse("大一点", c)
    assert i.domain == "tivoli", i


def test_都不活着时退回灯_行为不退化():
    """世界一无所知时，表现必须和改造前一模一样，否则是纯粹的倒退。"""
    i = parse("大一点", Context())
    assert (i.domain, i.action) == ("light", "brightness_step"), i


def test_关掉跟着正在响的东西走():
    assert parse("关掉", _ctx(music_playing=True)).domain == "music"
    assert parse("关掉", _ctx(light_on=True)).domain == "light"


def test_不知道不等于关着():
    """Tivoli 的状态全是影子，一开始就是 None。

    None 必须排在"确定关着"前面 —— 否则一台我们只是没把握的设备，
    会被当成明确不在候选里，而这正是影子状态最危险的误用方式。
    """
    c = _ctx(light_on=False)          # 灯确定关着
    c.world.tivoli_powered = None     # 音响不知道
    order = c.rank(["light", "tivoli"])
    assert order[0] == "tivoli", order


# ---------------------------------------------------------------- 该不该反问

def test_只有一个活着就别问():
    assert _ctx(music_playing=True).ambiguous(["light", "music"]) == []


def test_两个都活着且话题帮不上忙就该问():
    c = _ctx(light_on=True, music_playing=True)
    assert set(c.ambiguous(["light", "music", "aircon"])) == {"light", "music"}


def test_话题能帮上忙就别问():
    """灯和音乐都开着，但上一句刚聊过灯 —— 猜灯，别问。

    问多了比猜错还烦，这是这套交互能不能用的分界线。
    """
    c = _ctx(light_on=True, music_playing=True)
    c.record("开灯", "light", "on", True, "灯开了")
    assert c.ambiguous(["light", "music"]) == []


def test_不含糊的词永远不问():
    """「暗一点」自己就带着设备信息，不该因为音乐在放就去问。"""
    assert intent_mod.ambiguous_domains("dimmer") == []
    i = parse("暗一点", _ctx(light_on=True, music_playing=True))
    assert i.domain == "light", i


# ---------------------------------------------------------------- 选歌消歧

def test_选项解析():
    from netease import Song
    from player import _parse_choice
    songs = [Song(id=1, name="花儿与少年", artists=["黑鸭子"]),
             Song(id=2, name="花儿与少年", artists=["龚玥"]),
             Song(id=3, name="花儿与少年", artists=["中央民族乐团"])]
    assert _parse_choice("第二个", songs) == 1
    assert _parse_choice("第三首", songs) == 2
    assert _parse_choice("要黑鸭子的", songs) == 0
    assert _parse_choice("都不要", songs) == "cancel"
    # 最关键的一条：不是在回答的话必须还回 None，
    # 否则用户在问题悬着时说「打开台灯」会石沉大海。
    assert _parse_choice("打开台灯", songs) is None
    assert _parse_choice("放点别的", songs) is None


def test_不要为一个明显的赢家提问():
    """搜「红豆」王菲热度甩开第二名 —— 这种情况问一句纯属添乱。"""
    from netease import Song
    from player import MusicExecutor
    m = MusicExecutor.__new__(MusicExecutor)
    m.pending = None
    songs = [Song(id=1, name="红豆", artists=["王菲"], pop=100),
             Song(id=2, name="红豆", artists=["某翻唱"], pop=40)]
    picked, reply = m._pick_or_ask(songs, None, "红豆")
    assert m.pending is None and len(picked) == 1 and picked[0].artist == "王菲", reply


def test_热度分不开就问():
    from netease import Song
    from player import MusicExecutor
    m = MusicExecutor.__new__(MusicExecutor)
    m.pending = None
    songs = [Song(id=i, name="花儿与少年", artists=[a], pop=p)
             for i, (a, p) in enumerate([("黑鸭子", 55), ("龚玥", 52), ("中央民族乐团", 50)])]
    picked, reply = m._pick_or_ask(songs, None, "花儿与少年")
    assert picked == [] and m.pending is not None, reply
    assert "要哪个" in reply and "黑鸭子" in reply, reply


def test_指名的歌手一个都没搜到就问_不要默默放翻唱():
    """以前是放个翻唱再说一句"网易云上没有周杰伦的…"。

    "放错了还告诉你" 和 "问一句再放对" 之间，显然是后者。
    """
    from netease import Song
    from player import MusicExecutor
    m = MusicExecutor.__new__(MusicExecutor)
    m.pending = None
    songs = [Song(id=1, name="花儿与少年", artists=["黑鸭子"], pop=55),
             Song(id=2, name="花儿与少年", artists=["龚玥"], pop=52)]
    picked, reply = m._pick_or_ask(songs, "中央歌舞团", "花儿与少年")
    assert picked == [] and m.pending is not None, reply
    assert "中央歌舞团" in reply, reply


def test_指名的歌手搜到了就直接放():
    from netease import Song
    from player import MusicExecutor
    m = MusicExecutor.__new__(MusicExecutor)
    m.pending = None
    songs = [Song(id=1, name="稻香", artists=["周杰伦"], pop=90),
             Song(id=2, name="稻香", artists=["某翻唱"], pop=95)]
    picked, reply = m._pick_or_ask(songs, "周杰伦", "稻香")
    assert m.pending is None and picked[0].artist == "周杰伦", reply


def test_歌名完全相同的要压过名字里带的():
    """实测踩到的：搜「周杰伦 稻香」时，一个上传者昵称叫"周杰伦."的
    《稻香(治愈版)》因为热度高，盖过了所有歌名正好是「稻香」的结果。

    然后系统说"播放周杰伦的《稻香》"—— 这比放错更糟，它在骗人。
    """
    from netease import Song
    from player import MusicExecutor
    m = MusicExecutor.__new__(MusicExecutor)
    m.pending = None
    songs = [Song(id=1, name="稻香(治愈版)", artists=["周杰伦."], pop=100),
             Song(id=2, name="稻香", artists=["周杰伦"], pop=40)]
    picked, reply = m._pick_or_ask(songs, "周杰伦", "稻香")
    assert picked and picked[0].id == 2, reply


def test_歌手名只是包含不算命中():
    """"周杰伦." 含 "周杰伦"，但它不是周杰伦。

    这一步的后果是"不问就放"，门槛必须比展示用的 _is_artist 高。
    网易云上冒用原唱名字的上传者非常多。
    """
    from netease import Song
    from player import MusicExecutor
    m = MusicExecutor.__new__(MusicExecutor)
    m.pending = None
    songs = [Song(id=1, name="稻香", artists=["周杰伦."], pop=100),
             Song(id=2, name="稻香", artists=["Lucky小爱"], pop=90)]
    picked, reply = m._pick_or_ask(songs, "周杰伦", "稻香")
    assert picked == [] and m.pending is not None, reply
    assert "没有周杰伦的" in reply, reply


def test_选项必须念得完():
    """搜「花儿与少年」回来全是综艺主题曲，一首挂着七到九个演员。

    三个选项照实念要二十多秒，用户早忘了第一个是什么 —— 念不完 = 没问。
    """
    from netease import Song
    from player import MusicExecutor
    m = MusicExecutor.__new__(MusicExecutor)
    m.pending = None
    cast = ["华晨宇", "刘涛", "张翰", "郑佩佩", "张凯丽", "许晴", "李菲儿"]
    songs = [Song(id=i, name="花儿与少年", artists=list(a), pop=55 - i)
             for i, a in enumerate([cast, cast[:1] + ["那英"], ["龚玥"]])]
    _, reply = m._pick_or_ask(songs, None, "花儿与少年")
    assert "等7人" in reply, reply
    assert len(reply) < 60, f"问句太长（{len(reply)}字）：{reply}"


# ---------------------------------------------------------------- 端点 / 识别可信度

def test_STT说不可信时不要往下走():
    """SenseVoice 的 🎼 是「这段是音乐不是说话」。

    实测：板子送上来一段被截断的录音，服务端听成「🎼空调。」，
    我们照单全收 —— 过意图层、过拼音层、再花 2.6 秒问 LLM，
    最后回一句「这个我还不会」。模型第一个字符就告诉答案了。
    """
    from stt import STT
    assert STT.unreliable("🎼空调。")
    assert STT.unreliable("🎼")
    # 音乐里夹着一句听清了的话，该照常执行 —— 标记说的是「有音乐」，
    # 不是「这几个字错了」
    assert STT.unreliable("🎼打开台灯。") is None
    assert STT.unreliable("空调调低一点。") is None


def test_情绪标记不算不可信():
    """😊😔😡 说明这**是**语音（还带情绪），和 🎼 含义相反。

    第一版按 Unicode 区段一刀切，把「😊好的」判成了不可信 ——
    而「好的」正是回答问题时最常说的一句。
    """
    from stt import STT
    for t in ("😊好的", "😔算了", "😮对"):
        assert STT.unreliable(t) is None, t


# ---------------------------------------------------------------- 语序

def test_空调两种语序都要收():
    """中文允许动宾和主谓两种语序，规则表只写一种就会漏。

    `ac_off` 两个方向都有而 `ac_on` 只有一个 —— 说明这坑早被踩过一次，
    但当时只补了被踩到的那一条。同形状的邻居没一起补，是这份表最容易漏的地方。
    """
    for t in ("调低空调", "把空调调低", "空调调低一点"):
        i = parse(t, Context())
        assert (i.domain, i.action) == ("aircon", "temp_step"), (t, i)
        assert i.slots.get("step") == -1, (t, i)
    for t in ("空调开一下", "空调打开", "开空调"):
        i = parse(t, Context())
        assert (i.domain, i.action) == ("aircon", "on"), (t, i)


def test_多音字调要认两个读音():
    """STT 的同音错字会把整句的分词打乱，多音字跟着换读音。

    「调低空调」被听成「调地空调」之后，"调地" 不在 pinyin_fix 的词典里，
    「调」回落成默认的 diào，整条拼音规则当场失配。
    更狠的是「掉高空调」-> diaogaokong**diao** —— 连词尾「空调」里那个「调」
    都被句首的错字带偏了。多音字在这儿是**位置性**问题，不是词汇性问题。
    """
    for t in ("调地空调", "掉低空调", "掉高空调"):
        i = parse(t, Context())
        assert (i.domain, i.action) == ("aircon", "temp_step"), (t, i)
    assert parse("掉亮一点", Context()).domain == "light"


def test_音量词不能被空调抢走():
    """实测报过的 bug：说「声音低一点」回「已调低到23度」。"""
    c = _ctx(aircon_on=True)
    c.record("空调调低一点", "aircon", "temp_step", True, "调低到25度")
    for t in ("声音低一点", "调低音量", "太吵了"):
        i = parse(t, c)
        assert i.domain == "tivoli", (t, i)


def test_灯也要收两种语序():
    """全面体检时扫出来的：空调那条补过之后，灯这条同形状的还漏着。

    补一处、不补它同形状的邻居，是这份规则表最稳定的漏法。
    """
    for t in ("把灯打开", "台灯打开", "灯打开", "打开台灯"):
        assert parse(t, Context()).action == "on", t
    for t in ("把台灯关掉", "灯关掉", "关闭台灯"):
        assert parse(t, Context()).action == "off", t


def test_温度可以不带度字_但不能抢走灯的百分比():
    """「空调调到26」实测落到了 set_pct，把灯调成了 26%。

    STT 很容易把句尾轻读的「度」吞掉，所以这个说法必须收；
    但裸的「调到26」仍然该归灯 —— 区别就在有没有「空调」两个字。
    """
    for t in ("空调调到26", "空调调到26度", "空调26"):
        i = parse(t, Context())
        assert (i.domain, i.slots.get("temp")) == ("aircon", 26), (t, i)
    assert parse("调到26", Context()).domain == "light"
    assert parse("亮度调到60", Context()).domain == "light"


def test_空调不许短周期启停():
    """压缩机短周期启停会真的把它做坏 —— 刚停机时冷媒两侧压力还没平衡。

    加这条是因为一次自己造成的事故：三台设备的体检脚本连着跑了两轮
    「打开空调 … 关闭空调」，几十秒内开关四次，用户当场喊停。
    光记住"以后别这么干"是不够的，约束要写进代码。
    """
    import time as _t
    from aircon import AirconExecutor
    a = AirconExecutor.__new__(AirconExecutor)
    a.believed_on, a._last_power_at = None, 0.0
    assert a._power_guard(True) is None, "不知道状态时不该拦"

    a.believed_on, a._last_power_at = True, _t.time()
    assert a._power_guard(False), "刚开机就要关，必须拦"
    assert a._power_guard(True) is None, "同状态不是翻转，不该拦"

    a._last_power_at = _t.time() - AirconExecutor._POWER_GUARD_S - 1
    assert a._power_guard(False) is None, "过了保护期该放行"


def test_问题会过期():
    import time as _t
    from context import Question
    q = Question(kind="domain", options=[], text="灯还是音乐？", deadline=_t.time() - 1)
    assert q.expired


def _run() -> int:
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            r = fn()
            if asyncio.iscoroutine(r):
                asyncio.run(r)
            print(f"  ✓ {name}")
        except AssertionError as exc:
            fails += 1
            print(f"  ✗ {name}\n      {exc}")
    print(f"\n{'全过' if not fails else str(fails) + ' 条没过'}")
    return fails


if __name__ == "__main__":
    raise SystemExit(_run())
