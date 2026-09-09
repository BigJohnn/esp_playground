"""给 pypinyin 打的一处多音字补丁，TTS 前端和意图解析共用。

背景：pypinyin 单字「调」的默认读音是 diao4，词典里也没有「调到」。于是
  - MultiNet 的注音变成 "diao dao"，人念 tiao 就永远对不上（M3 实测：注册成功、
    出现在 ACTIVE 列表、却一次都不触发）；
  - Kokoro 的中文前端 misaki.zh 就是 jieba + lazy_pinyin，同一个错误让它把
    「调亮了」念成「掉亮了」。

同一个根因，两处发作，所以补丁只打一次、放在两边都会经过的地方。

要同时补两层才管用：
  load_phrases_dict  给词加正确读音；
  jieba.add_word     让分词先切出这个词 —— 否则「已调到最亮」会被切成
                     ['已调','到','最亮']，'调到' 根本没机会被查到。
"""
from __future__ import annotations

# 只收「调」表示"调节"的搭配。不动单字默认值：那会把「调查」「语调」一起念坏。
_PHRASES: dict[str, list[list[str]]] = {
    "调到": [["tiáo"], ["dào"]],
    "调亮": [["tiáo"], ["liàng"]],
    "调暗": [["tiáo"], ["àn"]],
    "调高": [["tiáo"], ["gāo"]],
    "调低": [["tiáo"], ["dī"]],
    "调大": [["tiáo"], ["dà"]],
    "调小": [["tiáo"], ["xiǎo"]],
    "调成": [["tiáo"], ["chéng"]],
    "调节": [["tiáo"], ["jié"]],
    "调整": [["tiáo"], ["zhěng"]],
}

_applied = False


def apply() -> None:
    """幂等。第一次调用会把 jieba 拉起来（约 0.4s），之后几乎零开销。"""
    global _applied
    if _applied:
        return
    from pypinyin import load_phrases_dict

    load_phrases_dict(_PHRASES)
    try:
        import jieba

        for w in _PHRASES:
            jieba.add_word(w, freq=100000)
    except ImportError:
        # 没装 jieba 也能用：词典那一半照样生效，只是长句里可能切不出来。
        pass
    _applied = True
