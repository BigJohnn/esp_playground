# S31 Voice 架构

> 配套 `README.md`。README 记的是**怎么走到这一步的**（踩过的坑、推翻过的结论），
> 这份记的是**现在长什么样**。两边冲突时以代码为准，然后来改这里。
>
> 最后核对：2026-09-16，对着 `server/*.py` 和 `firmware/main/*.c` 逐条验证过。
> §4 的三个缺陷已于同日修复，§6 的延迟账是实测的。

---

## 0. 一句话

> **LLM 不是意图层的入口，是它的兜底。**
> 设备判断（台灯 / Tivoli / 空调 / 音乐）在句子里有明确设备词时由规则层用微秒级正则完成，
> 置信度 1.0；只有句子**含糊**（「关掉」「大一点」）或**完全没命中**时，才轮到本地 LLM。

这不是省事，是刻意的——理由见 §3.4。

---

## 1. 全链路

```mermaid
flowchart LR
  subgraph BOARD["ESP32-S3 板子 (firmware/main)"]
    MIC["麦克风阵列<br/>AFE + NS + VAD"]
    WN["WakeNet<br/>wn9_nihaoxiaozhi_tts"]
    MN["MultiNet<br/>mn7_cn · 12 条词表"]
    SPK["喇叭<br/>TTS 回放 / 提示音"]
    MIC --> WN --> MN
  end

  subgraph NET["HTTP (:8790)"]
    direction TB
    E1["POST /wake<br/>唤醒即发，压低音乐"]
    E2["POST /command<br/>板上认出的整句"]
    E3["POST /utterance<br/>没认出来，整段 PCM"]
    E4["POST /tts<br/>回话合成"]
    E5["GET /commands<br/>词表下发 + 版本轮询"]
  end

  subgraph SERVER["服务端 (server/)"]
    STT["SenseVoice STT<br/>stt.py"]
    ROUTER["Router.parse_and_execute<br/>executor.py:273"]
    TTS["Kokoro TTS<br/>tts.py · zm_011"]
  end

  subgraph EXEC["执行器"]
    LX["LightExecutor<br/>executor.py:28"]
    TX["TivoliExecutor<br/>tivoli.py:88"]
    MX["MusicExecutor<br/>player.py:156"]
    AX["AirconExecutor<br/>aircon.py:34"]
  end

  subgraph HW["真会动的硬件"]
    LAMP["米家台灯<br/>miIO 局域网直连"]
    TIVOLI["Tivoli 音响<br/>红外 + AirPlay/RAOP"]
    AC["空调<br/>Coolix 绝对状态帧"]
    ESPH["ESPHome tivoli-ir 板<br/>发射 GPIO4 · 接收 GPIO14"]
  end

  WN -.唤醒即发.-> E1
  MN -- 命中 --> E2
  MN -- 没命中<br/>整段音频 --> E3
  E1 & E2 --> ROUTER
  E3 --> STT --> ROUTER
  ROUTER --> LX & TX & MX & AX
  LX --> LAMP
  TX --> ESPH --> TIVOLI
  MX -- RAOP 音频流 --> TIVOLI
  MX -. 借 TX 调音量 .-> TX
  AX --> ESPH --> AC
  ROUTER -- reply 文本 --> E4 --> TTS --> SPK
  E5 -.-> MN
  E1 -- "压低音量<br/>（只在输出指着 Tivoli 时）" --> VOL
  TX -. "set_volume / volume_step<br/>链路在就走网络，不走红外" .-> VOL
  VOL["macOS 系统音量<br/>**一个格子，两个写者**<br/>见 §6.6"]
  VOL --> TIVOLI

  classDef gap fill:#fee,stroke:#c33,stroke-width:2px
  classDef shared fill:#fff3cd,stroke:#c90,stroke-width:2px
  class ESPH gap
  class VOL shared
```

**红框那块**是目前的结构性依赖：ESPHome 那块板同时是 Tivoli 的红外发射器**和**空调的
Coolix climate 实体（带接收机，好让有人用实体遥控器时 HA 跟得上）。
M21 想做的"单主控"因此不成立——见 §5。

---

## 2. 意图三层

```mermaid
flowchart TD
  IN(["一句中文"]) --> Q{"有悬着的问题？<br/>ctx.question"}
  Q -- 是 --> ANS["_answer_slot / _answer_domain<br/>拿**原话**重解，不让用户重说"]
  ANS --> EXEC
  Q -- 否 --> W["_refresh_world()<br/>只读进程内状态，一次网络都不发"]
  W --> RANK["ctx.rank()<br/>候选顺序按**谁此刻活着**排<br/>不是按上次说的是谁"]
  RANK --> L1

  subgraph L1G["第 1 层 · 规则（微秒级）"]
    L1["intent.parse()<br/>intent.py:615"]
  end

  L1 --> CLR{"action == clarify？"}
  CLR -- "「空调」这种只说了设备名" --> ASKS["_ask_slot<br/>「空调要怎么样？」"]
  CLR -- 否 --> CONF["若 via 非空：<br/>confidence = ctx.confidence_for(via)"]

  CONF --> AMB{"两台以上**确定活着**？<br/>ctx.ambiguous()"}
  AMB -- 是 --> ASKD["_ask_domain<br/>「灯还是音乐？」"]
  AMB -- 否 --> L2

  subgraph L2G["第 2 层 · 拼音（同在 parse 内）"]
    L2["_to_pinyin + 拼音规则<br/>救 STT 的同音错字"]
  end

  L2 --> GATE{"domain == none<br/>或 confidence < 0.6？"}
  GATE -- 否 --> EXEC
  GATE -- 是 --> L3

  subgraph L3G["第 3 层 · 本地 LLM（硬超时 2.6s）"]
    L3["llm.classify()<br/>Qwen3-4B + JSON schema 约束"]
    WL["白名单 _ALLOWED<br/>编出来的动作一律丢掉"]
    VS["_validated_slots()<br/>schema 管生成，这里管执行"]
    L3 --> WL --> VS
  end

  VS --> EXEC["Router.execute()<br/>→ 对应执行器"]
  EXEC --> REC["ctx.record() + _note_effect()<br/>做成的动作**本身就是**关于世界的最新消息"]
  REC --> OUT(["reply 文本 → TTS"])

  classDef ask fill:#e8f0ff,stroke:#36c
  class ASKS,ASKD ask
```

### 2.1 置信度是"结论怎么来的"，不是概率

| 分值 | 含义 | 例子 |
|---|---|---|
| 1.0 | 句子里有明确设备词 | 「打开**台灯**」「把**空调**调到26度」 |
| 0.9 | 泛化词，但只有一台设备确实活着 | 放歌时说「大一点」 |
| 0.7 | 泛化词，靠"上一句聊的是它" | 话题是真信号，但比世界弱 |
| 0.6 | LLM 给的 | — |
| 0.4 | 泛化词，谁都没活着，纯兜底默认 | 开机就说「大一点」 |

`_ASK_LLM_BELOW = 0.6`（`executor.py:188`）——0.4 那一档会去问 LLM，0.7 不会。

---

## 3. 设备判断：实测四个域都对

`./.venv/bin/python -c "import intent; ..."` 的实际输出（2026-09-16）：

| 说的话 | domain.action | rule | via | slots |
|---|---|---|---|---|
| 打开台灯 | `light.on` | `on` | — | — |
| 播放王菲的《红豆》 | `music.play` | `music_play` | — | `artist=王菲 title=红豆` |
| 把空调调到26度 | `aircon.set_temp` | `ac_set_temp` | — | `temp=26` |
| 这首太吵了 | `tivoli.volume_step` | `volume_down` | — | `step=-3` |
| 把收音机声音关小 | `tivoli.volume_step` | `volume_down` | — | `step=-3` |
| 放点适合睡觉的歌 | `music.play` | `music_play` | — | `vibe=True` |
| 关掉 | `light.off` | `amb_off` | `light` | 含糊 → 降置信 |
| 大一点 | `light.brightness_step` | `amb_higher` | `light` | 含糊 → 降置信 |
| 空调 | `none.clarify` | `bare_device` | — | `device=空调` → 反问 |

**四个域在有设备词时全部 conf=1.0，LLM 一次都没被调用。** 这正是设计意图。

### 3.1 复杂点歌：「播放王菲的《红豆》」

```mermaid
sequenceDiagram
  autonumber
  participant U as 用户
  participant B as 板子
  participant S as /utterance
  participant R as Router
  participant I as intent.py
  participant P as MusicExecutor
  participant N as 网易云
  participant T as Tivoli

  U->>B: 你好小智 → 播放王菲的《红豆》
  Note over B: MultiNet 没这条词<br/>整段 PCM 兜底
  B->>S: POST /utterance (16k PCM)
  S->>S: SenseVoice → "播放王菲的《红豆》"
  S->>S: STT.unreliable()? 否
  S->>R: parse_and_execute
  R->>I: parse(text, rank())
  Note over I: _BOOK_RE 先吃书名号<br/>（用户自己给的边界，<br/>比任何启发式都准）
  I-->>R: music.play artist=王菲 title=红豆 conf=1.0
  Note over R: conf=1.0 且 domain≠none<br/>**LLM 不介入**
  R->>P: execute()
  P->>N: search("王菲 红豆", limit=30)
  P->>N: songs([ids])
  Note over P,N: 多一次往返换 pop 热度字段——<br/>搜索排名把翻唱排在王菲前面，<br/>按 pop 排立刻就对
  P->>P: _pick_or_ask()
  alt 唯一命中
    P->>N: url(song_id)
    P->>T: AirPlay/RAOP 流
    P-->>U: 「放王菲的《红豆》」
  else 多个难分
    P-->>U: 「有王菲的和某某的，要哪个？」
    Note over P: Pending 挂住，<br/>下一句走 music.answer()
  end
```

**关键点：这条路 LLM 根本没上场。** 规则层抽槽 + 曲库裁决比 LLM 强——
LLM 不知道网易云上「红豆」有几十个翻唱版本，而 `pop` 字段知道。
`_resolve()` 里还有一条"整句本身就是歌名"的裁决（`player.py:580`），
专治「月亮代表我的心」被"的"劈成 歌手=月亮代表我 / 歌名=心。

### 3.2 含糊句：「关掉」

```mermaid
sequenceDiagram
  autonumber
  participant U as 用户
  participant R as Router
  participant C as Context
  participant L as LocalLLM

  U->>R: 关掉
  R->>C: _refresh_world()<br/>（只读进程内，零网络）
  C-->>R: 音乐在放 / 灯开着 / 空调未知
  R->>C: rank(["light","music","tivoli","aircon"])
  R->>R: parse → light.off, via="light"
  R->>C: confidence_for("light")
  alt 灯和音乐**都确定活着**
    C-->>R: ambiguous
    R-->>U: 「灯还是音乐？」
    Note over R: 原话存进 q.options["pending_text"]<br/>用户答「音乐」后拿**原话**重解
  else 只有一台活着
    C-->>R: conf=0.9 → 直接执行
  else 谁都没活着
    C-->>R: conf=0.4 < 0.6
    R->>L: classify(text, ctx)
    Note over L: 提示词带**现场情况**<br/>这是它能解指代的前提
    L-->>R: 或结果，或超时（超时就当没这层）
  end
```

---

## 4. 三个缺陷（2026-09-16 已修）

这三条都会让系统**把指令发到错的设备上**或者**该发不发**。
回归在 `server/test_control_safety.py`（22 条）。

### 4.1 `_ACTION_HOME` 的跨设备修复是无条件的

`llm.py:145` 按"动作唯一属于哪个域"把 LLM 说错的域修回来。实测映射：

```
brightness_step → light      volume_step → tivoli      temp_step → aircon
```

修复本身是对的（「把收音机声音关小」LLM 答 `music.volume_step`，修成 tivoli 正确），
但它**不看原话**。如果 LLM 对着「这首太吵了」吐出 `music.brightness_step`，
`_ACTION_HOME` 会把它改成 `light.brightness_step` —— **去调台灯亮度**。
句子里的 domain 信号（「这首」= 音乐）是强的，幻觉出来的是 action，
而代码选择了相信 action。

> 现在这句被规则层以 conf=1.0 接住了（见 §3 表），所以暂时走不到这条路。
> 但这是巧合，不是防护。

**已修**：修复前先看原话**佐证**了哪个域。
判据是"它说的域被佐证了，而要修去的那个域没有" —— 这时候错的是动作，丢掉。

后半个条件是拿反例逼出来的：只判"原域被佐证就丢"会误伤
「这首歌音量小一点」（music 被"歌"佐证，但 tivoli 也被"音量"佐证，
而 volume_step 确实只有 tivoli 有）。两边都被提到时，动作才是分得开的信号。

### 4.2 合法但凭空来的数值

最初以为只是 `light.on` 的可选槽。**拿真模型一跑，面大得多**——
2026-09-16 qwen3:4b 实测，十句里七句带着 `"pct":0,"kelvin":0,"temp":0` 这样的填充值，
而其中一条是会出事的：

```
「太亮了受不了」 -> light.brightness pct=0     ← 必填槽，不是可选槽
```

`0` 落在 `[0,100]` 之内，范围检查一个字都挑不出来，灯被设成最低亮度——
一个用户从没说过的**绝对值**。编的动作有白名单挡着，编的数值长得和真数值一模一样。

**已修**：`_SLOT_EVIDENCE` 对 `pct/kelvin/temp/preset` 四个绝对值槽位要求原话依据，
必填可选一视同仁。没依据时：可选槽丢槽位保动作（「开灯」本身是对的），
必填槽转成"缺参"走 §4.3 的反问。

> 刻意不收裸中文数词：「调暗一点」「开一下」里的"一"会让任何填充值都过关。

### 4.3 缺参数时丢弃结果，而不是反问

```
_validated_slots({}, 'aircon', 'set_temp')  ->  None
_validated_slots({}, 'music',  'play')      ->  None
```

`None` 一路传回 `classify()` → `classify` 返回 `None` → `parsed` 保持规则层的
`domain=none` → `execute()` 回 **「这个我还不会」**（`executor.py:454`）。

但我们**明明知道**用户在说空调、在说要设温度，只是不知道设到几度。
`_ask_slot()` 那套机制就在旁边（`executor.py:376`），却没被接上。
这跟 `_ask_slot` 自己的注释是同一个毛病——那条注释写的是：

> 以前这里回的是"这个我还不会"，而那句话是**假的**：我们明明知道他在说空调。

**已修**：`_validated_slots()` 改成返回 `(槽位, 缺的键)`，三种结果分开：

| 返回 | 含义 | 该做的事 |
|---|---|---|
| `({...}, "")` | 拿到了 | 执行 |
| `(None, "temp")` | 该给的没给 | **问回去** |
| `(None, "")` | 给了但不合法 | 丢掉（幻觉信号） |

三道关的**顺序有讲究**，排错了会把"乱编"也变成"问回去"：
没给→问、类型/范围→丢、原话依据→问。（第一版就排错了，`pct=101` 被问成了"亮度调到多少？"）

Router 那边新增 `_ask_param` / `_answer_param`。不能复用 `_answer_slot`：
那条路是把域顶到最前面**重解整句**，而这里用户补的是一个**裸值**——
实测 `parse("26度")` 解不出任何意图（要「调到26度」才行）。

`step` 刻意不可问：「调高还是调低」本身就是这条指令的全部内容，
给不出方向说明模型压根没听懂，那是该丢的。问"要调亮还是调暗？"
只是把一次失败包装成一次对话。

```mermaid
flowchart LR
  A["LLM: aircon.set_temp<br/>temp 缺失"] --> B{"现在"}
  B --> C["_validated_slots → None"] --> D["classify → None"] --> E["「这个我还不会」"]
  B2["同一个输入"] --> F{"该改成"}
  F --> G["missing=['temp']"] --> H["_ask_slot"] --> I["「空调调到多少度？」"]
  classDef bad fill:#fee,stroke:#c33
  classDef good fill:#efe,stroke:#3a3
  class C,D,E bad
  class G,H,I good
```

---

## 5. 红外：M21 的前提已经不成立

```mermaid
flowchart TB
  subgraph NOW["现在"]
    S1["服务端 Router"] --> T1["TivoliExecutor.press()<br/>tivoli.py:100"]
    T1 --> HA1["HA button.tivoli_ir_*"] --> EH1["ESPHome tivoli-ir"]
    EH1 -- NEC 0x6B86 --> TV1["Tivoli"]
    EH1 -- Coolix --> AC1["空调"]
    AC1 -. 实体遥控器 .-> EH1
  end

  subgraph B["方案 B：服务端解析 + 板子发射"]
    S2["服务端 Router"] --> T2["press() 换实现"]
    T2 -- "POST /ir 推给板子<br/>（板子需新开 http server）" --> ESP["S3 · RMT 38kHz<br/>ir_nec.c"]
    ESP --> TV2["Tivoli"]
    EH2["ESPHome 板<br/>**仍然拿不掉**"] -- Coolix + 接收 --> AC2["空调"]
  end

  classDef blocked fill:#fee,stroke:#c33,stroke-width:2px
  class EH2 blocked
```

| | 现在 | 方案 B | M21 原本想要的 |
|---|---|---|---|
| Tivoli 红外 | ESPHome 板 | S3 板 | S3 板 |
| 空调 Coolix + 接收 | ESPHome 板 | **ESPHome 板** | — |
| 能不能砍掉第二块板 | — | **不能** | 能（前提错了） |
| 断网可用 | 否 | 否 | 是 |
| 物理约束 | 麦克风和发射器可分开放 | **S3 必须同时听得见人、照得到 Tivoli** | 同左 |

板端本地识别（`CONFIG_S31_LOCAL_IR_ENABLED`）实测两对候选词都是 **0/6**，
而对照组「打开台灯/关闭台灯」在同一条路径上 **6/6**——是词的问题不是路径的问题，
原因未查明。详见 `README.md` §4.1.26。

**结论：M21 暂停。** 真要往下做，第一件事是拿到 J2 的原理图确定 TX 驱动电路和 GPIO——
那是唯一的硬阻塞，换方案不会让它自己好。

### 验收有个现成的见证人

ESPHome 板的 `remote_receiver`（GPIO14，`dump: all`）能解 NEC。
让 S3 发、它收，比对 address / command / ditto 数量。
这是这条**开环**链路上难得能拿到客观证据的机会——
比"听着好像大声了"可靠得多。

---

## 6. 延迟：为什么「打开台灯」快、「开灯」慢

用户观察到的现象是真的，但它**不是延迟问题，是路由问题**。
两句话走的是完全不同的两条路。

```mermaid
flowchart TD
  W(["唤醒后说了一句话"]) --> MN{"这句话在板上<br/>那 12 条词表里吗？"}

  MN -- "「打开台灯」在<br/>（词表第 1 条）" --> F1["MultiNet 板上命中<br/>≈ 0 ms"]
  F1 --> F2["POST /command<br/>只传文本，几十字节"]
  F2 --> F3["规则层 微秒级"]
  F3 --> F4["miIO 一个 UDP 往返<br/>≈ 300–500 ms"]
  F4 --> FAST(["≈ 400 ms"])

  MN -- "「开灯」不在" --> S1["等 VAD 判说完<br/>掐尾 ≈ 380 ms"]
  S1 --> S2["上传 74 KB PCM<br/>≈ 100 ms"]
  S2 --> S3["SenseVoice STT<br/>≈ 780 ms（闲）"]
  S3 --> S4["规则层 微秒级<br/>（认得「开灯」，和快路一模一样）"]
  S4 --> S5["miIO<br/>≈ 300–500 ms"]
  S5 --> SLOW(["≈ 1600 ms（闲）<br/>2400 ms（机器忙）"])

  classDef fast fill:#efe,stroke:#3a3,stroke-width:2px
  classDef slow fill:#fee,stroke:#c33,stroke-width:2px
  class FAST fast
  class SLOW slow
```

**两条路的意图层和执行层是同一段代码**，一毫秒都不差。
差的全在"板子怎么知道你说了什么"。

### 6.1 哪些是不可压缩的（2026-09-16 实测）

板子要知道你说了什么，物理上只有两条路：**本地有这个词**，或者**把音频送出去**。
后者的每一段都有硬下限：

| 环节 | 实测 | 为什么压不下去 |
|---|---|---|
| 等说完 | 380 ms | 音频不能在它存在之前被送走。M18 的教训正是端点检测切太早会**把句子切断**（「空调调低一点」不响应的根因），往回调是拿正确性换延迟 |
| 上传 74 KB | 100 ms | 边录边传最多省掉这 100 ms |
| SenseVoice | **780 ms** | 见下 |
| miIO 控灯 | 300–500 ms | 两条路都付，不是差异来源 |

SenseVoice 的成本结构很反直觉。拿不同长度的真语音量了一遍：

| 音频长度 | STT 耗时 | rtf |
|---|---|---|
| 0.95s「开灯」 | 748 ms | 0.79 |
| 1.80s「打开台灯」 | 780 ms | 0.43 |
| 2.48s「播放王菲的红豆」 | 806 ms | 0.32 |
| 2.88s「空调调到二十六度」 | 827 ms | 0.29 |

线性拟合：**固定 ≈ 724 ms + 每秒音频 ≈ 22 ms**。

> 也就是说 SenseVoice 几乎**与句子长短无关**。
> "说短一点会不会快些" —— 不会。rtf 这个指标在这里是误导的：
> 它随音频变长而变好，只是因为分母变大了。

也测了几个常见猜想，都不成立：

- **闲置惩罚**：没有。闲 5/15/30 秒后首次调用 699–754 ms，和连续调用一样。
- **噪声代价**：没有。SNR 0 dB 时 789 ms（听成「他有证」，识别错了但不慢）。
- **静音补白**：没有。前后各补 1 秒静音只多 20 ms。

→ **兜底路径的地板 ≈ 380 + 100 + 780 + 300 ≈ 1.56 s。**
把网络和上传优化到 0 也回不到 400 ms。唯一能让「开灯」变快的办法，
是让它**不走这条路**。

### 6.2 板子日志里那个 1635 ms

```
耗时 2376 ms = 上传 100(74.6KB) + 服务端 2162[STT 1635 控灯 527] + 网络 114
```

1635 ms 是基线的两倍，值得单独解释一下，否则会照着一个错的数字去优化：

- **LLM 争抢**：实测 qwen3:4b 同时在生成时，STT 从 804 → 1099 ms（+295 ms，1.4×）。
  但这条日志对应的是「放大音量」，规则层 conf=1.0 直接命中，**LLM 根本没被调用**。
- **真正的原因**：那一条是在回归测试里抓的，而测试脚本自己正在合成并播放音频。
  争抢的是测试台，不是生产负载。

> 记下来免得下次照着 1635 优化：日常的数字是 ~800 ms。

### 6.3 能做的事，按杠杆排序

**1. 把高频短词换进词表**（400 ms vs 1600 ms，6×）

12 条里 5 条是模式词（全亮/黄光/冷光/阅读/夜灯），2 条是长形式（打开台灯/关闭台灯）——
**词表被花在长尾说法上，而最高频最短的那两句掉进了兜底**。

但 §4.1.18 的零和约束是硬的（23 条时 2/7），所以只能**替换不能追加**。
而且有个具体风险：`开灯 kai deng` 和 `打开台灯 da kai tai deng` 后三个音高度重叠，
正是 §4.1.18 说的"前两个音节要分得开"会踩的坑 —— 两条可能互相稀释。
必须跑 `tools/mn_regress.sh`，而且要**同时验新词和被它影响的老词**。

**2. 让 STT 不和别人抢 CPU**（省 ~300 ms，且对所有兜底命令生效）

这一条没有零和代价。qwen3:4b 常驻 3 GB、`keep_alive 15m`，
而规则层覆盖九成、LLM 很少真被调用。可以缩短 keep_alive，
或给 STT 那次调用限定线程数 / 让它和 Ollama 错开核。

**3. 边录边传**（省 ~100 ms）—— 杠杆最小，复杂度不低，不该先做。

**不该做的：缩短 VAD 尾判。** M18 已经踩过：切太早会把句子切断。

### 6.4 词表替换实验，以及它挖出来的测试台缺陷（2026-09-16）

想把高频短词「开灯 / 关灯」换进板上词表。用 `MULTINET_TABLE="开灯,关灯"` 让服务端
发一张两条词的表（板子每 60s 轮询 `/commands/version`，**不用烧固件**），
带一条判别词「打开台灯」——它不在表里，如果还能被认出来就说明旧表没换掉。

跑了两轮，第二轮换了激励源：

| 激励源 | 开灯 | 关灯 | 判别词「打开台灯」 |
|---|---|---|---|
| Kokoro（项目一直在用的） | 2/3，prob 0.16–0.18 | 3/3，0.19–0.50 | 0/3 未触发 |
| macOS `say -v Tingting` | **3/3，0.17–0.46** | **3/3，0.40–0.65** | 3/3 全部**听成「开灯」** |

换激励源之后置信度从 0.16 跳到 0.46。差别不在板子，在**送进去的声音本身**。

#### 用户听出来的：合成音的声调是错的

用户听着回归在放音，说了两次：「开1灯1，你说成了开4灯4」、「又是灯1kai4了」。
量基频（一声应为平调，四声为高降）：

| 合成方式 | 开 | 灯 | 应有 |
|---|---|---|---|
| Kokoro 孤立「开灯」 | −5.5 | **−10.1** | ≈0 ≈0 |
| Tingting 孤立「开灯」 | **−5.3** | +0.0 | ≈0 ≈0 |
| Kokoro 载体句「开灯以后再说」 | −2.4 | −1.5 | ≈0 ≈0 |

单字对照验证了这把尺子本身：Kokoro 念「妈」(一声) 测出 −2.9、「骂」(四声) 第一段反而 **+3.4**，
完全反了；Tingting 是 +1.3 / −6.1，正确。

g2p **没错**（misaki 给的是 `ㄎㄞ1ㄉㄥ1`，两个一声）。错的是声学模型：
**两音节词太短，整个词都落在陈述句语调的斜坡上**，词汇声调被语调盖掉。
两个引擎都有这个毛病，只是位置不同 —— Kokoro 压末字，Tingting 压首字。
载体句能把失真从 −10.1 压到 −1.5，证明症结是"处于句尾"，不是模型不会念一声。

#### 波及面：一条写进词表注释的"规律"可能是它造出来的

```mermaid
flowchart LR
  A["Kokoro 合成<br/>短词激励"] --> B["声调失真<br/>末字 −10 半音"]
  B --> C["短词在回归里<br/>大量失败"]
  C --> D["总结出规律：<br/>「每条至少 3 个音节」"]
  D --> E["后续选词绕开短词"]
  E -.->|"从未被质疑"| C
  classDef bad fill:#fee,stroke:#c33,stroke-width:2px
  class B,D bad
```

| 历史被毙的词 | 音节 | 末字调 | 实测末字 | 当年结论 |
|---|---|---|---|---|
| 最亮 | 2 | 4 | −12.1 | 0/9 静默 |
| 最暗 | 2 | 4 | −10.4 | 0/9 静默 |
| 下一首歌 | 4 | 1 | −3.4 | 0/3 |
| 关掉收音机 | 5 | 1 | **+0.4** | 正式表最稳 0.45 |

被毙的全是短词，而短词正是激励最失真的地方。
§4.1.18 那条「每条至少 3 个音节」的规律，**有相当一部分可能是测试台的产物**。

> README §4.1.18 自己写过：「测试工具出错比被测物出错更危险，因为错的是
> **你用来判断对错的那把尺子**。」那一节抓出了三个这样的 bug。这是第四个，
> 而且它不是让某一轮结果错，是让**一条被写进注释、指导了后续所有选词的"规律"**错。
> 抓出它的不是日志，是有人在旁边听见了。和 §4.1.18 末尾那条一模一样：
> 开着麦克风的系统，日志里没有"房间里实际听起来怎么样"这一维。

#### 结论：实验暂停，先修尺子

两条词本身**不是静默失效那一类**，它们能用。但有两个障碍：

1. **尺子还是坏的。** 目前没有一个能正确念出两音节词的激励源。
   用已知失真的尺子去量"两条短词和长词的干扰"，量出来的数字不足以支撑
   "要不要牺牲两条模式词"这种决定。
2. **`kai deng` 会塌到 `da kai tai deng` 上。** 判别词三轮三次被听成「开灯」——
   这不是偶然。两条并存时「打开台灯」很可能被吃掉，而它是现在唯一稳定的开灯命令。

修尺子有三条路：载体句合成后按静音切出目标词（改动只在 `speak.sh`，实测能压到 −1.5）、
录真人音、或者接受失真只做相对比较。

#### 实验期间的副作用（记录在案）

实验跑的时候用户正在正常使用系统，说「打开台灯」得到的回复是「灯已关」。
原因：**MultiNet 不会说"我不认识"** —— 它总在已注册的词里挑最近的一条，
而当时板上只有 `{开灯, 关灯}`。`DRY_RUN=1` 挡住了真实执行，所以灯没动，但话说错了。

> 判别词被误匹配是**设计好的**，它正是用来证明旧表已经换掉。
> 没预料到的是有人会在实验窗口里正常用这套系统 —— 换表这类操作
> 会让设备在一段时间内**系统性地听错话**，开跑之前得先打招呼。


### 6.5 「被无视了」的真相：听了一半就发车（2026-09-16 已修）

用户报「关灯」「播放爱的初体验」「播放张震岳的爱的初体验」三条都没反应。
抓串口才看到，那不是没反应，是**两个不同的故障**。

```
故障 A  听到唤醒词（第 1 个词）
        唤醒后没人说话，回到待唤醒          ← 命令根本没离开板子

故障 B  不在命令词表里，送 2.76s（掐头 3.24s 掐尾 0.00s）…（等到超时）
        服务端听成「你好，小智播放爱的。」-> 放爱的歌 (已执行)
                                            ↑ 真的放了一首叫《爱的》的歌
```

**故障 B 的机理**（`sr.c`）：`CONFIG_S31_COMMAND_TIMEOUT_MS = 6000` 的时限是
**从唤醒词那一刻**算起的，于是"唤醒词和命令词之间停顿多久"直接从命令的预算里扣。
那次开口前就烧掉 3.74s，留给一句 3 秒的话只剩 2.3 秒，撞上硬切。

而且有三层巧合让它一直没被发现：

1. `REC_MAX_SAMPLES` 正好也是 6 秒 —— 窗口到点，缓冲区也正好写满，
   连个溢出迹象都没有。
2. MultiNet 是用**同一个** 6000ms 创建的（`s_mn->create(mn_name, CONFIG_S31_COMMAND_TIMEOUT_MS)`），
   所以两个超时源同时到期，只堵一个没用。
3. 截断后的半句话**照样能解析成一条合法指令**，于是系统一声不吭地做错了事 ——
   没有报错、没有反问，只有一首放错的歌。

> 签名就写在日志里：**`掐尾 0.00s` + `等到超时`**。
> 这对组合的意思是"VAD 从没判出说完，是被时限硬切的"，它一直在打，
> 只是没人把它当成故障。和 §6.4 那个声调失真一样 ——
> **不是没有证据，是证据没被当成证据。**

```mermaid
flowchart LR
  subgraph OLD["改之前"]
    A1["唤醒"] --> A2["固定 6s 时限<br/>（从唤醒词算起）"]
    A2 --> A3{"6s 到了"}
    A3 --> A4["不管你说没说完<br/>直接发车"]
    A4 --> A5["半句话 -> 合法指令<br/>放错的歌"]
  end
  subgraph NEW["改之后"]
    B1["唤醒"] --> B2["6s 时限"]
    B2 --> B3{"还在说话？"}
    B3 -- 是 --> B4["时限 += 2s<br/>上限 12s"] --> B3
    B3 -- 否 --> B5["VAD 判定说完<br/>正常发车"]
  end
  classDef bad fill:#fee,stroke:#c33,stroke-width:2px
  classDef good fill:#efe,stroke:#3a3
  class A4,A5 bad
  class B4,B5 good
```

改了三处，缺一不可：

| 改动 | 为什么 |
|---|---|
| `REC_MAX_SAMPLES` 6s → **12s** | 窗口延长了缓冲区不跟上，多录的会被 `rec_len + n > REC_MAX_SAMPLES` 悄悄丢掉 |
| 说话期间 `listen_until += 2s`（`SPEECH_GRACE_MS`，硬上限 `MAX_LISTEN_MS` 12s） | 时限不该从唤醒词算，该从"还在不在说话"算 |
| `ESP_MN_STATE_TIMEOUT` 只记录、不收尾（`mn_expired`） | MultiNet 用同一个 6s，只堵 `listen_until` 它照样切 |

**刻意没做的**：MN 超时后不调 `s_mn->clean()`。半句话重启 MultiNet 会让它拿残句
去匹配，可能撞出一条**假命令** —— 那比截断更糟。兜底路径不需要它。

行为等价性（改端点检测最容易改出回归，所以逐条对过）：

- `still_speaking = spoke && !spoke_and_stopped`，和原有判据严格互补 ——
  **正常收尾路径一个字节都没变**
- `spoke == false`（没人说话）时 `still_speaking` 恒假 → 6 秒照旧超时
- 只有"你还在说、而时限到了"这一种情况走新路


### 6.6 音量：一个旋钮，两个写者（2026-09-16 已修）

用户报「音量调到1」被无视，并提醒："音量调低后，不要被之前那个指令结束后恢复高音量的逻辑覆盖。"
查下来是**两个独立的 bug**，而第二个比提醒的还要严重。

#### 先把状态理清楚：到底有几个音量

| 增益 | 可读 | 可精确设定 | 谁在写 |
|---|---|---|---|
| macOS 系统音量 | ✅ osascript | ✅ 实测 1/5/12/13/35/40/55 设进去读回来一个不差 | 压音量机制、用户指令 |
| Tivoli 自己的音量（红外 VOL±） | ❌ 开环无回执 | ❌ 相对档，按几下全靠记账 | 红外、实体遥控器 |

**但 AirPlay 挂着的时候这两个是同一个旋钮**：实测发 3 下红外 VOL−，macOS 系统音量
跟着从 18 掉到 12（`tivoli.py` 的 `volume_step` 注释里记着）。

由此推出两条：

1. **绝对音量只能走网络。** "设成 10" 是个绝对目标，而红外链路读不回当前位置、
   也没有回执 —— 在那条链上这句话根本无法表达，硬按几下只是在赌。
   这是 §8 那条底线的直接推论。
2. **压音量和用户指令写的是同一个格子**，于是天然是"丢失更新"的形状。

#### Bug 1：「音量调到1」被台灯抢走

```
音量调到1  ->  light.brightness  rule=set_pct   ← 去调了台灯亮度
```

`_PCT_RE` 看到"调到+数字"就认领，完全无视句子里的"音量"。用户看到的是
"音量没反应"，报上来是"被无视了" —— 而**日志里一切正常，执行=True**。

和 README §4.1.24 的「空调调到26」被灯抢走是同一类，而那次的修法（"不带「度」时
**必须有「空调」两个字**兜底"）就写在 `ac_set_temp` 的注释里，音量这边一直缺这本护照。

修法：新增 `tivoli.set_volume` 规则，排在灯的 `set_pct` **之前**，且数字前
必须有显式锚点（调到/设成/到/百分之）。

> 锚点这一条是第二次踩：第一版只要求"音量 + 数字"，于是
> **「声音大一点」里的「一」被抓成 pct=1**。`_PCT_RE` 的注释里早就警告过
> 同一个陷阱（「调到一半」被抓成「一」），我还是原样踩了一遍。

#### Bug 2：唤醒词本身就会把音量抬高

实测复现（不是推测）：

| 场景 | 改前 | 改后 |
|---|---|---|
| A. 音量设成 1，然后**只说一句唤醒词**（不下任何命令） | 抬到 **13** | 保持 **1** |
| B. 压制窗口内说「音量调到1」 | 被覆盖回 **40** | 保持 **1** |
| C. 没人插手，正常压制（回归对照） | 恢复到 40 | 恢复到 40 |

A 的成因：`_duck_from = max(cur, level + 1)` = max(1, 13) = 13。那个 `max` 本意是
防止把"压制档位"误记成原始值，但用户真想要低于压制档的音量时它反咬一口。
而且 `if cur > level` 已经让那次压制**一个分贝都没压**，恢复任务却照排不误。

#### 第一性原理：`_duck_from` 是预测，不是事实

```mermaid
sequenceDiagram
  participant U as 用户
  participant D as 压音量机制
  participant V as 系统音量（唯一的格子）

  Note over D,V: 压制的契约：「我把它从 X 压到 12，之后放回 X」
  U->>V: 唤醒词
  D->>V: 读到 X=40，存 _duck_from=40
  D->>V: 写 12
  U->>V: 「音量调到1」 → 写 1
  Note over D: 8 秒后……
  rect rgb(255,238,238)
    Note over D,V: 改之前：无条件写回 40 —— 用户的 1 没了
  end
  rect rgb(238,255,238)
    Note over D,V: 改之后：先读，发现是 1 而不是我压的 12<br/>→ 预测作废，放弃恢复
  end
```

> `_duck_from` **不是一个关于世界的事实，而是一个预测**：
> "这 8 秒里不会有别人写系统音量"。
> 用户说了「音量调到1」，或者有人伸手拧了实体旋钮，这个预测就作废了。
> **拿一个作废的预测去覆盖一个更新的、真实的用户意图，就是纯粹的丢失更新。**

所以恢复改成 **compare-and-swap**：只有当前值还是我压下去的那个档位时，
才说明这段时间没人动过，才轮得到我放回去。

为什么不用"用户改音量时显式取消恢复任务"：那能治 B，治不了**绕过我们的写入** ——
而 AirPlay 会把设备音量同步回系统音量，实体遥控器那条路是真实存在的。
比对当前值则两种都管。容差取 1（实测 set/get 精确，留 1 只防四舍五入）。

另外：压制时如果当前音量本来就不高于档位，**不排恢复任务**（没压过就没得恢复）。

回归在 `test_control_safety.py::VolumeOwnershipTests`（含"没人插手时必须照旧恢复"
这条对照，免得为了修前两条把压音量本身废掉）和 `::VolumeIntentTests`。


### 6.7 一个空转的容器吃掉两核，十天（2026-09-17 已修）

用户报「播放我喜欢的音乐」→「这个我还不会」。查下来牵出两件事，
第二件比第一件严重得多。

#### 第一件：这句话意图层本来就认得

```
播放我喜欢的音乐 -> music.play_favorites ✓
放我喜欢的歌     -> music.play_favorites ✓
来点我喜欢的     -> music.play_favorites ✓
```

失败的是 **STT**：服务端收到的文本是 **「我阳门去」**。
三层意图共享同一个输入，输入毁了三层一起毁 —— LLM 拿到的也是这四个字，
给它再多时间也翻译不出原话。

> 这一条容易被归错类。「规则不够用，该让 LLM 上」是一个看起来合理、
> 而且永远说得通的解释，但这里规则**命中了**，只是没机会看到真正的句子。
> 判断一次失败属于哪一层，要看**那一层的输入是什么**，不是看输出多难看。

顺带排除两个假设，都是实测推翻的：

- **"高负载导致识别质量下降"**（我自己先提出的）：同负载下喂干净音频
  **4/4 全对**（774–1041ms）。负载只让它慢，不让它错。
- **"句子被截断"**：截断只让句子变短，不会变成乱码 ——
  完整 2.65s → 「播放我喜欢的音乐」；掐掉开头 0.7s → 「我喜欢的音乐」；
  只留前 1.5s → 「播放我喜欢的」。全都通顺。

乱码来自**低信噪比的现场音频**，问题在麦克风那一端。

#### 第二件：esphome 容器空转 204%，十天

```
CPU=204.80%   MEM=734.2MiB   Up 10 days
```

| 观察 | 值 |
|---|---|
| 同时段 HA | CPU=0.01% |
| 编译产物（`esphome/pio`、`cache`） | **10 天零改动** —— 没在编译 |
| 容器日志 | 只有 HA 每 30s 一次健康检查，每次 2ms |
| 代理 :7897 | 开着 —— 不是拉不通在重试 |
| 重启后 | **0.26% / 25MB** |

十核机器被拿走两核，STT 从 800ms 退化到 2144ms、最坏 **10616ms**，
而**日志里一切正常**。

**成因（有证据支持，但未证实）**：`.esphome/` 里留着两个红外学习会话日志
（`tivoli-ir-learning.log` / `ceiling-light-learning.log`，都停在 9/10 22:16），
而 `tivoli_ir.yaml` 的 `remote_receiver` 配的是 `dump: all` —— 那块板子会把
看到的每个红外脉冲都吐进日志。学习会话结束后若 dashboard 的读取线程没退干净，
就会在已 EOF 的管道上空转：`read()` 立刻返回 0，循环不阻塞，一个线程吃满一核。
204% ≈ 两个这样的线程，和两个 log 文件对得上；734MB → 25MB 的落差也符合
"有缓冲区一直在长、没人消费"。

> 容器已重启，直接证据没了。要证实得在**下次发生时**进容器抓
> `py-spy dump` 或 `/proc/<pid>/stack`。记下推理链，是为了下次不用从头查。

#### 真正该改的是结构

根因是哪一行不重要 —— 重要的是**它能悄无声息吃掉两核十天，
而语音链路就在同一台机器上**：

```
RestartPolicy=unless-stopped    ← 崩了会重启
CPUs=0   Memory=0               ← 但空转不算崩，没有任何上限
```

```mermaid
flowchart TB
  subgraph M["同一台 10 核 Mac"]
    subgraph VM["Docker VM（改之前无上限）"]
      E["esphome<br/>空转 204%"]
      H["home assistant<br/>0.01%"]
    end
    S["SenseVoice STT<br/>预算 ~800ms"]
    K["Kokoro TTS"]
    Q["Qwen3-4B<br/>硬超时 2.6s"]
  end
  E -- "抢走 2 核" --> S
  S -- "800ms 变 10616ms" --> BAD["「这个我还不会」<br/>而日志里一切正常"]
  classDef bad fill:#fee,stroke:#c33,stroke-width:2px
  class E,BAD bad
```

`docker-compose.yml` 加硬上限：esphome `cpus 2.0 / 2g`、home_assistant `cpus 1.5 / 1.5g`。
实测生效（`NanoCpus` 从 0 变成 2000000000 / 1500000000），两个容器功能正常，
HA :8123 和 ESPHome :6052 都回 200。

**为什么是 `cpus` 而不是 `cpu_shares`**：`cpu_shares` 调的是 Docker VM **内部**
cgroup 之间的相对权重，而我们要挡的是整个 VM 向 macOS 要 CPU ——
争用发生在 VM 和原生 Python 进程之间，**在 VM 外面**，`cpu_shares` 够不着。

代价：esphome 编译固件时会慢一些。这笔交易划算 ——
编译是低频的、你在旁边看着的；空转是高频的、没人看得见的。

修复后 **STT 676ms**，比 §6.1 记的 800ms 基线还快。

#### 顺带回答"要不要换 Mac mini M6"

| 症状 | 换机器管用吗 |
|---|---|
| 听成「我阳门去」 | ❌ 音频信噪比问题。同机同负载喂干净音频 4/4 全对，CPU 再快也改不了送进去的声音 |
| STT 800ms 变 10616ms | ⚠️ 治标。真因是容器空转，换 12 核还是被吃掉 2 核，只是剩得多一点 |

> 现在换机器买不到正确性，只买到余量 —— 而余量正被一个空转容器白白吃掉。
> 要提升识别率，方向在麦克风那一端：README 的 R2 条目
> （"音乐响着的时候板子还叫不叫得醒"，`wake_bench.py` 那条曲线）正是为此准备的，一直没扫。


---

## 7. 这套设计为什么不让 LLM 当入口

| | 规则层 | 本地 LLM |
|---|---|---|
| 延迟 | 微秒 | 1.16–1.71s 典型，2.6s 硬超时 |
| 确定性 | 同一句话永远同一个结果 | 今天这么解，明天那么解 |
| 断网 | 照常 | 照常（本地跑），但吃 3G 内存 |
| 覆盖 | 家里的指令空间九成 | 剩下那一成 |
| 编设备 | 不可能 | 会——所以有 `_ALLOWED` 白名单 |

内存账（`llm.py` 开头记的）：
SenseVoice ≈1.2G + Kokoro ≈0.5G + Qwen3-4B ≈3G + HA 容器 ≈1G + 系统 ≈4G ≈ **10G / 16G**。
选 4B 不选 7B 是被内存逼的。

> 有一条经验值得单独记：曾经写过一版"拿不准就答 none"的提示词，实测**反了**——
> 模型明明听懂了（对「这首太吵了」它回的 reply 是"太吵了，调低点？"），
> 却照着提示词答 `domain=none`，五句里五句作废。
> 编造不存在的动作有白名单挡着，那才是真风险；
> **让一个答得对的模型闭嘴不是安全，只是失败。**

---

## 8. 不变的那条底线

> **没有回执的链路，不能把"命令发出去了"当成"事情做成了"。**

| 设备 | 有没有回执 | 后果 |
|---|---|---|
| 米家台灯 | 有（miIO 局域网直连） | `_note_effect()` 敢记世界状态 |
| 空调 | **有**（Coolix 是绝对状态帧，且 ESPHome 在收） | 敢记；连说「太热」「太冷」用自己的 `_last_set` 当基准 |
| Tivoli 红外 | **没有** | `press()` 返回 True 只代表发出去了；`power_off()` 永远不报成功 |
| AirPlay | 有（RAOP 链路状态） | 但**地址本身可能是错的**，见下 |

M20 就是在还这笔账：`reachable()` 失败只代表**不知道**，不代表设备关着。
详见 `README.md` §4.1.25。

### 同一笔账，`player.py` 里还欠着（2026-09-16 补上）

用户报「播放王菲的红豆」，系统答**「音响不在线，先打开它」**，而音响就在旁边开着。

查下来不是音响的问题，是**地址过期**，而且它伪装得很好：

| 地址 | ping | :7000 |
|---|---|---|
| `192.168.0.107`（`.env` 里写的） | ✅ 通 | ❌ ConnectionRefused |
| `192.168.0.108`（Tivoli 实际在的地方） | ✅ | ✅ 8 ms |
| `audiocast.local`（mDNS 名） | — | ✅ 27 ms |

DHCP 把 Tivoli 挪了一格，而旧地址上住进了别的设备 —— **ping 通、7000 拒绝**，
于是"地址过期了"被报成了"音响没开机"。mDNS 里 Tivoli 一直在正常广播
`_airplay._tcp`，这也正是用户在系统「声音」面板里看得见它的原因：
那个列表来自 Bonjour 发现，和我们写死的那个 IP 没有半点关系。

两处都改了：

1. `AIRPLAY_HOST` 改成 `audiocast.local` —— 不随租约漂。
   `discovery.py` 开头早就为板子写过同一段道理（"板子上写死 IP 是在用最贵的
   操作解决最廉价的问题"），只是 AirPlay 这头一直漏着。
2. `player.py:_ensure_link()` 那句回话不再断言。
   `reachable()` 失败只说明"在我被告知的那个地址上没找到它"。

> 骗人的不是探测，是那句把"不知道"说成"我知道"的回话。
