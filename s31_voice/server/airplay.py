"""把音频送到 Tivoli：走 **macOS 自己的 AirPlay 栈**，不走 pyatv。

为什么不是 pyatv（2026-09-11 实测得出的结论，别再走一遍）：
这台机器是严格的 AirPlay 2 接收端（`/info` 里 `PTPInfo: Apple Airplay 2.0.10 SDK PTP`）。
握手能走到一半——transient 配对成功、密钥都推导出来了——然后 `SETUP` 石沉大海。
把 pyatv 完全绕开、手写裸 RTSP 又验了一遍：`ANNOUNCE` 200，`SETUP` 403。
原因是 pyatv 在 RAOP 这条路上**从不给 RTSP 开加密**（整个包里只有 MRP 调
`enable_encryption`），而这台接收端配对完就要求加密通道，明文 SETUP 它不报错、直接丢。
这是库的能力缺口，配置绕不过去。

而 macOS 自己推没有任何问题（实测：系统 AirPlay 不要密码、出声正常）。
所以这里退一步，把 Apple 的实现当成传输层用：系统输出指着 AirPlay，
我们只管往系统默认输出上放音频。

代价说清楚：
  1. **切走 AirPlay 会拆掉连接**，而且 `SwitchAudioSource` 没法再切回去
     （macOS 不给命令行选 AirPlay 目标的口子）。所以这里**永不主动切走**。
  2. Tivoli 一断电链路就废，而且**用红外把源从 WiFi 切到 FM 也会断**，
     切回 WiFi 之后 macOS 也不会自动重连（实测等 30 秒不回来）。
     所以 anchor() 会在链路断掉时自动调 tools/airplay_relink.sh 去点控制中心。
     IR 的 POWER 是**硬断电**不是待机（WiFi 一起掉，重新入网 28 秒），默认不碰。
  3. 拿不到"歌名显示在 Tivoli 屏上"那个功能 —— afplay 不送元数据。
  4. 系统音量就是 Tivoli 的音量，两者是同一个旋钮。

日常回话**不走这里**，走 S31 板子自己的喇叭（voice.c，CONFIG_S31_HAS_SPEAKER）——
那条路不依赖 Tivoli 开着，也不依赖这条 AirPlay 链路活着。这里只管放音乐。
"""
from __future__ import annotations

import asyncio
import logging
import os
import shutil
import tempfile
import time

_LOG = logging.getLogger("airplay")

# CoreAudio 里那个虚拟输出设备的名字。注意它的 uid 每次重连都会变
# （实测 …-941178834869875-Audio -> …-942337989194583-Audio），只有名字是稳定的。
_DEVICE_NAME = os.environ.get("AIRPLAY_OUTPUT_NAME", "AirPlay")
_SWITCH = shutil.which("SwitchAudioSource") or "/opt/homebrew/bin/SwitchAudioSource"
# 链路断了之后重新挂上去的脚本。CoreAudio 那层没有命令行能选 AirPlay 目标
# （断开之后这个设备干脆就不在 SwitchAudioSource 的列表里），只剩 UI 脚本这一条路。
_RELINK = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "tools", "airplay_relink.sh")
# 设备掉电之后这个虚拟设备还会赖在列表里 30 秒以上（实测），
# 所以"在不在列表里"**不能**当存活判据 —— 存活判据是 ping/TCP。
_PROBE_PORT = 7000


class AirPlayError(RuntimeError):
    pass


async def _run(*args: str, timeout: float = 8.0) -> tuple[int, str]:
    proc = await asyncio.create_subprocess_exec(
        *args, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), timeout=timeout)
    except asyncio.TimeoutError:
        proc.kill()
        raise
    return proc.returncode or 0, (out or b"").decode(errors="replace").strip()


class AirPlay:
    def __init__(self, host: str | None = None, name: str | None = None,
                 password: str | None = None, idle_close_s: float = 20.0) -> None:
        # password / idle_close_s 保留只为不改调用方签名；macOS 这条路两者都用不上。
        self.host = host or None
        self.name = name or None
        self._last_ok: float | None = None
        self._play: asyncio.subprocess.Process | None = None
        self._duck_task: asyncio.Task | None = None
        self._duck_from: float | None = None
        # 我**实际压下去**的那个档位。恢复前要拿它和当前值比一比：
        # 对不上就说明压制期间有别人写过音量，那时候不能恢复。见 _unduck_after。
        self._ducked_to: float | None = None
        # 压制前的音量**落盘**。见 duck() 里那段说明：它守护的是一个
        # 活得比进程久的副作用（系统音量），只存在内存里是不够的。
        self._duck_file = os.path.join(tempfile.gettempdir(), "s31_duck_from")

    # ---------- 探活 ----------

    async def reachable(self, timeout: float = 1.5) -> bool:
        """设备在不在电、在不在网。**这是唯一可靠的存活信号。**

        用 TCP 连 :7000 而不是 ping：ping 通只说明网络栈起来了，
        而 :7000 连得上说明 AirPlay 服务也起来了，更接近我们真正关心的东西。
        实测断电后 ping 立刻就断，开机到重新入网要 28 秒。
        """
        if not self.host:
            return False
        try:
            fut = asyncio.open_connection(self.host, _PROBE_PORT)
            reader, writer = await asyncio.wait_for(fut, timeout=timeout)
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:  # noqa: BLE001
                pass
            return True
        except Exception:  # noqa: BLE001
            return False

    async def linked(self) -> bool:
        """macOS 这头的 AirPlay 链路还在不在（CoreAudio 里有没有这个输出设备）。

        注意它**不等于设备活着** —— 设备断电后这个虚拟设备还会留 30 秒以上。
        它的真正用途是反过来：这个不在，就一定推不了音频，而且要人手重连。
        """
        try:
            code, out = await _run(_SWITCH, "-a", "-t", "output", timeout=5)
        except Exception:  # noqa: BLE001
            return False
        return code == 0 and _DEVICE_NAME in out.splitlines()

    async def relink(self, timeout: float = 60.0) -> bool:
        """把系统输出重新挂回 Tivoli。靠 UI 脚本点控制中心，慢（几秒到十几秒）。

        慢是因为设备名**没有暴露给辅助功能**（macOS 26.2 实测：AXTitle/AXDescription
        全空），只能一个个点、点完回头问 SwitchAudioSource 当前输出变成了谁。
        所以这个不能放在快路径上 —— 它是"要放音乐了，先把路铺好"那一步。
        """
        try:
            code, out = await _run("/bin/bash", _RELINK, timeout=timeout)
        except asyncio.TimeoutError:
            _LOG.warning("重挂 AirPlay 超时（%.0fs）", timeout)
            return False
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("重挂 AirPlay 出错：%s", exc)
            return False
        if code != 0:
            _LOG.warning("重挂 AirPlay 失败：%s", out.splitlines()[-1] if out else code)
            return False
        _LOG.info("AirPlay 已重挂：%s", out.splitlines()[-1] if out else "")
        return True

    async def current_output(self) -> str:
        try:
            _, out = await _run(_SWITCH, "-c", "-t", "output", timeout=5)
            return out
        except Exception:  # noqa: BLE001
            return ""

    # ---------- 原语 ----------

    async def anchor(self, timeout: float = 8.0) -> bool:
        """确认这条链路现在能把声音送到 Tivoli。

        和最初的设计相比这里**降级**了：原来的锚定是"推一段静音、成功即证明
        设备开着且停在 WiFi 源上"。macOS 这条路推流是单向的，afplay 不论
        设备在不在都返回 0，所以拿不到那个回执。现在的判据是两条的合取：
            设备 TCP 可达（在电、在网）  且  macOS 这头链路还在
        源在哪依然读不回来 —— 但推音频这个动作**本身**会把设备拽到 AirPlay 源，
        所以"锚定"作为**执行器**仍然成立，只是作为**传感器**弱了一档。
        """
        t0 = time.perf_counter()
        alive = await self.reachable()
        if not alive:
            _LOG.info("锚定失败（%.0fms）：设备 TCP 不可达 —— 断电了或掉网了",
                      (time.perf_counter() - t0) * 1000)
            return False
        if not await self.linked():
            # 链路断了。这在正常使用里很常见 —— 只要用红外切过一次源（去听 FM），
            # 回来就是断的。所以这里不报错，直接去修：点控制中心把它挂回来。
            _LOG.info("AirPlay 链路断了，自动重挂……")
            if not await self.relink():
                _LOG.warning("重挂失败 —— 多半是辅助功能权限没给，"
                             "见 tools/airplay_relink.sh 开头的说明")
                return False
        self._last_ok = time.time()
        _LOG.info("锚定成功（%.0fms）—— 设备在线且链路通", (time.perf_counter() - t0) * 1000)
        return True

    @property
    def last_anchor(self) -> float | None:
        return self._last_ok

    # ---------- 播放 ----------

    async def stream(self, source, metadata=None, volume: float | None = None) -> None:
        """放一个本地文件，**阻塞到放完**。metadata 在这条路上无效（见文件头）。

        用 afplay 而不是自己灌 CoreAudio：afplay 就是系统默认输出，
        而系统默认输出已经指着 Tivoli。少一层就少一层会坏的东西。
        """
        if not isinstance(source, str):
            raise AirPlayError("macOS 这条路只能放本地文件，给不了内存流")
        if volume is not None:
            await self.set_volume(volume)
        await self.stop()
        self._play = await asyncio.create_subprocess_exec(
            "/usr/bin/afplay", source,
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
        try:
            await self._play.wait()
        except asyncio.CancelledError:
            await self.stop()
            raise
        finally:
            self._last_ok = time.time()
            self._play = None

    async def stop(self) -> None:
        """停掉正在放的那一首。RAOP 没有"暂停"，停就是不再往那边送。"""
        proc, self._play = self._play, None
        if proc is not None and proc.returncode is None:
            proc.kill()
            try:
                await proc.wait()
            except Exception:  # noqa: BLE001
                pass

    async def close(self) -> None:
        await self.stop()

    # ---------- 音量 ----------
    #
    # 系统音量就是 Tivoli 的音量 —— AirPlay 输出时两者是同一个旋钮。
    # 这比红外的 VOL± 好：红外是相对的、开环的，按几次全靠记账，
    # 而记账一定会跟实体遥控器的操作对不上。这里给的是绝对值。

    async def duck(self, level: float, seconds: float) -> None:
        """把音量压下去一段时间，然后自动恢复。**连续对话能不能成立全靠它。**

        板子的识别在 SNR 掉到 5dB 时就崩（实测唤醒 1/6），而音乐一响就是那个量级 ——
        今晚现场看到过：服务端收到的是「🎼我唱唱给的算」，`🎼` 是 SenseVoice 的
        "这段是音乐"标记，也就是说在麦克风那一端就已经输了，规则层再聪明也没用。

        而音乐是**我们自己推的**，音量归我们管 —— 这是唯一能正面打赢那条悬崖的办法。
        别人做不到，是因为他们不掌握声源。

        重入是安全的：第二次 duck 会取消上一次的恢复任务，并沿用**最早**那个原始音量，
        否则连续几轮对话会把"原始音量"一路记成压低后的值，最后再也恢复不回去。
        """
        # 输出设备不是 AirPlay 的时候什么都不做。
        # 压音量压的是**系统音量** —— 也就是 Mac 上正在响的一切，不只是我们
        # 推给 Tivoli 的那份音乐。只有当输出确实指着 Tivoli 时，
        # "屋里的噪音"和"我们的系统音量"才是同一个东西，压它才既有效又正当。
        #
        # 这条是踩出来的：声学回归脚本从**内置喇叭**放测试音给板子听，
        # 板子一听到唤醒词就 POST /wake，服务端把系统音量从 85 压到 12，
        # 于是紧随其后的命令词在音量 12 下播出去 —— 整轮回归 0/10 全灭，
        # 而且看起来像是"命令词全部静默失效"这种最难查的故障。
        if not await self.linked():
            return
        cur = await self.volume()
        if cur is None:
            return
        already_ducking = self._duck_task is not None and not self._duck_task.done()
        if already_ducking:
            # 重入：沿用**最早**那个原始音量，只把恢复的计时器往后推。
            self._duck_task.cancel()
        else:
            if cur <= level:
                # 没在压制中，而且音量本来就不高于档位 ——
                # 没有什么可压的，也就没有什么可恢复的。**直接不参与**。
                #
                # 原来这里照样排一个恢复任务，于是用户把音量设到 1 之后，
                # 只要说一句唤醒词（哪怕不下任何命令），8 秒后音量就被抬到 13：
                # _duck_from 取 max(cur, level+1) = 13，那个 max 本意是防止把
                # "压制档位"误记成原始值，但用户真想要低于压制档的音量时它反咬一口。
                #
                # **这个早退必须待在 else 分支里**。第一版写在外面，于是压制窗口内
                # 的第二次唤醒会：先 cancel 掉恢复任务，再因为"音量已经是 12 了"
                # 直接 return —— 恢复被吞掉且再没排上，系统音量永久停在 12。
                # 实测踩过（2026-09-17，日志里连着刷"当前音量 12 看着像是
                # 上次没恢复的压制值"）。见 §6.6。
                self._duck_from = None
                self._ducked_to = None
                self._remember_duck_from(None)
                return
            # 只有不在压制中时才记原始值。但"当前音量"这个来源本身会骗人 ——
            # 它已经被压过的话，我们就会把压低后的值当成原始值记下来，
            # 而且从此再也回不去了（实测：一晚上之后系统音量永久停在 12，
            # 音乐、回话、什么都几乎听不见，看起来像喇叭坏了）。
            #
            # 怎么会已经被压过：**压制的恢复状态只活在进程内存里，而它守护的
            # 副作用（macOS 的系统音量）活得比进程久**。服务端在压制窗口里
            # 被重启/崩掉一次，音量就永远留在 12 了。今晚重启了七八次，
            # 每次都踩一遍。
            #
            # 两道防线：落盘的原始值优先（跨重启还认得回来），
            # 再不行就用一个下限兜底，绝不把"压制档位"本身当成原始值。
            self._duck_from = self._recall_duck_from() or max(cur, level + 1)
            if self._duck_from != cur:
                _LOG.info("当前音量 %.0f 看着像是上次没恢复的压制值，按 %.0f 记原始音量",
                          cur, self._duck_from)
        self._remember_duck_from(self._duck_from)
        if cur > level:
            # 只有真写下去了才更新 _ducked_to —— 恢复时要拿它和当前值比对，
            # 记一个我们没写过的值会让那次比对失去意义。
            await self.set_volume(level)
            self._ducked_to = level
        self._duck_task = asyncio.create_task(self._unduck_after(seconds))

    def _remember_duck_from(self, value: float | None) -> None:
        try:
            if value is None:
                os.path.exists(self._duck_file) and os.unlink(self._duck_file)
            else:
                with open(self._duck_file, "w") as f:
                    f.write(str(value))
        except OSError:
            pass        # 记不住就退回纯内存的老行为，不该因此让压音量整个失败

    def _recall_duck_from(self) -> float | None:
        """上一条命（可能是上一次进程）留下的原始音量。"""
        try:
            with open(self._duck_file) as f:
                v = float(f.read().strip())
            return v if 0 < v <= 100 else None
        except (OSError, ValueError):
            return None

    async def restore_volume_if_stuck(self, level: float) -> None:
        """启动时叫一次：上次是不是死在压制窗口里了。

        判据是"当前音量 <= 压制档位，而且盘上还留着一个更高的原始值" ——
        那就只能是上次没恢复成。用户自己把音量调到 12 以下也会命中，
        但那种情况恢复到他自己设过的原始值，也不算冤枉。
        """
        if not await self.linked():
            return
        cur = await self.volume()
        saved = self._recall_duck_from()
        if cur is None or saved is None or cur > level or saved <= cur:
            return
        _LOG.warning("上次是在压音量期间退出的，系统音量还停在 %.0f，恢复到 %.0f",
                     cur, saved)
        await self.set_volume(saved)
        self._remember_duck_from(None)

    # 恢复前比对当前值时允许的偏差。实测 macOS 的 set/get 是精确的
    # （1/5/12/13/35/40/55 设进去读回来一个不差），留 1 只是防四舍五入。
    _VOLUME_EPS = 1.0

    async def _unduck_after(self, seconds: float) -> None:
        """压制窗口结束，把音量放回去 —— **但要先确认没人插过手**。

        这里是整个压音量机制唯一可能覆盖用户意图的地方，所以判据要说清楚：

        `_duck_from` 不是一个关于世界的事实，而是一个**预测** ——
        "这 8 秒里不会有别人写系统音量"。用户在窗口里说「音量调到1」，
        或者有人伸手拧了实体旋钮，这个预测就作废了。
        **拿一个作废的预测去覆盖一个更新的、真实的用户意图，是纯粹的丢失更新。**

        所以恢复是 compare-and-swap：只有当前值还是我压下去的那个档位时，
        才说明这段时间没人动过，才轮得到我放回去。
        用显式的"用户改音量时取消恢复任务"也能治 A 类情况，但治不了实体遥控器
        那种绕过我们的写入 —— 而 AirPlay 会把设备音量同步回系统音量，
        那条路是真实存在的。比对当前值则两种都管。
        """
        try:
            await asyncio.sleep(seconds)
        except asyncio.CancelledError:
            return
        if self._duck_from is None:
            return
        now = await self.volume()
        if (now is not None and self._ducked_to is not None
                and abs(now - self._ducked_to) > self._VOLUME_EPS):
            _LOG.info("压制期间音量被改成了 %.0f（不是我压下去的 %.0f），"
                      "放弃恢复 —— 那是比我手里这个更新的用户意图", now, self._ducked_to)
            self._duck_from = None
            self._ducked_to = None
            self._remember_duck_from(None)
            return
        await self.set_volume(self._duck_from)
        self._duck_from = None
        self._ducked_to = None
        self._remember_duck_from(None)

    async def set_volume(self, pct: float) -> bool:
        pct = max(0.0, min(100.0, float(pct)))
        try:
            code, _ = await _run("/usr/bin/osascript", "-e",
                                 f"set volume output volume {pct:.0f}", timeout=5)
            return code == 0
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("设音量失败：%s", exc)
            return False

    async def volume(self) -> float | None:
        try:
            code, out = await _run("/usr/bin/osascript", "-e",
                                   "output volume of (get volume settings)", timeout=5)
            return float(out) if code == 0 and out.lstrip("-").isdigit() else None
        except Exception:  # noqa: BLE001
            return None
