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
