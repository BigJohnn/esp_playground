"""miIO 局域网协议的最小实现。

为什么不用 python-miio：我们只需要 handshake + set_properties/get_properties 两件事，
自己写不到 150 行，还能精确控制超时和重发。python-miio 会带进来一大堆依赖。

协议（小米自家的 UDP:54321）：
    magic    2B  0x2131
    length   2B  整包长度
    unknown  4B  hello 包为 0xFFFFFFFF，其余为 0
    did      4B  设备 id
    stamp    4B  设备侧的运行秒数，必须跟设备对齐，否则包被丢弃
    checksum 16B md5(把 checksum 字段替换成 token 后的整包)
    payload  nB  AES-128-CBC(md5(token), md5(md5(token)+token)) 加密的 JSON

token 从 Home Assistant 的 xiaomi_home 存储里拿（见 tools/dump_miio_token.py）。
"""
from __future__ import annotations

import hashlib
import json
import logging
import socket
import struct
import time
from dataclasses import dataclass

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

_LOG = logging.getLogger("miio")

MAGIC = 0x2131
PORT = 54321
# 握手多久算过期。设备只认和它自己时钟接近的 stamp，所以隔久了必须重握。
# 60 秒是保守值；真正的代价是每次重握都要一个额外的 UDP 往返，
# 所以另有一条后台任务在这个间隔之内提前握好（见 executor.py 的 _keep_warm）。
HANDSHAKE_TTL = 60
HELLO = bytes.fromhex("21310020" + "ff" * 28)


def local_broadcasts() -> list[str]:
    """本机每个网卡所在 /24 的广播地址。

    只发 255.255.255.255 是不够的：这台机器同时接了 192.168.1.x 和 192.168.0.x 两个网段，
    受限广播只会从默认路由那张网卡出去，而灯泡在另一张上。
    """
    import subprocess

    out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    nets = []
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("inet ") and "broadcast" in line:
            parts = line.split()
            bcast = parts[parts.index("broadcast") + 1]
            if bcast not in nets:
                nets.append(bcast)
    nets.append("255.255.255.255")
    return nets


def _pad(data: bytes) -> bytes:
    n = 16 - len(data) % 16
    return data + bytes([n]) * n


def _unpad(data: bytes) -> bytes:
    return data[: -data[-1]] if data else data


class MiioError(RuntimeError):
    pass


@dataclass
class _Handshake:
    did: int
    stamp: int
    received_at: float


class MiioDevice:
    """一台 miIO 设备。IP 按 did 动态发现 —— 实测这只灯泡的 DHCP 租约会漂，写死 IP 必挂。"""

    def __init__(self, did: int, token: str, ip: str | None = None,
                 timeout: float = 1.0, broadcast_nets: list[str] | None = None) -> None:
        self.did = did
        self.token = bytes.fromhex(token)
        if len(self.token) != 16:
            raise ValueError("token 必须是 32 位十六进制")
        self.ip = ip
        self.timeout = timeout
        self.broadcast_nets = broadcast_nets or local_broadcasts()

        self._key = hashlib.md5(self.token).digest()
        self._iv = hashlib.md5(self._key + self.token).digest()
        self._hs: _Handshake | None = None
        self._req_id = 1

    # ---------- 加解密 ----------

    def _encrypt(self, plain: bytes) -> bytes:
        enc = Cipher(algorithms.AES(self._key), modes.CBC(self._iv)).encryptor()
        return enc.update(_pad(plain)) + enc.finalize()

    def _decrypt(self, data: bytes) -> bytes:
        dec = Cipher(algorithms.AES(self._key), modes.CBC(self._iv)).decryptor()
        return _unpad(dec.update(data) + dec.finalize())

    # ---------- 组包 ----------

    def _build(self, payload: bytes, stamp: int) -> bytes:
        body = self._encrypt(payload)
        header = struct.pack(">HHIII", MAGIC, 32 + len(body), 0, self.did, stamp)
        # checksum 位先填 token，md5 之后再换成真正的 checksum
        checksum = hashlib.md5(header + self.token + body).digest()
        return header + checksum + body

    # ---------- 发现 / 握手 ----------

    def discover(self) -> str:
        """广播 hello，找到 did 匹配的那台设备的 IP。"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.4)
        try:
            deadline = time.time() + 3.0
            next_send = 0.0
            while time.time() < deadline:
                # 每 0.6s 重发一轮：UDP 广播丢一两个包很正常，实测确实会偶发丢失
                if time.time() >= next_send:
                    for net in self.broadcast_nets:
                        try:
                            sock.sendto(HELLO, (net, PORT))
                        except OSError:
                            pass
                    next_send = time.time() + 0.6
                try:
                    data, addr = sock.recvfrom(1024)
                except socket.timeout:
                    continue
                if len(data) >= 32 and data[:2] == b"\x21\x31":
                    did = struct.unpack(">I", data[8:12])[0]
                    if did == self.did:
                        self.ip = addr[0]
                        self._hs = _Handshake(
                            did, struct.unpack(">I", data[12:16])[0], time.time())
                        _LOG.info("发现设备 did=%d @ %s", did, self.ip)
                        return self.ip
        finally:
            sock.close()
        raise MiioError(f"局域网里没找到 did={self.did} 的设备")

    def _handshake(self) -> _Handshake:
        if self.ip is None:
            self.discover()
            assert self._hs is not None
            return self._hs

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout)
        try:
            sock.sendto(HELLO, (self.ip, PORT))
            data, _ = sock.recvfrom(1024)
        except (socket.timeout, OSError):
            # IP 可能变了（DHCP），重新发现一次
            self.ip = None
            self.discover()
            assert self._hs is not None
            return self._hs
        finally:
            sock.close()

        self._hs = _Handshake(
            struct.unpack(">I", data[8:12])[0],
            struct.unpack(">I", data[12:16])[0],
            time.time())
        return self._hs

    def _stamp_now(self) -> tuple[int, float]:
        """返回 (stamp, 为此花掉的握手秒数)。

        握手时间要单独回出去，不能混进请求耗时里：控灯慢的时候，
        "多握了一次手"和"设备本身响应慢"是两种病 —— 前者能靠预热消掉，后者不能。
        """
        # 设备只接受和它自己时钟接近的 stamp，所以要用握手值 + 本地流逝的秒数
        hs_secs = 0.0
        if self._hs is None or time.time() - self._hs.received_at > HANDSHAKE_TTL:
            t0 = time.perf_counter()
            self._handshake()
            hs_secs = time.perf_counter() - t0
        assert self._hs is not None
        return self._hs.stamp + int(time.time() - self._hs.received_at), hs_secs

    def warm(self) -> bool:
        """提前把握手做掉。给后台预热任务用 —— 见 executor.py。"""
        try:
            self._stamp_now()
            return True
        except (socket.timeout, MiioError, OSError) as exc:
            _LOG.debug("预热握手失败: %s", exc)
            self._hs = None
            return False

    # ---------- 调用 ----------

    def send(self, method: str, params, retries: int = 2):
        last: Exception | None = None
        for attempt in range(retries + 1):
            try:
                return self._send_once(method, params)
            except (socket.timeout, MiioError, OSError) as exc:
                last = exc
                _LOG.debug("第 %d 次 %s 失败: %s", attempt + 1, method, exc)
                self._hs = None      # 强制重新握手
                if attempt == 0:
                    self.ip = None   # 第二次起顺便重新发现 IP
        raise MiioError(f"{method} 失败: {last}")

    def _send_once(self, method: str, params):
        stamp, hs_secs = self._stamp_now()
        t0 = time.perf_counter()
        self._req_id = self._req_id % 9999 + 1
        payload = json.dumps(
            {"id": self._req_id, "method": method, "params": params},
            separators=(",", ":")).encode()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout)
        try:
            sock.sendto(self._build(payload, stamp), (self.ip, PORT))
            data, _ = sock.recvfrom(4096)
        finally:
            sock.close()

        _LOG.info("miIO %s: 握手 %d ms + 请求 %d ms", method,
                  round(hs_secs * 1000), round((time.perf_counter() - t0) * 1000))
        if len(data) <= 32:
            raise MiioError("设备返回了空包（通常是 token 不对或 stamp 不同步）")
        resp = json.loads(self._decrypt(data[32:]))
        if "error" in resp:
            raise MiioError(f"设备报错: {resp['error']}")
        return resp.get("result")


class MiotLight:
    """按 MIoT spec 操作这只灯：siid=2 下 piid 1=开关 2=亮度 3=色温。"""

    SIID = 2
    P_ON, P_BRIGHT, P_CCT = 1, 2, 3

    def __init__(self, dev: MiioDevice) -> None:
        self.dev = dev

    def _set(self, piid: int, value):
        return self.set_many({piid: value})

    def set_many(self, props: dict[int, object]):
        """一次 set_properties 写多个属性。

        协议本身就接受属性数组，而每次 UDP 往返实测 ~300ms，
        把"开灯+设亮度+设色温"拆成三次调用会让延迟直接翻三倍。
        """
        return self.dev.send("set_properties", [
            {"did": str(self.dev.did), "siid": self.SIID, "piid": piid, "value": value}
            for piid, value in props.items()])

    def get(self) -> dict:
        res = self.dev.send("get_properties", [
            {"did": str(self.dev.did), "siid": self.SIID, "piid": p}
            for p in (self.P_ON, self.P_BRIGHT, self.P_CCT)])
        out = {}
        for item in res or []:
            out[{self.P_ON: "on", self.P_BRIGHT: "brightness", self.P_CCT: "kelvin"}
                .get(item.get("piid"), item.get("piid"))] = item.get("value")
        return out

    def turn_on(self):            return self._set(self.P_ON, True)
    def turn_off(self):           return self._set(self.P_ON, False)
    def set_brightness(self, pct: int):
        return self._set(self.P_BRIGHT, max(1, min(100, int(pct))))
    def set_kelvin(self, k: int):
        return self._set(self.P_CCT, max(2700, min(6500, int(k))))
