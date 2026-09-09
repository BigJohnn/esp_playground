"""让板子自己找到服务端，而不是把 IP 编译进固件。

为什么不硬编码：这台 Mac 的地址是 DHCP 来的，跟灯泡一样会漂
（灯泡实测一小时内从 .106 漂到 .105）。板子上写死 IP，意味着每次续租
都要重新烧固件 —— 这是在用最贵的操作解决最廉价的问题。

为什么不用 mDNS：服务端要多装 zeroconf，板子要多链一个 mdns 组件，
而我们只需要回答一个问题"服务端在哪"。一问一答的 UDP 广播 20 行就够了，
而且这条路已经在 miIO 上验证过在这个网络里是通的。

协议：
  板 -> 广播:8791   "S31VOICE?"
  服务端 -> 板       "S31VOICE " + {"host": "…", "hosts": ["…", …], "port": 8790}

`hosts` 是**候选列表**，板子逐个试到通为止（见 net.c 的 pick_reachable_host）。
`host` 是其中最可能的那个，只为兼容旧固件保留。

为什么不是单个地址 —— 这里踩过一次坑，值得写下来：
第一版用"不发包的 UDP connect + getsockname"问内核"去板子该用哪个源地址"。
这个办法在只有物理网卡时是对的，但代理软件的 TUN 网卡（utun，地址常是
198.18.0.1 这类）会把整条路由抢过去，于是内核诚实地回答"用 198.18.0.1"，
而那个地址对板子毫无意义。板子把它存下来，之后每次请求都 ESP_ERR_HTTP_CONNECT。

更糟的是**它不报错**：服务端日志里"回 {host: 198.18.0.1}"看着像成功。
所以现在的做法是：只从真实网卡上列地址、明确剔掉已知没意义的段、
按"和板子同网段"排序、把整串给板子让它自己验证；一个同网段的都没有时，
在日志里喊出来，而不是默默回一个能骗过所有人的地址。
"""
from __future__ import annotations

import ipaddress
import json
import logging
import socket
import subprocess
import threading

_LOG = logging.getLogger("discovery")

DISCOVERY_PORT = 8791
PROBE = b"S31VOICE?"
REPLY_PREFIX = b"S31VOICE "

# 这些段就算配在网卡上，也不是板子能用来找到我们的地址。
_USELESS = [
    ipaddress.ip_network("127.0.0.0/8"),      # 回环
    ipaddress.ip_network("169.254.0.0/16"),   # 没拿到 DHCP 时的自赋地址
    ipaddress.ip_network("198.18.0.0/15"),    # 基准测试段，被各种代理 TUN 拿去当假网关
]


def local_ipv4s() -> list[tuple[str, str]]:
    """本机每张真实网卡上的 (地址, 掩码)。

    和 miio.local_broadcasts 一样靠 ifconfig：装 psutil/netifaces 只为读几行地址
    不划算，而这个工程本来就只在 macOS 上跑。
    """
    out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    found: list[tuple[str, str]] = []
    for line in out.splitlines():
        line = line.strip()
        if not line.startswith("inet "):
            continue
        parts = line.split()
        addr = parts[1]
        mask = "255.255.255.0"
        if "netmask" in parts:
            raw = parts[parts.index("netmask") + 1]
            if raw.startswith("0x"):
                mask = str(ipaddress.IPv4Address(int(raw, 16)))
            else:
                mask = raw
        found.append((addr, mask))
    return found


def candidates_for(peer: str) -> list[str]:
    """给这台板子的候选地址，最可能的排前面。

    排序依据只有一条：**能不能让板子连上**。
      1. 和板子同一个 /掩码 网段的  —— 一定通
      2. 其它私网地址              —— 路由器可能在两个网段之间转发，值得试
      3. 剩下的                    —— 兜底
    """
    try:
        peer_ip = ipaddress.ip_address(peer)
    except ValueError:
        peer_ip = None

    same_subnet, private, other = [], [], []
    for addr, mask in local_ipv4s():
        try:
            ip = ipaddress.ip_address(addr)
        except ValueError:
            continue
        if any(ip in net for net in _USELESS):
            continue
        if peer_ip is not None:
            try:
                if peer_ip in ipaddress.ip_network(f"{addr}/{mask}", strict=False):
                    same_subnet.append(addr)
                    continue
            except ValueError:
                pass
        (private if ip.is_private else other).append(addr)

    if not same_subnet:
        # 这不是小事：多半是那张和板子同网段的网卡掉了（实测就是 USB 有线网卡被拔）。
        # 后面的候选只是"也许路由器会转发"，成不成要板子自己试。
        _LOG.warning("没有和板子 %s 同网段的本机地址！候选只能靠猜：%s。"
                     "检查那张和板子同网段的网卡是不是断了", peer, private + other)
    return same_subnet + private + other


def _fake_hosts() -> list[str] | None:
    """排查/回归用：DISCOVERY_FAKE_HOSTS=1.2.3.4,5.6.7.8 就回这串假地址。

    没有它，"发现服务给了个连不上的地址"这个故障只能靠物理拔网线来复现 ——
    而它正是这轮真出过的那个 bug，值得能一键重演。
    """
    import os
    raw = os.environ.get("DISCOVERY_FAKE_HOSTS", "").strip()
    return [h.strip() for h in raw.split(",") if h.strip()] if raw else None


def serve(port: int, stop: threading.Event) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.5)
    try:
        sock.bind(("0.0.0.0", DISCOVERY_PORT))
    except OSError as e:
        _LOG.warning("发现服务起不来（:%d 被占用？）: %s", DISCOVERY_PORT, e)
        return
    _LOG.info("发现服务在 UDP :%d 上等板子", DISCOVERY_PORT)

    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break
        if not data.startswith(PROBE):
            continue
        hosts = _fake_hosts() or candidates_for(addr[0])
        if not hosts:
            _LOG.error("一个可用的本机地址都没有，没法回答板子 %s", addr[0])
            continue
        body = json.dumps({"host": hosts[0], "hosts": hosts, "port": port}).encode()
        sock.sendto(REPLY_PREFIX + body, addr)
        _LOG.info("板子 %s 来问路，回 %s", addr[0], body.decode())
    sock.close()


def start(port: int) -> tuple[threading.Thread, threading.Event]:
    stop = threading.Event()
    t = threading.Thread(target=serve, args=(port, stop), daemon=True, name="discovery")
    t.start()
    return t, stop


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)-7s %(message)s")
    print("本机地址:")
    for a, m in local_ipv4s():
        print(f"  {a}/{m}")
    import sys
    peer = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.7"
    print(f"给 {peer} 的候选: {candidates_for(peer)}")
