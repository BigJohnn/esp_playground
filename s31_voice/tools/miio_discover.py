#!/usr/bin/env python3
"""扫描局域网上的小米 Wi-Fi (miIO) 设备。

miIO 设备在 UDP 54321 上响应 "hello" 握手包（0x2131 + 全 0xff），回包里带 device id。
MYRKISEL 是 Wi-Fi 米家灯泡，所以能被这个脚本扫到 —— 这也是判定"它不是蓝牙 Mesh"的依据。

用法：  python3 miio_discover.py [网段前缀 ...]
不带参数时自动用本机各网卡所在的 /24。
"""
import ipaddress
import socket
import struct
import subprocess
import sys
import time

HELLO = bytes.fromhex("21310020" + "ff" * 28)


def local_subnets() -> list[str]:
    out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    nets = []
    for line in out.splitlines():
        line = line.strip()
        if not line.startswith("inet ") or "127.0.0.1" in line:
            continue
        ip = line.split()[1]
        try:
            net = ipaddress.ip_network(ip + "/24", strict=False)
        except ValueError:
            continue
        if net.is_private and str(net) not in nets:
            nets.append(str(net))
    return nets


def scan(networks: list[str], rounds: int = 2, wait: float = 3.0) -> dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.5)

    targets = ["255.255.255.255"]
    for net in networks:
        targets += [str(h) for h in ipaddress.ip_network(net).hosts()]

    found: dict[str, int] = {}
    for _ in range(rounds):
        for t in targets:
            try:
                sock.sendto(HELLO, (t, 54321))
            except OSError:
                pass
        deadline = time.time() + wait
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            if len(data) >= 32 and data[:2] == b"\x21\x31":
                found[addr[0]] = struct.unpack(">I", data[8:12])[0]
    return found


def mac_of(ip: str) -> str:
    subprocess.run(["ping", "-c1", "-W500", ip], capture_output=True)
    out = subprocess.run(["arp", "-n", ip], capture_output=True, text=True).stdout
    for tok in out.split():
        if tok.count(":") == 5:
            return tok
    return "?"


if __name__ == "__main__":
    nets = sys.argv[1:] or local_subnets()
    print("扫描网段:", ", ".join(nets))
    devices = scan(nets)
    if not devices:
        print("没有发现 miIO 设备。如果灯泡在别的网段/VLAN，把网段作为参数传进来。")
    for ip, did in sorted(devices.items()):
        print(f"  {ip:<16} device_id={did:<12} mac={mac_of(ip)}")
