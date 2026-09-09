#!/usr/bin/env python3
"""从 Home Assistant 的 xiaomi_home 集成存储里提取设备的 miIO did / token。

有了 token 就能在局域网里直连设备（UDP 54321），绕开小米云 —— 实测延迟从 ~1s 降到 ~300ms。

存储格式：明文 JSON，末尾附 32 字节 SHA256 校验。直接 json.load 会因为把 hash 一起读进去而失败。

用法：python3 tools/dump_miio_token.py [.storage/xiaomi_home 路径]
"""
import hashlib
import json
import sys
from pathlib import Path

DEFAULT = Path(__file__).resolve().parents[2] / "home_assistant/config/.storage/xiaomi_home"


def load_dict(path: Path):
    raw = path.read_bytes()
    body, digest = raw[:-32], raw[-32:]
    if hashlib.sha256(body).digest() != digest:
        print(f"警告：{path.name} 的 hash 校验没过，文件可能损坏", file=sys.stderr)
    return json.loads(body)


def main() -> None:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT
    files = sorted((root / "miot_devices").glob("*.dict"))
    if not files:
        sys.exit(f"{root}/miot_devices 下没有 .dict 文件，先在 HA 里配好 Xiaomi Home 集成")

    for f in files:
        for did, dev in load_dict(f).items():
            token = dev.get("token")
            wired = "Wi-Fi" if dev.get("connect_type") == 0 else f"connect_type={dev.get('connect_type')}"
            print(f"{dev.get('name')}")
            print(f"  model = {dev.get('model')}   ({wired})")
            print(f"  MIIO_DID={did}")
            print(f"  MIIO_TOKEN={token}" if token else "  （这台设备没有 token，多半不是 Wi-Fi 设备，没法 miIO 直控）")
            print()


if __name__ == "__main__":
    main()
