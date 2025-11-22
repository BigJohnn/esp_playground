import sys
from pathlib import Path

if len(sys.argv) < 2:
    print("Usage: python print_audio.py <path> [max_lines]")
    sys.exit(1)

target = Path(sys.argv[1])
max_lines = int(sys.argv[2]) if len(sys.argv) > 2 else 80

content = target.read_text(encoding="utf-8")
for idx, line in enumerate(content.splitlines(), start=1):
    print(f"{idx}: {line}")
    if idx >= max_lines:
        break
