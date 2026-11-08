"""The hint ladder behind `python3 hint.py`: three hints per block, from the idea to the code.

The texts are in hints.dat next to this file; the hints a group has opened, with the time, in .hints.json
at the root of the repository. A new hint opens only after a try of your own, and not straight after
the last one.
"""
import argparse
import base64
import json
import math
import os
import re
import textwrap
import time
import zlib

PACKAGE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(PACKAGE)
DATA = os.path.join(PACKAGE, "hints.dat")
STATE = os.path.join(ROOT, ".hints.json")
WAIT = 300.0                    # s from one hint of a block to the next


def encode(data: dict) -> str:
    raw = zlib.compress(json.dumps(data, ensure_ascii=False, sort_keys=True).encode("utf-8"), 9)
    return "\n".join(textwrap.wrap(base64.b64encode(raw).decode("ascii"), 76)) + "\n"


def decode(text: str) -> dict:
    return json.loads(zlib.decompress(base64.b64decode("".join(text.split()))).decode("utf-8"))


def load(path: str = DATA) -> dict:
    with open(path, encoding="ascii") as handle:
        return decode(handle.read())


def load_state(path: str = None) -> dict:
    try:
        with open(path or STATE, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}


def save_state(seen: dict, path: str = None) -> None:
    with open(path or STATE, "w", encoding="utf-8") as handle:
        json.dump(seen, handle, indent=1, sort_keys=True)


def untouched(block: str, root: str = ROOT) -> bool:
    """True while a file in student/ still has the raise of the template for this block."""
    folder = os.path.join(root, "student")
    mark = re.compile(r'^\s*raise NotImplementedError\("STUDENT ' + re.escape(block) + r'"\)', re.M)
    for name in sorted(os.listdir(folder)):
        if name.endswith(".py"):
            with open(os.path.join(folder, name), encoding="utf-8") as handle:
                if mark.search(handle.read()):
                    return True
    return False


def indent(text: str, by: int = 4) -> str:
    return textwrap.indent(text, " " * by)


def ask(block: str, seen: dict, now: float, data: dict, root: str = ROOT, wait: float = WAIT,
        repeat: bool = False) -> str:
    """What `hint.py <block>` prints. Opens the next hint if it may, and notes it in `seen`."""
    title, levels = data["blocks"][block]["title"], data["blocks"][block]["levels"]
    times = seen.get(block, [])
    out = [f"STUDENT {block}: {title}"]
    for i in range(len(times)):
        out.append(f"\n  hint {i + 1} of {len(levels)}\n" + indent(levels[i]))
    if repeat:
        if not times:
            out.append(f"\n  no hint opened yet:  python3 hint.py {block}")
        return "\n".join(out)
    if len(times) == len(levels):
        out.append("\n  That was the last hint. Still stuck? Ask your supervisor.")
        return "\n".join(out)
    egg = None
    if times and untouched(block, root):
        egg = "untouched"
    elif times and now - times[-1] < wait:
        egg = "wait"
    if egg:
        left = math.ceil(wait - (now - times[-1]))
        out.append("\n" + indent(data["eggs"][egg].format(
            block=block, test="extra" if block.startswith("Z") else "task" + block.split(".")[0],
            level=len(times) + 1, previous=len(times), wait=f"{left // 60}:{left % 60:02d} min"), 2))
        return "\n".join(out)
    seen[block] = times = times + [now]
    out.append(f"\n  hint {len(times)} of {len(levels)}\n" + indent(levels[len(times) - 1]))
    if len(times) < len(levels):
        out.append(f"\n  (python3 hint.py {block} again for hint {len(times) + 1})")
    return "\n".join(out)


def overview(seen: dict, data: dict) -> str:
    rows = ["Hints opened so far:", ""]
    for block, entry in data["blocks"].items():
        n, total = len(seen.get(block, [])), len(entry["levels"])
        rows.append(f"  {block:4} {'#' * n}{'.' * (total - n)}  {entry['title']}")
    rows += ["", "python3 hint.py 1.3       the next hint for STUDENT 1.3",
             "python3 hint.py 1.3 -r    the hints of 1.3 you already have, again"]
    return "\n".join(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="hint.py", description="Hints for the STUDENT blocks, one at a time.")
    ap.add_argument("block", nargs="?", help="the block, as in # TODO STUDENT 1.3")
    ap.add_argument("-r", "--repeat", action="store_true", help="show the hints you have, open no new one")
    args = ap.parse_args(argv)
    data, seen = load(), load_state()
    if args.block is None:
        print(overview(seen, data))
        return 0
    block = args.block.upper().removeprefix("STUDENT").strip()
    if block not in data["blocks"]:
        ap.error(f"no block {args.block!r}; the blocks are " + ", ".join(data["blocks"]))
    print(ask(block, seen, time.time(), data, repeat=args.repeat))
    save_state(seen)
    return 0
