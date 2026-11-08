"""`python3 hint.py`: every block has three hints, a new one needs a try and some time, the texts stay hidden."""
import os
import re
import sys
import textwrap

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from ohm_icp import hints                                          # noqa: E402

DATA = hints.load()
BLOCK = re.compile(r"^[ \t]*# (?:>>> |TODO )STUDENT ([\dZ]+\.\d+)", re.M)
SOLUTION = re.compile(r"^[ \t]*# >>> STUDENT ([\dZ]+\.\d+)\n(.*?)^[ \t]*# <<< STUDENT \1", re.M | re.S)


def student_texts():
    folder = os.path.join(ROOT, "student")
    return [open(os.path.join(folder, n), encoding="utf-8").read() for n in sorted(os.listdir(folder))
            if n.endswith(".py")]


def level(block, i):
    """Hint i of a block as hint.py prints it."""
    return hints.indent(DATA["blocks"][block]["levels"][i])


def make_root(tmp_path, code):
    (tmp_path / "student").mkdir(parents=True)
    (tmp_path / "student" / "icp.py").write_text(code)
    return str(tmp_path)


EMPTY = '    # TODO STUDENT 1.3\n    raise NotImplementedError("STUDENT 1.3")\n'
TRIED = "    # TODO STUDENT 1.3\n    inliers = []\n"


def test_every_block_has_three_hints():
    blocks = [b for text in student_texts() for b in BLOCK.findall(text)]
    assert sorted(blocks) == sorted(DATA["blocks"])
    for block, entry in DATA["blocks"].items():
        assert len(entry["levels"]) == 3 and all(level.strip() for level in entry["levels"]), block


def test_the_last_hint_is_the_solution():
    found = {b: code for text in student_texts() for b, code in SOLUTION.findall(text)}
    if not found:
        pytest.skip("this is the template: the solution stays with the supervisor")
    for block, code in found.items():
        assert DATA["blocks"][block]["levels"][2] == textwrap.dedent(code).strip(), block


def test_hints_dat_is_up_to_date():
    if not os.path.exists(os.path.join(ROOT, "tools", "hints.py")):
        pytest.skip("this is the template: tools/ stays with the solution")
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import hints as source
    assert hints.load() == source.data(), "python3 tools/hints.py"


def test_the_texts_are_not_readable_in_the_package():
    secret = [entry["levels"][2].split("\n")[0] for entry in DATA["blocks"].values()]
    secret += [egg.split("\n")[0].strip("* ") for egg in DATA["eggs"].values()]
    for folder, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs if d not in {".git", "build", "install", "log", "tools", "student", "docs"}]
        for name in files:
            if name.endswith((".py", ".md", ".json", ".txt", ".dat", ".rviz", ".xml")):
                with open(os.path.join(folder, name), encoding="utf-8", errors="ignore") as handle:
                    text = handle.read()
                assert not [s for s in secret if s in text], os.path.join(folder, name)


def test_one_hint_after_the_other(tmp_path):
    root, seen = make_root(tmp_path, TRIED), {}
    first = hints.ask("1.3", seen, 1000.0, DATA, root)
    assert level("1.3", 0) in first and level("1.3", 1) not in first
    second = hints.ask("1.3", seen, 1000.0 + hints.WAIT, DATA, root)
    assert level("1.3", 1) in second and len(seen["1.3"]) == 2
    hints.ask("1.3", seen, 1000.0 + 2 * hints.WAIT, DATA, root)
    last = hints.ask("1.3", seen, 1000.0 + 3 * hints.WAIT, DATA, root)
    assert "last hint" in last and len(seen["1.3"]) == 3
    assert level("1.3", 2) in hints.ask("1.3", seen, 0.0, DATA, root, repeat=True)


def test_too_soon_after_the_last_hint(tmp_path):
    root, seen = make_root(tmp_path, TRIED), {}
    hints.ask("1.3", seen, 1000.0, DATA, root)
    out = hints.ask("1.3", seen, 1000.0 + 61.0, DATA, root)
    assert DATA["eggs"]["wait"].split("\n")[0] in out and "3:59 min" in out
    assert level("1.3", 1) not in out and len(seen["1.3"]) == 1


def test_no_second_hint_before_a_try(tmp_path):
    root, seen = make_root(tmp_path, EMPTY), {}
    assert level("1.3", 0) in hints.ask("1.3", seen, 1000.0, DATA, root)
    out = hints.ask("1.3", seen, 1000.0 + 10 * hints.WAIT, DATA, root)
    assert DATA["eggs"]["untouched"].split("\n")[0] in out and "-k task1" in out
    assert level("1.3", 1) not in out and len(seen["1.3"]) == 1
    assert "-k extra" in hints.ask("Z.1", {"Z.1": [0.0]}, 1e9, DATA, make_root(tmp_path / "z", EMPTY.replace(
        "1.3", "Z.1")))


def test_the_command_line_keeps_the_state(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(hints, "STATE", str(tmp_path / ".hints.json"))
    assert hints.main(["student 2.1"]) == 0
    assert level("2.1", 0) in capsys.readouterr().out
    assert hints.main([]) == 0
    assert re.search(r"2\.1\s+#\.\.", capsys.readouterr().out)
    with pytest.raises(SystemExit):
        hints.main(["7.7"])
