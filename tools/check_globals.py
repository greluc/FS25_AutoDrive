"""Are AutoDrive's and Courseplay's global names still disjoint?

Both mods run in their own sandbox today, so a collision costs nothing. That changes the moment the
two share one environment - a merge, or one mod sourcing the other's file - and the failure mode is
the worst kind: the game's mod environments fall through to _G by __index, so a collision is a
silent last-definition-wins overwrite with no warning anywhere. One mod's Course class quietly
becomes the other's.

Right now the two sets do not overlap at all, which is what keeps a merge on the table. This check
is the cheap insurance that it stays that way. It is deliberately NOT part of check.py's pass/fail:
it needs the other repository, which is not always present, and a test that silently stops testing
when a path is missing is worse than no test.

    python tools/check_globals.py [--cp <path to Courseplay_FS25>]

Exit 0 when disjoint, 1 when they collide or the Courseplay tree cannot be found. The default path
is the sibling checkout used on this machine.
"""
import argparse
import re
import sys
from pathlib import Path

AD_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CP = AD_ROOT.parent.parent / "courseplay" / "Courseplay_FS25"

# Directories that never ship in the mod, so a name defined there cannot collide in the game.
SKIP_DIRS = {"tools", "test", "tests", ".git", ".github", ".idea"}

# An assignment at column zero, or a function defined at column zero: both create a global unless
# the line says local. Plus g_-prefixed assignments anywhere, which are globals by this project's
# convention and are usually written inside a function.
TOP_LEVEL_ASSIGN = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*(?:=[^=]|\[)", re.M)
TOP_LEVEL_FUNC = re.compile(r"^function\s+([A-Za-z_][A-Za-z0-9_]*)\s*[.:(]", re.M)
G_ASSIGN = re.compile(r"(?<![\w.:])(g_[A-Za-z0-9_]*)\s*=[^=]")
LOCAL_LINE = re.compile(r"^\s*local\b")


def globals_of(root: Path):
    """name -> the first file:line that defines it."""
    found = {}
    for path in sorted(root.rglob("*.lua")):
        if SKIP_DIRS & {p.name for p in path.parents}:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        lines = text.splitlines()
        for pattern in (TOP_LEVEL_ASSIGN, TOP_LEVEL_FUNC):
            for m in pattern.finditer(text):
                line_no = text.count("\n", 0, m.start()) + 1
                if LOCAL_LINE.match(lines[line_no - 1]):
                    continue
                found.setdefault(m.group(1), f"{path.relative_to(root)}:{line_no}")
        for m in G_ASSIGN.finditer(text):
            line_no = text.count("\n", 0, m.start()) + 1
            if LOCAL_LINE.match(lines[line_no - 1]):
                continue
            found.setdefault(m.group(1), f"{path.relative_to(root)}:{line_no}")
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cp", type=Path, default=DEFAULT_CP,
                        help="path to the Courseplay_FS25 checkout")
    args = parser.parse_args()

    if not (args.cp / "modDesc.xml").is_file():
        print(f"GLOBALS  cannot find Courseplay at {args.cp}")
        print("         pass --cp <path>; this check needs both trees and is not run by check.py")
        return 1

    ad = globals_of(AD_ROOT)
    cp = globals_of(args.cp)
    shared = sorted(set(ad) & set(cp))

    print(f"GLOBALS  AutoDrive {len(ad)}, Courseplay {len(cp)}, shared {len(shared)}")
    if not shared:
        print("         disjoint - the two would still cohabit in one environment")
        return 0
    print("         COLLISION: in a shared environment these silently overwrite each other")
    for name in shared:
        print(f"           {name}\n             AD: {ad[name]}\n             CP: {cp[name]}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
