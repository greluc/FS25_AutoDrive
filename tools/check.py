#!/usr/bin/env python3
"""
Verification gate for AutoDrive.

Two independent checks, both of which must pass before a change is considered done:

  1. COMPILE - every .lua file in the mod is compiled with the Luau compiler the game itself
     uses (via fs-luau-compile from fs-utils). This catches syntax errors that a Lua 5.4
     parser would accept or reject differently from FS25. If the compiler is not available
     the check degrades to `luac -p`, which is weaker but better than nothing, and says so.

  2. UNIT TESTS - every tools/tests/test_*.lua is run with the Lua interpreter. Each file is a
     standalone luaunit suite and exits non-zero on failure.

Usage:
    python tools/check.py             # both
    python tools/check.py --compile   # syntax only (fast)
    python tools/check.py --tests     # unit tests only
    python tools/check.py -v          # show individual test names
"""

import argparse
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parent.parent
TESTS = REPO / "tools" / "tests"

LUAU_COMPILER_CANDIDATES = [
    pathlib.Path(
        r"D:/Coding/greluc/fs25/farmmanager/tools/fs-utils/target/release/fs-luau-compile.exe"
    ),
    REPO / "tools" / "bin" / "fs-luau-compile.exe",
]

GREEN, RED, YELLOW, DIM, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def find_lua():
    for name in ("lua", "lua5.4", "lua54", "luajit"):
        p = shutil.which(name)
        if p:
            return p
    # winget installs outside PATH for the current shell
    base = pathlib.Path(os.path.expanduser("~")) / "AppData/Local/Microsoft/WinGet/Packages"
    if base.exists():
        for c in base.rglob("lua.exe"):
            return str(c)
    return None


def find_luau_compiler():
    for c in LUAU_COMPILER_CANDIDATES:
        if c.exists():
            return c
    p = shutil.which("fs-luau-compile")
    return pathlib.Path(p) if p else None


def compiler_skip_encoding_flag(compiler):
    """['-e'] if this build of fs-luau-compile has it, [] otherwise.

    -e skips writing the ENCODED bytecode, which a syntax gate does not need and which costs a
    shift table lookup per file. Older builds of fs-luau-compile do not have the switch, and argh
    treats an unknown argument as a hard error rather than ignoring it - so passing it blindly
    failed every single file with "Unrecognized argument" while the sources were all fine. CI
    builds the compiler from the public repository and hit exactly that; the locally built one has
    the flag. Ask rather than assume.
    """
    try:
        res = subprocess.run([str(compiler), "--help"], capture_output=True, text=True, timeout=60)
    except Exception:
        return []
    return ["-e"] if "-e" in (res.stdout or "") + (res.stderr or "") else []


def lua_files():
    """Every .lua file that ships in the mod. tools/ is excluded from the release zip."""
    out = []
    for f in sorted(REPO.rglob("*.lua")):
        rel = f.relative_to(REPO)
        if rel.parts[0] in (".git", "tools"):
            continue
        out.append(f)
    return out


def find_luau_compile():
    """The official Luau compiler from luau-lang/luau, if it is on the PATH or in tools/bin."""
    for name in ("luau-compile", "luau-compile.exe"):
        p = shutil.which(name)
        if p:
            return pathlib.Path(p)
    for c in (REPO / "tools" / "bin" / "luau-compile", REPO / "tools" / "bin" / "luau-compile.exe"):
        if c.exists():
            return c
    return None


def compile_backends():
    """Every way of checking Lua syntax that this machine offers, strongest first.

    Each entry is (label, argv builder). The builder gets the file and a scratch output path and
    returns the command line.
    """
    out = []

    fs_luau = find_luau_compiler()
    if fs_luau:
        skip = compiler_skip_encoding_flag(fs_luau)
        out.append((f"Luau ({fs_luau.name})",
                    lambda f, o, e=fs_luau, s=skip: [str(e)] + s + [str(f), str(o)]))

    luau = find_luau_compile()
    if luau:
        # --null compiles in full and emits nothing. --binary would work too, but it writes the
        # bytecode to STDOUT, which this reads as text and chokes on.
        out.append((f"Luau ({luau.name}, luau-lang)",
                    lambda f, o, e=luau: [str(e), "--null", str(f)]))

    luac = shutil.which("luac") or shutil.which("luac5.4")
    if luac:
        out.append(("luac -p (Ersatz, nicht der Spiel-Dialekt)",
                    lambda f, o, e=luac: [e, "-p", str(f)]))

    return out


def backend_works(build_argv):
    """Compile something trivially valid before trusting a backend with the real sources.

    A binary that exists but cannot run is worse than one that is missing: it reports every file as
    broken and the failure looks like a hundred and twenty-eight syntax errors. That is exactly what
    happened in CI, twice - once with an argument the build did not accept, then with a wrapper whose
    own backing compiler was not installed - while every source file was fine.
    """
    with tempfile.TemporaryDirectory() as tmp:
        probe = pathlib.Path(tmp) / "probe.lua"
        probe.write_text("local a = 1\nreturn a\n", encoding="utf-8")
        try:
            res = subprocess.run(build_argv(probe, pathlib.Path(tmp) / "probe.out"),
                                 capture_output=True, text=True, timeout=120)
        except Exception:
            return False
        return res.returncode == 0


def check_compile(verbose):
    files = lua_files()
    failures = []

    label, build_argv = None, None
    for candidate_label, candidate_argv in compile_backends():
        if backend_works(candidate_argv):
            label, build_argv = candidate_label, candidate_argv
            break
        print(f"{YELLOW}COMPILE  {candidate_label} ist vorhanden, laeuft aber nicht - naechster{RESET}")

    if build_argv is None:
        print(f"{YELLOW}COMPILE  uebersprungen - kein funktionierender Compiler gefunden{RESET}")
        return None

    with tempfile.TemporaryDirectory() as tmp:
        for i, f in enumerate(files):
            res = subprocess.run(build_argv(f, pathlib.Path(tmp) / f"{i}.out"),
                                 capture_output=True, text=True)
            if res.returncode != 0:
                failures.append((f, (res.stderr or res.stdout).strip().splitlines()[:3]))

    if failures:
        print(f"{RED}COMPILE  {len(files) - len(failures)}/{len(files)} ok, {len(failures)} FEHLER{RESET}  {DIM}[{label}]{RESET}")
        for f, msg in failures:
            print(f"  {RED}x{RESET} {f.relative_to(REPO)}")
            for line in msg:
                print(f"      {DIM}{line}{RESET}")
        return False

    print(f"{GREEN}COMPILE  {len(files)}/{len(files)} Dateien ok{RESET}  {DIM}[{label}]{RESET}")
    return True


def check_tests(verbose):
    lua = find_lua()
    if not lua:
        print(f"{YELLOW}TESTS    uebersprungen - kein Lua-Interpreter gefunden{RESET}")
        return None

    suites = sorted(TESTS.glob("test_*.lua"))
    if not suites:
        print(f"{YELLOW}TESTS    keine test_*.lua in tools/tests{RESET}")
        return None

    total_ok = total_fail = 0
    failed_suites = []

    for suite in suites:
        res = subprocess.run(
            [lua, suite.name], cwd=str(TESTS), capture_output=True, text=True
        )
        out = res.stdout + res.stderr
        ok = fail = 0
        for line in out.splitlines():
            # luaunit summary: "Ran 12 tests in 0.001 seconds, 12 successes, 0 failures"
            if "successes" in line and "Ran" in line:
                parts = line.replace(",", " ").split()
                for i, p in enumerate(parts):
                    if p.startswith("success") and i > 0:
                        ok = int(parts[i - 1])
                    if p.startswith("failure") and i > 0:
                        fail = int(parts[i - 1])
                    if p.startswith("error") and i > 0 and parts[i - 1].isdigit():
                        fail += int(parts[i - 1])
        total_ok += ok
        total_fail += fail
        bad = res.returncode != 0 or fail > 0
        if bad:
            failed_suites.append((suite, out))
        mark = f"{RED}x{RESET}" if bad else f"{GREEN}+{RESET}"
        if verbose or bad:
            print(f"  {mark} {suite.name}  {DIM}{ok} ok, {fail} fehlgeschlagen{RESET}")

    if failed_suites:
        print(f"{RED}TESTS    {total_ok} ok, {total_fail} fehlgeschlagen in {len(failed_suites)} Suite(n){RESET}")
        for suite, out in failed_suites:
            print(f"\n{DIM}--- {suite.name} ---{RESET}")
            for line in out.splitlines()[-40:]:
                print("  " + line)
        return False

    print(f"{GREEN}TESTS    {total_ok} ok in {len(suites)} Suite(n){RESET}")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--compile", action="store_true", help="nur Syntaxpruefung")
    ap.add_argument("--tests", action="store_true", help="nur Unit-Tests")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    run_compile = args.compile or not args.tests
    run_tests = args.tests or not args.compile

    results = []
    if run_compile:
        results.append(check_compile(args.verbose))
    if run_tests:
        results.append(check_tests(args.verbose))

    if any(r is False for r in results):
        sys.exit(1)
    if all(r is None for r in results):
        print(f"{YELLOW}Nichts geprueft.{RESET}")
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
