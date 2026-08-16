# Working on FS25_AutoDrive

A Farming Simulator 25 mod. Lua, no build step for the mod itself - the game loads the `.lua`
files directly out of the zip.

## The gate

```bash
python tools/check.py
```

Two independent checks, both must pass:

1. **COMPILE** - every `.lua` file compiled with the Luau compiler *the game itself uses*
   (`fs-luau-compile`). FS25 runs Luau, not Lua 5.4, and the two disagree about what is valid.
2. **TESTS** - every `tools/tests/test_*.lua`, run under Lua 5.4 as standalone luaunit suites.

The suite loads **real mod files**, not copies. That only works because the codebase deliberately
avoids Luau-only syntax, so the same source parses under both. Keep it that way: if a file stops
loading under Lua 5.4 the test that covers it silently stops existing.

`tools/make_release.py` builds `FS25_AutoDrive.zip`. It does **not** copy to the mods folder; that
is manual (`~/Documents/My Games/FarmingSimulator2025/mods/`). Build from a clean export
(`git archive HEAD | tar -x -C <tmp>`) if anything else might be touching the working tree.

## A green suite proves nothing on its own

This is the single most expensive lesson from this codebase. Repeatedly, a defect was found in
code that had tests, and the tests passed against the broken version. So:

**Every fix needs a test that fails when the fix is reverted, verified by reverting it.** One
mutation at a time, run the gate, confirm exactly that test goes red, restore. Automate it - a
small Python script that patches one string, runs `check.py`, restores, and reports which tests
turned red is worth writing every time.

Things that went wrong doing exactly this, all real:

- **A mutation that is not a reversion proves nothing.** Adding the old line back at the top while
  leaving the new one at the bottom left both in place; the gate stayed green and looked like a
  coverage gap that was not there.
- **Mutating a shared helper changes both sides of a comparison.** A sweep that compares production
  against a transcription of production will happily agree when you break the helper they share.
  Mutate the transcription instead.
- **Test the band where the rule actually binds.** A corner test placed 1 m from the target passed
  against the broken version, because the broken floor was `min(8, 2+d) = 3` there and did not bind.
  The measured failure was 6-17 m out.
- **A test can pass because the code under test never ran.** A `createDebugMarkers` test was green
  until the debug channel mask was set - the whole body was behind a channel check. Assert your
  setup reached the code (`lu.assertTrue(AutoDrive.getDebugChannelIsSet(...))`).
- **A mutation can hang instead of failing.** Removing one `removeValue` turned a merge loop
  infinite and killed a ten-minute run. Bound loops that consume from a list.

Prefer extracting a rule into its own named function over building a large fixture to reach it -
this codebase already does that (`stepAsideDistance`, `shouldReverseTo`, `approachSpeedLimit`,
`callSecondUnloaderFor`). A test that restates a rule cannot notice the rule changing; a test that
*drives* it can.

## Reading the engine instead of guessing

`D:\Coding\greluc\fs25\farmmanager\tools\fs-utils` (built binaries in `target/release/`):

| Tool | Use |
| --- | --- |
| `fs-luau-decompile` | FS25 `.l64` bytecode → readable Lua. Reads **straight out of the archive**. |
| `fs-luau-compile` | the syntax gate `check.py` uses |
| `fs-unpack` | extract a whole `.gar`/`.dlc` |
| `fs-xml-format` | pretty-print game XML |
| `fs-luajit-decompile` | FS19/FS22 bytecode (not FS25) |

```bash
# one engine script, no extraction needed
fs-luau-decompile.exe "<game>/dataS.gar/scripts/vehicles/specializations/FillUnit.l64" ./out
# a whole subtree
fs-luau-decompile.exe -r "<game>/dataS.gar/scripts/vehicles/" ./out
```

Game install: `D:\SteamLibrary\steamapps\common\Farming Simulator 25` (`dataS.gar` ~3.3 GB).

**Use this before asserting anything about Giants API semantics.** Several findings here turned on
exactly that: `getFillUnits` exists only because the `FillUnit` specialization registers it, which
is why vehicle types that do not inherit `baseFillable` answer `nil`.

Also on disk and worth reading rather than guessing: `<game>/sdk/xmlDoku/vehicleTypes.xml` (which
specializations each vehicle type inherits) and `<game>/data/vehicles/**` (what each machine
actually declares). GDN's own downloads are the Editor and exporter plugins plus documentation -
useful for i3d/XML work, but they contain **no bytecode decompiler**, so `fs-luau-decompile` is the
tool for reading scripts.

### What reading it has already settled

- **`math.sign` and `math.clamp` are C-level extensions**, not Lua. The engine's own scripts call
  `math.sign` ~170 times and define it nowhere. There is no `MathUtil.sign` - reaching for one raises.
- **`overlapBox`'s extents are local X in slot 7 (ex) and local Z in slot 9 (ez).** The engine's own
  calls (`PlacementUtil`, `TransportMission`) put half *width* in ex and half *length* in ez, with a
  true Y heading. AutoDrive's pathfinder rotates by `atan2(-dz, dx)` instead, which is that heading
  minus 90 degrees - so under that angle the long extent belongs in **ex**. Check which angle a call
  site uses before reading its slots.
- **`AIVehicleUtil.getDriveDirection` already normalises**, so the `math.acos(lz)` inside
  `driveInDirection` is safe for anything that came out of it.
- **`driveInDirection` reads `self.motor` and `self.cruiseControl.state` off the vehicle**, which no
  specialization puts there. AutoDrive assigning them before each call is a required shim, not
  clobbering.
- **`terrainDetailId` is the whole packed detail map** - ground type, stones, weeds and foliage all
  init on it. So `FSDensityMapUtil.getIsFieldAtWorldPos` (one density read, no masking) is *cheaper
  but not equivalent* to `getFieldDataAtWorldPosition`, which masks out the ground-type channels.
  AutoDrive uses the masked one deliberately; swapping it is a behaviour change, not an optimisation.
- **There is no spatial vehicle index.** `VehicleSystem` keeps a plain list, so scanning all vehicles
  is the only option and a cheap distance cull in front of the expensive work is the right shape.

## Lua and Luau traps that have actually bitten here

- **`%w` excludes underscores.** `CREATE_%w+` matched `CREATE_OFF` and stopped at the first
  underscore in `CREATE_SUB_PRIO_DUAL_TWOWAY`. Use `[%w_]+`.
- **`for k, v in someTable do` is not iteration** - it asks Lua to *call* the table, and raises.
  Needs `pairs`/`ipairs`. One such line killed the road-network fault check on its first tile.
- **Removing from an array while iterating it with `pairs` skips entries.** Walk backwards, or
  collect and remove afterwards.
- **`AutoDriveTON:timer(false)` resets `elapsedTime` to 0.** Anything that calls a helper which
  releases a timer and then ticks that same timer measures one frame, forever.
- **Stream bit widths are silent.** `streamWriteUIntN(id, value, 3)` truncates 9 to 1 without a
  word. Writer and reader must agree, and both must hold the largest value the constant list can
  reach - assert against the constants, not against a number.
- **`pairs` order over the mission vehicle list is stable until the list changes**, which it does
  when vehicles are bought or sold. Do not build a rule on "the first one found".

## Idioms this codebase relies on

- **Throttled scans**: `if ((g_updateLoopIndex + self.vehicle.id) % AutoDrive.PERF_FRAMES == 0)`.
  The `+ vehicle.id` spreads the fleet across frames. A test that does not put the loop index on
  that vehicle's frame gets the *cached* answer and every `assertFalse` passes for the wrong reason.
- **Two clocks**: `dt` is the frame delta passed to `update`; `g_time` is a timestamp. Do not mix.
- **Per-partner state, not one slot.** Twice now, state that needed one entry per vehicle was kept
  in a single field and silently did nothing: the give-up memory, then the clock that goes with it.
  If two of something can be in the way at once, it needs a table - weak-keyed, so a vehicle that
  leaves the game is not held alive by it.
- **Every frame must leave the vehicle with a drive command or a brake.** A state that falls through
  its branches issuing neither leaves the vehicle coasting, and `specialDrivingModule:update(dt)`
  is what applies a stop, so it has to be reached too.
- **Measure the whole rig, not `components[1]`.** That node is the tractor. A trailer swung across a
  road is found by walking `getAttachedImplements` for real node positions - a length along the
  vehicle's axis assumes a straight line the rig does not have.
- **Comments here carry reasoning, not description.** Many explain a defect that a change fixed and
  why the obvious alternative is worse. Before "simplifying" something odd-looking, read the comment
  three lines up - and when fixing something, write down what it cost, measured.

## What cannot be checked here

Nothing in this repo observes the mod in a running game. Frame timing, collision box behaviour
against real geometry, whether a manoeuvre actually completes on a real field - none of that is
covered by the gate, and several constants (`CORNER_LATERAL_ACCELERATION`, `ASKER_PASSING_CLEARANCE`)
are judgement rather than measurement. Say so plainly rather than reporting a green suite as if it
settled the question. The game log
(`~/Documents/My Games/FarmingSimulator2025/log.txt`, all debug channels on) is the only evidence
from the real thing - and it is only useful where the code actually prints something, which is worth
checking before promising an answer from it.
