--[[
Where AutoDrive's per-vehicle data goes, and why it must not depend on the folder name.

The data lives under the plain node "vehicles.vehicle(N).AutoDrive". The schema says so
(Specialization.lua:139-149, literals), and both load paths build it literally
(Specialization.lua onPostLoad, AutoDriveVehicleData.lua:78). Only the two SAVE paths went the other
way round: they took the key the game hands a specialization -
"vehicles.vehicle(N).<modName>.<SpecName>" - and cut the mod name out with string.gsub against a
hardcoded "FS25_AutoDrive.…".

On any install whose folder is not called FS25_AutoDrive, that pattern matches nothing. The write
then lands on an unregistered path, XMLFile:setValue does nothing at all (fs25src/xml/XMLFile.lua),
and every AutoDrive setting on every vehicle is gone at the next save. The read side, being
name-free, keeps working perfectly - so the session it happens in looks completely normal and the
loss only shows up the next time the savegame is opened.

Two further traps, both of which the obvious fix walks into:

  string.gsub takes a Lua PATTERN. The dot in "FS25_AutoDrive.AutoDrive" is the any-character
  wildcard, not a literal dot.

  "FS25_AutoDrive.AutoDrive" is a PREFIX of "FS25_AutoDrive.AutoDriveVehicleData". A prefix-matching
  rewrite turns the vehicle data key into ".AutoDriveVehicleData" - also unregistered, same silent
  loss, and now in the one place a player would never look.

So the tests below check the renamed case explicitly, not just the current one.
]]

lu = require('luaunit')
require('test-setup')
require('AutoDrive')

TestSavegameNode = {}

function TestSavegameNode:setUp()
    TestSetup.reset()
end

--- The shape the game actually passes. Two specializations, one shared node.
local function specKey(modName, specSuffix)
    return 'vehicles.vehicle(3).' .. modName .. '.' .. specSuffix
end

------------------------------------------------------------------------------------------------------------------------
--- The node is the same one the schema and the load path use
------------------------------------------------------------------------------------------------------------------------

function TestSavegameNode:testTheMainSpecWritesWhereTheSchemaSaysItShould()
    local key = specKey('FS25_AutoDrive', 'AutoDrive')
    lu.assertEquals(AutoDrive.getSavegameNodeKey(key, 'FS25_AutoDrive.AutoDrive'),
            'vehicles.vehicle(3).AutoDrive')
end

--- The vehicle data specialization shares the SAME node - it is not "AutoDriveVehicleData".
--- Getting this wrong sends parkDestination to a path nobody registered and nobody reads.
function TestSavegameNode:testTheVehicleDataSpecSharesTheSameNode()
    local key = specKey('FS25_AutoDrive', 'AutoDriveVehicleData')
    lu.assertEquals(AutoDrive.getSavegameNodeKey(key, 'FS25_AutoDrive.AutoDriveVehicleData'),
            'vehicles.vehicle(3).AutoDrive')
end

--- The two must agree, or a vehicle's settings and its park destination end up in different places.
function TestSavegameNode:testBothSpecializationsLandOnOneNode()
    lu.assertEquals(
            AutoDrive.getSavegameNodeKey(specKey('FS25_AutoDrive', 'AutoDrive'), 'FS25_AutoDrive.AutoDrive'),
            AutoDrive.getSavegameNodeKey(specKey('FS25_AutoDrive', 'AutoDriveVehicleData'), 'FS25_AutoDrive.AutoDriveVehicleData'))
end

------------------------------------------------------------------------------------------------------------------------
--- The renamed install - the case that was silently losing data
------------------------------------------------------------------------------------------------------------------------

--- A folder called anything else at all. This is what a merged mod would be, and also what any
--- player who renames the zip already has.
function TestSavegameNode:testARenamedFolderStillWritesToTheRightNode()
    for _, modName in ipairs({ 'FS25_Courseplay', 'FS25_CourseDrive', 'AutoDrive_test', 'X' }) do
        lu.assertEquals(
                AutoDrive.getSavegameNodeKey(specKey(modName, 'AutoDrive'), modName .. '.AutoDrive'),
                'vehicles.vehicle(3).AutoDrive',
                modName .. ': settings would be written to a path the schema does not know')
        lu.assertEquals(
                AutoDrive.getSavegameNodeKey(specKey(modName, 'AutoDriveVehicleData'), modName .. '.AutoDriveVehicleData'),
                'vehicles.vehicle(3).AutoDrive',
                modName .. ': the park destination would be lost')
    end
end

--- The prefix trap, stated as a test because the obvious fix falls into it: the main spec's name is
--- a prefix of the vehicle data spec's name, so anything that matches on a prefix converts
--- "….FS25_AutoDrive.AutoDriveVehicleData" into "….AutoDriveVehicleData" and calls it a day.
function TestSavegameNode:testTheVehicleDataKeyIsNotTruncatedToItsPrefix()
    local converted = AutoDrive.getSavegameNodeKey(
            specKey('FS25_AutoDrive', 'AutoDriveVehicleData'), 'FS25_AutoDrive.AutoDriveVehicleData')
    lu.assertNil(converted:find('VehicleData', 1, true),
            'a prefix match left the vehicle data suffix behind; that path is not registered either')
end

--- The pattern trap: a dot in a Lua pattern matches any character. A name-based match would accept
--- a key that merely looks similar, and quietly write somebody else's data into our node.
function TestSavegameNode:testMatchingIsLiteralNotAPattern()
    -- The character that differs has to sit where the pattern would have a DOT, or the probe proves
    -- nothing: an underscore is a literal in a Lua pattern too, so changing FS25_ to FS25X would
    -- fail to match either way. Here the dot between the mod name and the spec name is an X.
    local key = 'vehicles.vehicle(3).FS25_AutoDriveXAutoDrive'
    lu.assertEquals(AutoDrive.getSavegameNodeKey(key, 'FS25_AutoDrive.AutoDrive'), key,
            'the mod name was matched as a pattern, so a key that merely looks similar was converted')
end

------------------------------------------------------------------------------------------------------------------------
--- Failing loudly rather than quietly
------------------------------------------------------------------------------------------------------------------------

--- A key that does not end in the specialization name is a shape we do not understand. The whole
--- point of this function is that the bug it replaces was silent, so an unconvertible key has to
--- say so rather than pass through and lose the data on the next save.
function TestSavegameNode:testAnUnrecognisedKeyIsReported()
    local errors = {}
    local realError = Logging.error
    Logging.error = function(fmt, ...) table.insert(errors, string.format(fmt, ...)) end

    local key = 'vehicles.vehicle(3).SomethingElse'
    local converted = AutoDrive.getSavegameNodeKey(key, 'FS25_AutoDrive.AutoDrive')

    Logging.error = realError
    lu.assertEquals(converted, key, 'an unconvertible key must be left alone, not mangled')
    lu.assertEquals(#errors, 1, 'and it has to be logged - silence here is the original bug')
    lu.assertNotNil(errors[1]:find('SomethingElse', 1, true))
end

function TestSavegameNode:testNilInputsAreHarmless()
    lu.assertNil(AutoDrive.getSavegameNodeKey(nil, 'FS25_AutoDrive.AutoDrive'))
    lu.assertEquals(AutoDrive.getSavegameNodeKey('a.b', nil), 'a.b')
end

------------------------------------------------------------------------------------------------------------------------
--- Wiring: nothing may go back to a hardcoded name
------------------------------------------------------------------------------------------------------------------------

local function readSource(path)
    local file = assert(io.open('../../' .. path, 'r'), 'cannot read ' .. path)
    local text = file:read('*a')
    file:close()
    return (text:gsub('%-%-[^\n]*', ''))
end

--- The two save paths, and a guard against the literal creeping back in anywhere.
function TestSavegameNode:testNoSavePathSpellsTheModNameOut()
    for _, case in ipairs({
        { path = 'scripts/Specialization.lua', spec = 'AutoDrive.ADSpecName' },
        { path = 'scripts/Utils/AutoDriveVehicleData.lua', spec = 'AutoDrive.ADVDSpecName' },
    }) do
        local text = readSource(case.path)
        lu.assertNotNil(text:find('AutoDrive.getSavegameNodeKey(key, ' .. case.spec .. ')', 1, true),
                case.path .. ' no longer derives the savegame node from its own specialization name')
        lu.assertNil(text:find('"FS25_AutoDrive.AutoDrive', 1, true),
                case.path .. ' spells the mod folder name out again; a renamed install loses its data')
    end
end

--- The specialization names themselves have to stay name-derived, or the fix above has nothing
--- correct to work from.
function TestSavegameNode:testTheSpecNamesComeFromTheLoadedModName()
    local text = readSource('register.lua')
    for _, spec in ipairs({ 'ADSpecName', 'ADVDSpecName', 'ADPDSpecName' }) do
        lu.assertNotNil(text:find('AutoDrive.' .. spec .. ' = g_currentModName', 1, true),
                spec .. ' is no longer built from the actually loaded mod name')
    end
end

--- And the node itself must stay the literal the schema and the load path use. Deriving it from
--- g_currentModName would move it, which is the same data loss from the other direction.
function TestSavegameNode:testTheNodeStaysALiteral()
    lu.assertEquals(AutoDrive.SAVEGAME_NODE, 'AutoDrive')
    local spec = readSource('scripts/Specialization.lua')
    lu.assertNotNil(spec:find('"vehicles.vehicle(?).AutoDrive#mode"', 1, true),
            'the schema no longer declares the literal node these keys are built for')
end

os.exit(lu.LuaUnit.run())
