--[[
The translation files as data, checked the way code is checked.

Eighteen languages, contributed by as many people over years, and nothing ever looked at them
together. Three things can go wrong without anyone noticing until a player sees it:

  a key that exists in one language and not another, so that language falls back or shows the key;
  the same key twice in one file, where the second silently wins;
  and stray whitespace, which is invisible in a diff and doubled in the game.

The last one was live: AD_parkVehicle_selected ended with a space in fifteen of the eighteen files,
and its one caller formats it as "$l10n_AD_parkVehicle_selected; %s" - a space is already there - so
the message read "Als Parkplatz zugewiesen:  Hof 4".
]]

lu = require('luaunit')
require('test-setup')

local LANGUAGES = {
    'br', 'cs', 'ct', 'cz', 'da', 'de', 'ea', 'en', 'es',
    'fr', 'hu', 'it', 'nl', 'pl', 'pt', 'ru', 'tr', 'uk',
}

--- name -> text for one language, plus the keys in file order so duplicates can be spotted.
local function load(language)
    local path = '../../translations/translation_' .. language .. '.xml'
    local file = io.open(path, 'r')
    if file == nil then
        return nil, nil, path
    end
    local source = file:read('*a')
    file:close()

    local byKey, order = {}, {}
    for key, text in source:gmatch('<text%s+name="([^"]+)"%s+text="([^"]*)"') do
        order[#order + 1] = key
        byKey[key] = text
    end
    return byKey, order, path
end

TestTranslations = {}

function TestTranslations:setUp()
    TestSetup.reset()
    self.english = load('en')
    lu.assertNotNil(self.english, 'test setup: the English file has to be readable')
end

function TestTranslations:testEveryLanguageFileIsReadableAndNotEmpty()
    for _, language in ipairs(LANGUAGES) do
        local byKey, order, path = load(language)
        lu.assertNotNil(byKey, 'cannot read ' .. path)
        lu.assertTrue(#order > 100, path .. ' holds only ' .. #order .. ' entries')
    end
end

--- A key present in one language and missing in another shows up as the raw key in game.
function TestTranslations:testEveryLanguageHasExactlyTheEnglishKeys()
    for _, language in ipairs(LANGUAGES) do
        local byKey = load(language)
        local missing, extra = {}, {}
        for key in pairs(self.english) do
            if byKey[key] == nil then
                missing[#missing + 1] = key
            end
        end
        for key in pairs(byKey) do
            if self.english[key] == nil then
                extra[#extra + 1] = key
            end
        end
        table.sort(missing)
        table.sort(extra)
        lu.assertEquals(missing, {}, language .. ' is missing keys')
        lu.assertEquals(extra, {}, language .. ' has keys English does not')
    end
end

--- The second definition silently wins, so a duplicate is a translation nobody can see.
function TestTranslations:testNoLanguageDefinesAKeyTwice()
    for _, language in ipairs(LANGUAGES) do
        local _, order = load(language)
        local seen, dupes = {}, {}
        for _, key in ipairs(order) do
            if seen[key] then
                dupes[#dupes + 1] = key
            end
            seen[key] = true
        end
        table.sort(dupes)
        lu.assertEquals(dupes, {}, language .. ' defines a key more than once')
    end
end

--- Whitespace nobody can see in the file and everybody can see in the game.
function TestTranslations:testNoValueCarriesStrayWhitespace()
    for _, language in ipairs(LANGUAGES) do
        local byKey = load(language)
        local offenders = {}
        for key, text in pairs(byKey) do
            if text:match('^%s') or text:match('%s$') or text:match('  ') then
                offenders[#offenders + 1] = key
            end
        end
        table.sort(offenders)
        lu.assertEquals(offenders, {},
            language .. ' has values with leading, trailing or doubled spaces')
    end
end

--- The one that was actually wrong in fifteen files, stated as itself: its caller supplies the
--- separating space, so the value must not.
function TestTranslations:testTheParkMessageDoesNotDoubleItsSpace()
    for _, language in ipairs(LANGUAGES) do
        local byKey = load(language)
        local text = byKey['AD_parkVehicle_selected']
        lu.assertNotNil(text, language .. ' has no AD_parkVehicle_selected')
        lu.assertNil(text:match('%s$'),
            language .. ': the caller formats this as "...; %s", so a trailing space doubles it')
    end
end

os.exit(lu.LuaUnit.run())
