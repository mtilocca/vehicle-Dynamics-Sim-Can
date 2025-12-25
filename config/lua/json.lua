-- config/lua/json.lua
-- Minimal JSON decoder for numeric + table data

local json = {}

function json.decode(str)
    -- VERY minimal JSON → Lua table converter
    -- Assumptions:
    --  - numbers, strings, arrays, objects
    --  - no comments
    --  - no trailing commas

    local f, err = load("return " ..
        str
        :gsub("null", "nil")
        :gsub("true", "true")
        :gsub("false", "false")
    )

    if not f then
        error("JSON decode error: " .. err)
    end

    return f()
end

function json.load_file(path)
    local f = io.open(path, "r")
    if not f then
        error("Cannot open JSON file: " .. path)
    end

    local content = f:read("*a")
    f:close()
    return json.decode(content)
end

return json
