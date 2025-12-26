-- config/lua/json.lua
-- Simple JSON decoder for scenario files

local json = {}

local function skip_whitespace(str, pos)
  while pos <= #str do
    local c = str:sub(pos, pos)
    if c ~= ' ' and c ~= '\t' and c ~= '\n' and c ~= '\r' then
      break
    end
    pos = pos + 1
  end
  return pos
end

local function decode_value(str, pos)
  pos = skip_whitespace(str, pos)
  local c = str:sub(pos, pos)
  
  -- null
  if str:sub(pos, pos+3) == 'null' then
    return nil, pos + 4
  end
  
  -- true
  if str:sub(pos, pos+3) == 'true' then
    return true, pos + 4
  end
  
  -- false
  if str:sub(pos, pos+4) == 'false' then
    return false, pos + 5
  end
  
  -- number
  if c == '-' or (c >= '0' and c <= '9') then
    local i = pos
    while i <= #str do
      local ch = str:sub(i, i)
      if not (ch == '-' or ch == '+' or ch == '.' or ch == 'e' or ch == 'E' or (ch >= '0' and ch <= '9')) then
        break
      end
      i = i + 1
    end
    return tonumber(str:sub(pos, i-1)), i
  end
  
  -- string
  if c == '"' then
    local i = pos + 1
    local result = {}
    while i <= #str do
      local ch = str:sub(i, i)
      if ch == '"' then
        return table.concat(result), i + 1
      elseif ch == '\\' then
        i = i + 1
        local esc = str:sub(i, i)
        if esc == 'n' then table.insert(result, '\n')
        elseif esc == 't' then table.insert(result, '\t')
        elseif esc == 'r' then table.insert(result, '\r')
        else table.insert(result, esc) end
        i = i + 1
      else
        table.insert(result, ch)
        i = i + 1
      end
    end
    error("Unterminated string")
  end
  
  -- array
  if c == '[' then
    local result = {}
    pos = pos + 1
    pos = skip_whitespace(str, pos)
    if str:sub(pos, pos) == ']' then
      return result, pos + 1
    end
    while true do
      local val, newpos = decode_value(str, pos)
      table.insert(result, val)
      pos = skip_whitespace(str, newpos)
      local c = str:sub(pos, pos)
      if c == ']' then return result, pos + 1 end
      if c == ',' then pos = pos + 1 else error("Expected ',' or ']'") end
    end
  end
  
  -- object
  if c == '{' then
    local result = {}
    pos = pos + 1
    pos = skip_whitespace(str, pos)
    if str:sub(pos, pos) == '}' then
      return result, pos + 1
    end
    while true do
      pos = skip_whitespace(str, pos)
      local key, newpos = decode_value(str, pos)
      pos = skip_whitespace(str, newpos)
      if str:sub(pos, pos) ~= ':' then error("Expected ':'") end
      pos = pos + 1
      local val, newpos2 = decode_value(str, pos)
      result[key] = val
      pos = skip_whitespace(str, newpos2)
      local c = str:sub(pos, pos)
      if c == '}' then return result, pos + 1 end
      if c == ',' then pos = pos + 1 else error("Expected ',' or '}'") end
    end
  end
  
  error("Unexpected character: " .. c)
end

function json.decode(str)
  local val, pos = decode_value(str, 1)
  return val
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