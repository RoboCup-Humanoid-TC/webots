local function islist (t)
	local itemcount = 0
	local last_type = nil
	for k,v in pairs(t) do
		itemcount = itemcount + 1
		if last_type == nil then
			last_type = type(v)
		end

		if type(v) ~= last_type or (type(v) ~= "string" and type(v) ~= "number" and type(v) ~= "boolean" and type(v) ~= "table") then
			return false
		end
	
		last_type = type(v)
	end

	if itemcount ~= #t then
		return false
	end

	return true
end

local function cmp_alphanumeric (a, b)
	if (type(a) == "string" and type(b) == "string" or type(a) == "number" and type(b) == "number") then
		return a < b
	else
		return type(a) < type(b)
	end
end

local function gen_ordered_indices (t)
	local ordered_indices = {}
	for k in pairs (t) do
		table.insert (ordered_indices, k)
	end
	table.sort (ordered_indices, cmp_alphanumeric)
	return ordered_indices
end

local function ordered_next (t, state)
	if state == nil then
		t.__orderedIndices = gen_ordered_indices (t)
		local key = t.__orderedIndices[1]
		return key, t[key]
	end

	local key = nil
	for i = 1, table.getn(t.__orderedIndices) do
		if t.__orderedIndices[i] == state then
			key = t.__orderedIndices[i + 1]
		end
	end

	if key then
		return key, t[key]
	end

	t.__orderedIndices = nil
	return
end

local function ordered_pairs (t)
	return ordered_next, t, nil
end

function serialize (o, tabs, sorted)
  local result = ""
	local pairs_func = pairs
	if sorted then
		pairs_func = ordered_pairs
	end
  
  if tabs == nil then
    tabs = ""
  end

  if type(o) == "number" then
    result = result .. tostring(o)
  elseif type(o) == "boolean" then
    result = result .. tostring(o)
  elseif type(o) == "string" then
		result = result .. "\"" .. o .. "\""
	elseif type(o) == "table" and islist(o) then
		result = result .. "{"
		local last_was_subtable = false
		for i,v in ipairs(o) do
			last_was_subtable = false
			if type(v) == "table" then
				last_was_subtable = true
				result = result .. "\n" .. tabs .. "  " .. serialize (v, tabs .. "  ", sorted) .. ","
			elseif type(v) == "string" then
				result = result .. " \"" .. v .. "\","
			else
				result = result .. " " .. tostring (v) .. ","
			end
		end
		if last_was_subtable then
			result = result .. "\n" .. tabs
		end
		result = result .. "}"
  elseif type(o) == "table" then
   if o.dont_serialize_me then
      return "{}"
    end
    result = result .. "{\n"
    for k,v in pairs_func(o) do
      if type(v) ~= "function" then
        -- make sure that numbered keys are properly are indexified
        if type(k) == "number" then
				  if type(v) == "number" then
						result = result .. tabs .. "  [" .. tostring(k) .. "] = " .. tostring(v) .. ",\n"
					else
						result = result .. tabs .. "  [" .. tostring(k) .. "] = " .. serialize (v, tabs .. "  ", sorted) .. ",\n"
					end
        else
					result = result .. tabs .. "  " .. tostring (k) .. " = " .. serialize(v, tabs .. "  ", sorted) .. ",\n"
        end
      end
    end
    result = result .. tabs .. "}"
  else
    print ("not serializing entry of type " .. type(o) )
  end
  return result
end

return serialize
