--- Test de recepción BLE usando robotito_ble.
--- Referencia: test_ble.lua del proyecto.

-- Segundos que el test escucha antes de terminar
local TEST_DURATION = 600

-- Leer nombre del dispositivo desde NVS (igual que test_ble.lua)
nvs_read = nvs_read or function(namespace, key, default)
  return nvs.exists(namespace, key) and nvs.read(namespace, key) or default
end

local name = nvs_read("ble", "name")
          or nvs_read("wifi", "ssid", "robotito" .. tostring(id or ''))

print('bt_scan_test.lua', 'Iniciando BLE con nombre', name)

local rble = require('robotito_ble')
rble.init(name)


local ok, err = rble.mapper_init()
if not ok then
    print("Error al inicializar mapper:", err)
    return
end

-- local pairs_map = {
--     {1, 10},   -- physical 1  -> logical 10
--     {2, 20},   -- physical 2  -> logical 20
--     {3, 30},   -- physical 3  -> logical 30
--     {5,  1},   -- physical 5  -> logical 1
-- }

-- local ok, err = rble.mapper_set_pairs(pairs_map)
-- if not ok then
--     print("Error al guardar pares:", err)
-- else
--     print("Pares guardados OK")
-- end
-- tmr.sleep(5)

-- --------------------------------------------------------------------------
-- Contadores para el resumen final
-- --------------------------------------------------------------------------
local total_bytes    = 0
local total_lines    = 0
local total_packets  = 0

-- --------------------------------------------------------------------------
-- Callback: se invoca cada vez que llegan bytes (cualquier cantidad)
-- --------------------------------------------------------------------------
local arrived = function(s)
  total_bytes   = total_bytes + #s
  total_packets = total_packets + 1
  print('bt_scan_test.lua', 'arrived', #s, 'bytes | acum:', total_bytes, 'bytes')
end

local function parse_pairs(input)
    local pairs_map = {}
    for pair_str in input:gmatch("[^;]+") do
        local physical, logical = pair_str:match("(%d+),(%d+)")
        if physical and logical then
            table.insert(pairs_map, {tonumber(physical), tonumber(logical)})
        end
    end
    return pairs_map
end

-- --------------------------------------------------------------------------
-- Callback: se invoca cuando llega una línea completa (terminada en \n)
-- --------------------------------------------------------------------------
local on_line = function(s)
  total_lines = total_lines + 1
  print('bt_scan_test.lua', 'linea recibida:', s)
  print("LINE:", s)
  -- local p = parse_pairs(s)
  -- rble.mapper_set_pairs(p)

  -- Responder con eco para verificar que el canal TX también funciona
  local response = 'echo:' .. s .. '\n'
  rble.send(response)
  print('bt_scan_test.lua', 'eco enviado:', response:gsub('\n', '\\n'))
end

local on_connect = function(mac)
  print('bt_scan_test.lua', 'cliente conectado, MAC:', mac)
end

local on_disconnect = function(reason)
  print('bt_scan_test.lua', 'cliente desconectado, razon:', reason)
end

local function to_hex(bytes)
  local t = {}
  for i = 1, #bytes do
    t[#t + 1] = string.format("%02X", bytes:byte(i))
  end
  return table.concat(t, " ")
end

local function parse_cfg_pairs(bytes)
  local count = bytes:byte(1) or 0
  local pairs = {}
  local expected = 1 + (count * 2)
  if #bytes < expected then
    return nil, string.format("cfg len corto: %d < %d", #bytes, expected)
  end
  local idx = 2
  for i = 1, count do
    local physical = bytes:byte(idx)
    local logical = bytes:byte(idx + 1)
    pairs[#pairs + 1] = {physical, logical}
    idx = idx + 2
  end
  return pairs
end

local function parse_seq_list(bytes)
  local count = bytes:byte(1) or 0
  local seqs = {}
  local idx = 2
  for i = 1, count do
    if idx > #bytes then break end
    local len = bytes:byte(idx)
    idx = idx + 1
    local seq = {}
    for j = 1, len do
      if idx > #bytes then break end
      seq[#seq + 1] = bytes:byte(idx)
      idx = idx + 1
    end
    seqs[#seqs + 1] = seq
  end
  return seqs
end

local function on_mapper_cfg(bytes)
  print("mapper cfg bytes:", #bytes, to_hex(bytes))
  local pairs, err = parse_cfg_pairs(bytes)
  if not pairs then
    print("mapper cfg parse error:", err)
    return
  end
  local ok, err2 = rble.mapper_set_pairs(pairs)
  if not ok then
    print("mapper_set_pairs error:", err2)
  else
    print("mapper_set_pairs OK, pairs:", #pairs)
  end
end

local function on_mapper_seq(bytes)
  print("mapper seq bytes:", #bytes, to_hex(bytes))
  local seqs = parse_seq_list(bytes)
  if #seqs == 0 then
    local ok, err = rble.mapper_set_sequence({})
    if not ok then
      print("mapper_set_sequence error:", err)
    else
      print("mapper_set_sequence OK (vacía)")
    end
    return
  end
  -- Lua API solo soporta una secuencia; usamos la primera.
  local ok, err = rble.mapper_set_sequence(seqs[1])
  if not ok then
    print("mapper_set_sequence error:", err)
  else
    print("mapper_set_sequence OK, len:", #seqs[1], "seqs recibidas:", #seqs)
  end
end

-- --------------------------------------------------------------------------
-- Registrar callbacks
-- --------------------------------------------------------------------------
rble.set_rcv_callback(arrived)
rble.set_line_callback(on_line)
rble.set_connect_callback(on_connect)
rble.set_disconnect_callback(on_disconnect)
rble.set_mapper_cfg_callback(on_mapper_cfg)
rble.set_mapper_seq_callback(on_mapper_seq)


print('bt_scan_test.lua', 'Escuchando durante', TEST_DURATION, 'segundos...')
print('bt_scan_test.lua', '(cada linea recibida sera respondida con eco)')

-- --------------------------------------------------------------------------
-- Loop de espera: imprime un "heartbeat" cada 5 s para saber que el
-- script sigue corriendo, igual que el loop de envío de test_ble.lua
-- --------------------------------------------------------------------------

for i = 1, TEST_DURATION / 5 do
  tmr.sleep(5)
  print('bt_scan_test.lua', 'tick', i,
        '| bytes:', total_bytes,
        '| lineas:', total_lines,
        '| paquetes:', total_packets)
  
  -- Mostrar pares guardados (opcional)
  -- local saved_pairs = rble.mapper_get_pairs()
  -- for j, pair in ipairs(saved_pairs) do
  --     print(string.format("  par %d: physical=%d -> logical=%d", j, pair[1], pair[2]))
  -- end
  
  -- Mostrar secuencias guardadas para verificar carga
  local saved_seq = rble.mapper_get_sequence()
  if #saved_seq > 0 then
    local seq_str = table.concat(saved_seq, ", ")
    print('bt_scan_test.lua', 'Secuencia guardada:', seq_str)
  else
    print('bt_scan_test.lua', 'Sin secuencia guardada')
  end
end

-- --------------------------------------------------------------------------
-- Resumen final
-- --------------------------------------------------------------------------
print('bt_scan_test.lua', '--- RESUMEN ---')
print('bt_scan_test.lua', 'Total paquetes :', total_packets)
print('bt_scan_test.lua', 'Total bytes     :', total_bytes)
print('bt_scan_test.lua', 'Total lineas    :', total_lines)
print('bt_scan_test.lua', 'Test finalizado.')
