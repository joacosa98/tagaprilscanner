# AprilTag Scanner - Robotito Thesis Project

Sistema de detección y mapeo de AprilTags para robots móviles con integración ESP32 vía Bluetooth Low Energy (BLE).

## 🚀 Quick Start

### Prerequisites

- **Browser**: Chrome 89+ o Edge 89+ (requiere Web Bluetooth API)
- **Python 3** (para servidor local)
- **WASM Files**: Descargá los archivos necesarios (ver instalación)

### Installation

1. **Descargar archivos WASM**
   
   Necesitás los archivos del compilador AprilTag WASM desde [apriltag-js-standalone](https://github.com/arenaxr/apriltag-js-standalone):
   
   ```bash
   # Colocá estos archivos en la raíz del proyecto:
   - apriltag_wasm.js
   - apriltag_wasm.wasm
   ```

2. **Servir el proyecto localmente**
   
   ```bash
   cd tagaprilscanner
   python -m http.server 8000
   ```

3. **Abrir en el navegador**
   
   ```
   http://localhost:8000
   ```
   
   ⚠️ **Importante**: Debe ser `localhost` o HTTPS. Camera y BLE no funcionan sobre HTTP inseguro.

---

## 📁 Estructura del Proyecto

```
tagaprilscanner/
├── index.html                  # Detector principal de AprilTags (cámara web)
├── apriltag-mapper.html        # Configurador de rutas y estaciones
├── apriltag-map-builder.html  # Constructor de mapeo físico ↔ lógico
├── apriltag.js                 # Wrapper con Web Workers (Comlink)
├── apriltag-standalone.js      # Wrapper standalone (sin workers)
├── apriltag_wasm.js            # WASM glue code (descargarlo)
├── apriltag_wasm.wasm          # Módulo WASM compilado (descargarlo)
└── c/                          # Firmware ESP32
    ├── huskylens_mapper.c      # Implementación BLE del mapper
    ├── huskylens_mapper.h      # Header del mapper
    └── huskylens_lua.c         # Bindings Lua para HuskyLens
```

---

## 🎯 Componentes y Uso

### 1. **index.html** - Detector Principal

**Propósito**: Detecta AprilTags en tiempo real usando la cámara web.

**Características**:
- Detección en tiempo real (~15 FPS)
- Visualización con overlay gráfico
- Mapeo de IDs (físico → lógico)
- Modo embebido para integración en iframe

**Uso**:
```html
<!-- Embeber en otra página -->
<iframe src="index.html?embedded=1"></iframe>

<!-- Recibir detecciones vía postMessage -->
<script>
window.addEventListener('message', (e) => {
  if (e.data.type === 'apriltag-detected') {
    console.log('Tag detected:', e.data.id);
  }
});
</script>
```

**Configuración de mapeo**:
```javascript
// En index.html, línea ~457
const ID_MAPPING = {
  3: 5,     // Tag físico 3 → Mostrar como 5
  10: 20,   // Tag físico 10 → Mostrar como 20
  // Agregar más mapeos según necesites
};
```

---

### 2. **apriltag-mapper.html** - Configurador de Rutas

**Propósito**: Configurar estaciones, secuencias de recorrido y enviarlas al ESP32 vía BLE.

**Características**:
- Gestión de estaciones con nombres personalizados
- Múltiples secuencias de recorrido
- Validación de IDs (reservados: 1, 2, 3, 4)
- Monitor de detecciones en tiempo real
- Lectura de tags vía cámara integrada
- Persistencia en NVS del ESP32

**Flujo de trabajo**:
1. **Conectar al ESP32**: Click en "Conectar ESP32"
2. **Crear secuencia**: Click en "Nueva secuencia"
3. **Agregar estaciones**: Click en "Agregar estación"
4. **Asignar tags físicos**: Click en "Leer tag" para cada estación
5. **Validar**: Revisar que no haya errores
6. **Enviar**: Click en "Enviar Mapa + Secuencias"

**Tags Reservados**:
- `1-4`: Reservados para control del sistema (inicio, fin, checkpoints)

---

### 3. **apriltag-map-builder.html** - Constructor de Mapeo

**Propósito**: Crear mapeo simple físico ↔ lógico sin secuencias complejas.

**Características**:
- Mapeo directo de IDs físicos a lógicos
- Lectura de tags por cámara
- Envío y carga desde ESP32
- Validación de duplicados

**Uso simple**:
1. Conectar ESP32
2. "Agregar mapeo"
3. "Leer tag" para capturar ID físico
4. Asignar ID lógico manualmente
5. "Enviar Mapa"

---

## 🔌 Integración ESP32 (BLE)

### Protocolo BLE

**Device Name**: `AprilTag_Mapper`

**Service UUID**: `12561234-5678-1234-1234-56789abc0000`

### Características BLE

| Characteristic | UUID | Tipo | Descripción |
|----------------|------|------|-------------|
| Config Write | `...0001` | Write | Escribir mapa (buffer binario) |
| Config Read | `...0002` | Read | Leer mapa guardado |
| Detection Notify | `...0003` | Notify | Notificaciones de detecciones |
| Sequence Write | `...0004` | Write | Escribir secuencias |
| Sequence Read | `...0005` | Read | Leer secuencias |

### Formato de Datos

**Mapa (Config Write/Read)**:
```
[count:1] + count * [physical_id:1, logical_id:1]

Ejemplo: 3 mapeos
[0x03, 0x0A, 0x14, 0x0F, 0x1E, 0x05, 0x0A]
       │     │     │     │     │     │     └─ logical: 10
       │     │     │     │     │     └─ physical: 5
       │     │     │     │     └─ logical: 30
       │     │     │     └─ physical: 15
       │     │     └─ logical: 20
       │     └─ physical: 10
       └─ count: 3
```

**Secuencias (Sequence Write/Read)**:
```
[seq_count:1] + seq_count * ([len:1] + len * [station_id:1])

Ejemplo: 2 secuencias
[0x02, 0x03, 0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04]
 │     │     │     │     │     │     │     │     │     └─ station 4
 │     │     │     │     │     │     │     │     └─ station 3
 │     │     │     │     │     │     │     └─ station 2
 │     │     │     │     │     │     └─ station 1
 │     │     │     │     │     └─ len: 4
 │     │     │     │     └─ station 3
 │     │     │     └─ station 2
 │     │     └─ station 1
 │     └─ len: 3
 └─ count: 2 sequences
```

### Implementación ESP32

Ver archivos en `c/`:
- `huskylens_mapper.c`: Implementación completa del servidor BLE
- `huskylens_mapper.h`: API pública

**Funciones principales**:
```c
bool huskylens_mapper_start(huskylens_t *husky);
void huskylens_mapper_stop(void);
uint8_t huskylens_remap_april_tag(uint8_t physical_id);
bool huskylens_mapper_set_pairs(const uint8_t *pairs, uint8_t count);
```

---

## 🛠️ Configuración Avanzada

### Detector WASM

**Opciones del detector** (en `apriltag.js` y `apriltag-standalone.js`):

```javascript
this._opt = {
  quad_decimate: 2.0,        // Factor de decimación (mayor = más rápido, menos preciso)
  quad_sigma: 0.0,           // Blur gaussiano (0 = sin blur)
  nthreads: 1,               // Threads (sin efecto en WASM)
  refine_edges: 1,           // Refinamiento de bordes (1 = activado)
  max_detections: 0,         // Máximo de detecciones (0 = ilimitado)
  return_pose: 0,            // Retornar pose 3D (require calibración)
  return_solutions: 0        // Retornar soluciones alternativas
};
```

### Calibración de Cámara

Para estimación de pose 3D:

```javascript
detector.set_camera_info(fx, fy, cx, cy);
detector.set_tag_size(tagId, sizeInMeters);
detector.set_return_pose(1);
```

Donde:
- `fx, fy`: Focal length en píxeles
- `cx, cy`: Centro óptico (generalmente width/2, height/2)
- `sizeInMeters`: Tamaño físico del tag

---

## 🐛 Troubleshooting

### Error: "Camera Access Denied"
**Solución**: Permitir acceso a cámara en configuración del navegador. Debe ser HTTPS o localhost.

### Error: "WASM Files Not Found"
**Solución**: Descargar `apriltag_wasm.js` y `apriltag_wasm.wasm` y colocarlos en la raíz.

### Error: "Browser Not Supported"
**Solución**: Usar Chrome 89+ o Edge 89+. Safari no soporta Web Bluetooth.

### BLE: "Connection Timeout"
**Solución**: 
- Verificar que ESP32 esté encendido y en rango
- Verificar que el dispositivo se llame "AprilTag_Mapper"
- Reiniciar ESP32 y volver a intentar

### BLE: "UUID inválido"
**Solución**: Verificar que los UUIDs en la configuración avanzada coincidan con el firmware ESP32.

### Tags no se detectan
**Solución**:
- Verificar iluminación (evitar sombras y reflejos)
- Asegurar que el tag esté perpendicular a la cámara
- Verificar distancia (recomendado: 30cm - 2m)
- Limpiar superficie del tag

---

## 📊 Limitaciones y Restricciones

| Aspecto | Límite | Nota |
|---------|--------|------|
| Estaciones | 50 | `HUSKYLENS_MAP_MAX` en C |
| Secuencias | 10 | `HUSKYLENS_SEQ_MAX_SEQS` |
| IDs por secuencia | 50 | `HUSKYLENS_SEQ_MAX` |
| Rango de IDs | 0-255 | 1 byte por ID |
| Tags reservados | 1-4 | Control del sistema |
| MTU BLE | 23 bytes | Paquete mínimo garantizado |

---

## 🔒 Seguridad

### Contexto Seguro
- ✅ HTTPS
- ✅ localhost / 127.0.0.1
- ❌ HTTP en red local
- ❌ IP pública sin HTTPS

### Validaciones Implementadas
- IDs en rango 0-255
- Validación de UUIDs BLE
- Timeout de conexión (15s)
- Validación de duplicados
- Sanitización de inputs

---

## 🧪 Testing

### Casos de Prueba Recomendados

1. **Compatibilidad de navegador**
   - ✅ Chrome 89+
   - ✅ Edge 89+
   - ⚠️ Firefox (requiere habilitar flag)
   - ❌ Safari (sin Web Bluetooth)

2. **Escenarios de error**
   - Cámara denegada
   - BLE desconectado durante envío
   - IDs inválidos (negativos, >255)
   - Secuencias con duplicados
   - Estaciones sin tag asignado

3. **Edge cases**
   - 50 estaciones (límite)
   - Múltiples tags simultáneos en frame
   - Tags parcialmente ocluidos
   - Iluminación extrema

---

## 📝 Changelog

### v1.0.0 (Actual)
- ✅ Detección en tiempo real con WASM
- ✅ BLE ESP32 con persistencia NVS
- ✅ Validación completa de datos
- ✅ Toast notifications
- ✅ Progress indicators
- ✅ Secure context checks
- ✅ UUID validation
- ✅ Connection timeouts
- ✅ Formatted error messages

---

## 🤝 Contribución

Este es un proyecto de tesis. Para consultas o colaboración, contactar al autor.

---

## 📄 Licencia

[TBD - Especificar licencia]

---

## 🙏 Agradecimientos

- [apriltag-js-standalone](https://github.com/arenaxr/apriltag-js-standalone) - WASM wrapper
- AprilTag library original - University of Michigan
- ESP-IDF framework - Espressif Systems

---

## 📬 Contacto

**Autor**: [Joaquín Sanson]  
**Universidad**: FING  
**Proyecto**: Tesis - Robotito Tag Scanner  
**Fecha**: Marzo 2026
