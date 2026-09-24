#pragma once
// =============================================================================
// Plantilla de credenciales. NO poner datos reales acá, este archivo se versiona.
//
// Para configurar tu placa:
//   1. Copiá este archivo como include/secrets.h (ese sí está en .gitignore).
//   2. Completá el nombre y la contraseña del hotspot del celular.
//
// Si WIFI_SSID queda vacío, la telemetría se desactiva y el resto funciona igual.
// =============================================================================

// Hotspot del celular. En iPhone el nombre es el del teléfono (Ajustes >
// General > Información > Nombre) y hay que activar "Maximizar compatibilidad"
// para que emita en 2,4 GHz, que es la única banda que ve el ESP32.
#define WIFI_SSID     ""
#define WIFI_PASSWORD ""

// Servidor Traccar. El demo público sirve para pruebas y lo pueden borrar.
// Protocolo OsmAnd por HTTP en el puerto 5055.
#define TRACCAR_HOST "demo.traccar.org"
#define TRACCAR_PORT 5055

// Prefijo de los dispositivos en Traccar. Cada rol sube con su sufijo:
//   <prefijo>-reg         registrador (fase 1)
//   <prefijo>-tren        posición del tren según su propio GNSS
//   <prefijo>-cruce       posición del nodo cruce
//   <prefijo>-tren-lora   posición del tren según lo que llegó por radio al cruce
// Hay que dar de alta en Traccar los que se vayan a usar.
#define TRACCAR_ID_PREFIX "panda-itba"

// Identificador del nodo tren en el beacon. Si no se define, sale de la MAC.
// Define también su ranura TDMA (id módulo cantidad de ranuras).
// #define PANDA_NODE_ID 1
