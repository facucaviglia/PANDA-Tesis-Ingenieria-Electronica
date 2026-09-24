#pragma once
// Estado y funciones compartidas entre la parte común del enlace y las tareas
// de transmisión y recepción. No es parte de la interfaz pública.

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "radio_link.h"

namespace radiolink {

// Bits de notificación de la tarea de enlace (el bit 0 es la IRQ de la radio).
constexpr uint32_t kReqProfile = 1u << 1;
constexpr uint32_t kReqPower = 1u << 2;
constexpr uint32_t kReqScan = 1u << 3;
constexpr uint32_t kReqMask = kReqProfile | kReqPower | kReqScan;
constexpr uint32_t kNotifyInject = 1u << 4;

// Paquete inyectado por el simulador de la PC.
struct InjectedPacket {
  uint8_t data[35];
  float rssiDbm;
  float snrDb;
};

extern TaskHandle_t g_linkTask;
extern QueueHandle_t g_txStatusQ;
extern QueueHandle_t g_rxStatusQ;
extern QueueHandle_t g_authBeaconQ;
extern QueueHandle_t g_injectQ;

// Atiende los pedidos pendientes. Devuelve true si cambió la configuración de
// la radio (perfil o frecuencia), para que la tarea recalcule lo que dependa.
// Al volver, la radio queda en standby en el canal de trabajo.
bool handleRequests(uint32_t bits);

}  // namespace radiolink
