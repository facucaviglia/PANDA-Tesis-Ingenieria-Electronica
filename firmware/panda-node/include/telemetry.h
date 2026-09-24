#pragma once
// =============================================================================
// Telemetría a Traccar por Wi-Fi (capa de inteligencia).
//
// El nodo se conecta al hotspot del celular y sube una posición por segundo
// con el protocolo OsmAnd de Traccar (un GET HTTP). El celular hace de módem
// 4G, igual que lo hará el SIM7080G en el producto. Cuando llegue el módem solo
// cambia el transporte, no esta interfaz.
//
// Aislamiento respecto de la seguridad (R-31): corre en el core 0 con prioridad
// baja y solo lee el último fix con xQueuePeek. Si el Wi-Fi se cae o el
// servidor no responde, ninguna tarea del core 1 se entera.
// =============================================================================

class Telemetry {
 public:
  void startTask();

 private:
  static void taskEntry(void* arg);
  void taskLoop();
};

extern Telemetry g_telemetry;
