#pragma once
// =============================================================================
// OLED SH1106 128x64 por I2C de hardware (bus Wire, GPIO 17/18) y consola.
//
// La tarea de interfaz corre en el core 0 con la prioridad más baja: refresca
// la pantalla a 5 Hz, atiende el botón, lee la batería y escribe el resumen
// por la consola serie una vez por segundo. Un refresco completo tarda ~25 ms
// y nunca compite con el lazo de seguridad del core 1.
// =============================================================================

#include <cstdint>

class DisplayManager {
 public:
  // Si el OLED no responde se sigue sin pantalla (no es crítico).
  bool begin();

  // Mensajes de arranque, antes de lanzar la tarea.
  void bootStep(const char* line);

  // Pantalla de falla fatal. Solo se usa desde setup().
  void showFatal(const char* subsystem, const char* detail);

  void startTask();

 private:
  static void taskEntry(void* arg);
  void taskLoop();
  void drawStatus();
  bool online_ = false;
  uint8_t bootLine_ = 0;
};

extern DisplayManager g_display;
