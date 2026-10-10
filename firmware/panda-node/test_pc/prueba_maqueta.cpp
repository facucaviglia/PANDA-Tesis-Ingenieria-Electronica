// Prueba en PC de la maqueta de barrera y las intermitencias de
// crossing_io.cpp, con un reloj simulado. Se corre con test_pc/correr.sh.
// Verifica: luces alternadas (nunca las dos), luces en toda fase menos
// ARRIBA (también en la subida, SETOP 8.6.6), al menos 5 s de luces antes de
// cada bajada (incluida la que arranca desde SUBIENDO), campana reducida solo
// con el brazo abajo (Anexo XII 5.5), rojo peatonal intermitente durante t_p
// y OTRO TREN intermitente (Anexo XII 4.2).
#include <cstdio>
#include "Arduino.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "board_pins.h"
#include "config.h"
#include "crossing_io.h"
#include "system_state.h"

HardwareSerial Serial; EspClass ESP;
static int64_t g_now = 0;
static int g_pin[64];
static esp_timer_cb_t g_cb;
static int g_duty = -1; static double g_freq = 0;
int64_t esp_timer_get_time() { return g_now; }
esp_err_t esp_timer_create(const esp_timer_create_args_t* a, esp_timer_handle_t*) { g_cb = a->callback; return ESP_OK; }
esp_err_t esp_timer_start_periodic(esp_timer_handle_t, uint64_t) { return ESP_OK; }
int gpio_set_level(gpio_num_t p, uint32_t v) { g_pin[p] = v; return 0; }
int gpio_get_level(gpio_num_t p) { return g_pin[p]; }
void pinMode(uint8_t, uint8_t) {}
double ledcSetup(uint8_t, double, uint8_t) { return 0; }
void ledcAttachPin(uint8_t, uint8_t) {}
void ledcWrite(uint8_t ch, uint32_t d) { if (ch == 0) g_duty = d; }
double ledcWriteTone(uint8_t ch, double f) { if (ch == 0) { g_freq = f; g_duty = f ? 512 : 0; } return f; }
void logNote(NoteCode, uint32_t) {}
int HardwareSerial::printf(const char*, ...) { return 0; }

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { ++fails; std::printf("FALLA t=%.2f: ", g_now / 1e6); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

int main() {
  using crossio::BarrierPhase;
  crossio::begin();
  crossio::Outputs libre{true, true, false, false, false, false};
  crossio::Outputs cierre{false, true, true, true, false, false};
  crossio::Outputs cierre2{false, true, true, true, true, false};
  BarrierPhase prev = BarrierPhase::Arriba;
  int64_t lightsSince = -1;
  int pedToggles = 0, pedLast = -1, otherToggles = 0, otherLast = -1;
  bool sawSubiendoLights = false, sawLow = false, sawFullInFono = false;
  for (g_now = 50000; g_now <= 60000000; g_now += 50000) {
    const double t = g_now / 1e6;
    // Guion: cierre a 1 s, libera a 20 s, vuelve a pedir a 21 s (en plena
    // subida), OTRO TREN de 25 a 30 s, libera a 40 s.
    const crossio::Outputs& o = t < 1 ? libre : t < 20 ? cierre : t < 21 ? libre : (t >= 25 && t < 30) ? cierre2 : t < 40 ? cierre : libre;
    crossio::apply(o);
    g_cb(nullptr);
    const BarrierPhase p = crossio::barrierPhase();
    const bool lights = g_pin[pins::kOutBarrierLightA] || g_pin[pins::kOutBarrierLightB];
    if (lights && lightsSince < 0) lightsSince = g_now;
    if (!lights) lightsSince = -1;
    // Luces alternadas: nunca las dos a la vez.
    CHECK(!(g_pin[pins::kOutBarrierLightA] && g_pin[pins::kOutBarrierLightB]), "las dos luces a la vez");
    // Toda fase distinta de Arriba tiene luces.
    CHECK((p != BarrierPhase::Arriba) == lights, "fase %s con luces=%d", crossio::barrierPhaseName(p), lights);
    if (p == BarrierPhase::Subiendo && lights) sawSubiendoLights = true;
    if (p == BarrierPhase::Bajando && prev != BarrierPhase::Bajando) {
      CHECK(lightsSince >= 0 && g_now - lightsSince >= 5000000, "bajada con %.2f s de luces previas",
            lightsSince < 0 ? 0.0 : (g_now - lightsSince) / 1e6);
      std::printf("t=%5.2f  empieza a bajar desde %s, luces desde hace %.2f s\n", t, crossio::barrierPhaseName(prev),
                  lightsSince < 0 ? 0.0 : (g_now - lightsSince) / 1e6);
    }
    if (p == BarrierPhase::Subiendo && prev != BarrierPhase::Subiendo) { std::printf("t=%5.2f  empieza a subir\n", t); }
    // Campana: reducida solo con el brazo horizontal.
    if (g_duty > 0) {
      if (p == BarrierPhase::Abajo) { CHECK(g_duty == (int)cfg::crossing::kBellLowDuty, "abajo con duty %d", g_duty); sawLow = true; }
      else { CHECK(g_duty == 512, "%s con duty %d", crossio::barrierPhaseName(p), g_duty);
             if (p == BarrierPhase::Fono) sawFullInFono = true; }
      CHECK(g_freq == cfg::crossing::kBeepHz, "tono %.0f", g_freq);
    }
    // Peatonal: intermitente durante t_p desde el aviso (1 s y 21 s) y fija después.
    const int ped = g_pin[pins::kOutPedestrian];
    if (o.pedestrian && t >= 1 && t < 18) { if (pedLast >= 0 && ped != pedLast) ++pedToggles; }
    if (o.pedestrian && t > 18.2 && t < 20) CHECK(ped == 1, "peatonal apagada después de t_p");
    if (!o.pedestrian) CHECK(ped == 0, "peatonal encendida sin aviso");
    pedLast = ped;
    const int ot = g_pin[pins::kOutOtherTrain];
    if (t >= 25 && t < 30) { if (otherLast >= 0 && ot != otherLast) ++otherToggles; }
    else CHECK(ot == 0, "OTRO TREN encendido fuera de su ventana");
    otherLast = ot;
    prev = p;
  }
  CHECK(sawSubiendoLights, "nunca hubo luces durante la subida");
  CHECK(sawLow, "nunca bajó el nivel con el brazo abajo");
  CHECK(sawFullInFono, "la fonoluminosa no sonó a nivel pleno");
  std::printf("peatonal: %d cambios en los primeros 17 s (esperado unos 34)\n", pedToggles);
  std::printf("OTRO TREN: %d cambios en 5 s (esperado unos 9)\n", otherToggles);
  CHECK(pedToggles >= 30 && pedToggles <= 36, "intermitencia peatonal");
  CHECK(otherToggles >= 8 && otherToggles <= 10, "intermitencia OTRO TREN");
  CHECK(crossio::barrierPhase() == BarrierPhase::Arriba, "no terminó arriba");
  std::printf("%s (%d fallas)\n", fails ? "FALLA" : "OK", fails);
  return fails ? 1 : 0;
}
