#include "system_state.h"

#include <esp_heap_caps.h>
#include <esp_timer.h>

#include "config.h"

SystemStats g_stats;
QueueHandle_t g_latestFix = nullptr;
QueueHandle_t g_logQueue = nullptr;

// La cola de registro ocupa unos 150 KB. Se aloja en PSRAM para no quitarle
// RAM interna al Wi-Fi ni a las pilas de las tareas. Solo la tocan tareas,
// nunca una ISR, así que no hay problema con la caché de la flash.
static StaticQueue_t s_logQueueCtrl;

bool systemStateInit() {
  g_latestFix = xQueueCreate(1, sizeof(GnssFix));
  if (g_latestFix == nullptr) {
    return false;
  }

  const size_t bytes = cfg::sdlog::kQueueDepth * sizeof(LogRecord);
  auto* storage = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (storage == nullptr) {
    // Sin PSRAM se degrada a una cola chica en RAM interna.
    g_logQueue = xQueueCreate(256, sizeof(LogRecord));
  } else {
    g_logQueue = xQueueCreateStatic(cfg::sdlog::kQueueDepth, sizeof(LogRecord), storage, &s_logQueueCtrl);
  }
  return g_logQueue != nullptr;
}

bool logPush(const LogRecord& rec) {
  if (g_logQueue == nullptr) {
    return false;
  }
  if (xQueueSend(g_logQueue, &rec, 0) != pdTRUE) {
    g_stats.logDropped.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

void logNote(NoteCode code, uint32_t value) {
  LogRecord rec{};
  rec.type = LogType::Note;
  rec.ev.tUs = esp_timer_get_time();
  rec.ev.seq = static_cast<uint32_t>(code);
  rec.ev.value = value;
  logPush(rec);
}
