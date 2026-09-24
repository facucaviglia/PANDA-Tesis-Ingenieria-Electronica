// =============================================================================
// Tarea de enlace del NODO TREN: un beacon por trama de 100 ms en su ranura.
// =============================================================================
#if defined(PANDA_ROLE_TREN)

#include <Arduino.h>
#include <algorithm>
#include <climits>
#include <esp_timer.h>

#include "beacon.h"
#include "gnss_manager.h"
#include "persist.h"
#include "radio_link_internal.h"
#include "radio_manager.h"
#include "system_state.h"
#include "tdma.h"

namespace radiolink {

// Margen entre "ahora" y la ranura elegida: tiempo para dormir, despertar y
// armar el paquete. Si la ranura de esta trama ya está demasiado cerca, se usa
// la de la trama siguiente.
static constexpr int64_t kSlotMarginUs = 3000;

// Tiempo que se duerme antes de la ranura y después se espera activo. El tick
// de FreeRTOS es de 1 ms, así que dormir hasta el instante exacto no alcanza:
// se despierta antes y se espera los últimos ~2 ms sin soltar el núcleo. Esto
// deja el error de arranque de la ranura en microsegundos.
static constexpr int64_t kSpinWindowUs = 1500;

// Espera hasta el instante local target con precisión de microsegundos.
static void waitUntil(int64_t target) {
  const int64_t remaining = target - esp_timer_get_time();
  if (remaining > kSpinWindowUs + 1000) {
    vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>((remaining - kSpinWindowUs) / 1000)));
  }
  while (esp_timer_get_time() < target) {
    // Espera activa corta, en el core 1 y con la prioridad más alta.
  }
}

// Espera el TX_DONE de la radio. Los pedidos que lleguen mientras tanto no se
// pierden: quedan en el valor de notificación para la próxima vuelta.
static bool waitTxDone(uint32_t timeoutUs) {
  const int64_t deadline = esp_timer_get_time() + timeoutUs;
  for (;;) {
    const int64_t left = deadline - esp_timer_get_time();
    if (left <= 0) {
      return false;
    }
    uint32_t bits = 0;
    xTaskNotifyWait(0, kNotifyRadioIrq, &bits, pdMS_TO_TICKS(left / 1000 + 1));
    if (bits & kNotifyRadioIrq) {
      return true;
    }
  }
}

static void txTask(void*) {
  const uint16_t nodeId = persist::nodeId();
  g_stats.nodeId.store(nodeId);

  uint32_t counter = persist::reserveCounterBlock();
  uint32_t counterLimit = counter + cfg::beacon::kCounterBlock;
  logNote(NoteCode::CounterReserve, counter);

  BeaconCodec codec(cfg::beacon::kEncKey, cfg::beacon::kMacKey);
  g_radio.setIrqTask(xTaskGetCurrentTaskHandle());

  tdma::SlotPlan plan = tdma::makePlan(g_radio.timeOnAirUs(cfg::radio::kBeaconLength), nodeId);
  Serial.printf("[TREN] Nodo %04X, ranura %u de %u, %lu us por ranura, aire %lu us\n", nodeId, plan.mySlot,
                plan.slotCount, static_cast<unsigned long>(plan.slotUs), static_cast<unsigned long>(plan.airUs));
  g_radio.standbyXosc();

  TxStatus status{};
  status.lastTxAgeMs = INT32_MIN;
  int64_t lastUnsyncedTx = 0;

  for (;;) {
    // 1) Pedidos pendientes (perfil, potencia, barrido). No bloquea.
    //    Se leen y se borran en una sola operación atómica. Con xTaskNotifyWait
    //    y timeout cero no alcanza: si el pedido llegó mientras se esperaba un
    //    TX_DONE, el bit queda puesto sin estado "notificado" y nunca se borra,
    //    y el pedido se ejecutaría en cada trama.
    const uint32_t bits = ulTaskNotifyValueClear(nullptr, kReqMask);
    if (bits & kReqMask) {
      if (handleRequests(bits)) {
        plan = tdma::makePlan(g_radio.timeOnAirUs(cfg::radio::kBeaconLength), nodeId);
        Serial.printf("[TREN] Nuevo plan: ranura %u de %u, %lu us\n", plan.mySlot, plan.slotCount,
                      static_cast<unsigned long>(plan.slotUs));
      }
      g_radio.standbyXosc();
    }

    // 2) Cuándo transmitir.
    const int64_t now = esp_timer_get_time();
    int64_t towNow = 0;
    const TimeQuality q = gpsTowAt(now, towNow);
    int64_t txLocal;
    uint8_t slot;
    if (q != TimeQuality::None) {
      const int64_t slotTow = tdma::nextSlotTow(plan, towNow, kSlotMarginUs);
      txLocal = now + towDiffUs(slotTow, towNow);
      slot = plan.mySlot;
    } else if (cfg::tdma::kAllowUnsyncedTx) {
      // Sin tiempo GPS: 10 Hz con el reloj local, sin ranura. Solo sirve para
      // medir el enlace en banco. El cruce nunca lo toma como dato válido.
      txLocal = std::max(now + kSlotMarginUs, lastUnsyncedTx + static_cast<int64_t>(cfg::tdma::kFrameUs));
      slot = 255;
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    waitUntil(txLocal);

    // 3) Armar el beacon con el fix más reciente.
    const int64_t tStart = esp_timer_get_time();
    GnssFix fix{};
    const bool haveFix = xQueuePeek(g_latestFix, &fix, 0) == pdTRUE;
    const bool fixFresh = haveFix && fix.fixOk && (tStart - fix.tRxUs) < 1000000;
    int64_t towTx = 0;
    const TimeQuality qTx = gpsTowAt(tStart, towTx);

    BeaconData b{};
    b.nodeId = nodeId;
    b.counter = counter;
    b.flags = 0;
    if (fixFresh) b.flags |= BeaconFlag::kFixOk;
    if (qTx != TimeQuality::None) b.flags |= BeaconFlag::kTimeSync;
    if (qTx == TimeQuality::Pps) b.flags |= BeaconFlag::kTimePps;
    if (g_stats.imuOnline.load()) b.flags |= BeaconFlag::kImuOk;
    if (g_stats.imuOnline.load() && g_stats.imuStill.load()) b.flags |= BeaconFlag::kStill;
    if (haveFix) {
      b.itowMs = fix.itowMs;
      b.latE7 = fix.latE7;
      b.lonE7 = fix.lonE7;
      b.speedCms = static_cast<uint16_t>(std::min<int32_t>(std::max<int32_t>(fix.gSpeedMms / 10, 0), 65535));
      // headMot viene en 1e-5 grados, se pasa a centésimas de grado en [0, 36000).
      int32_t head = (fix.headMotE5 / 1000) % 36000;
      if (head < 0) head += 36000;
      b.headingCdeg = static_cast<uint16_t>(head);
      b.hAccCm = static_cast<uint16_t>(std::min<uint32_t>(fix.hAccMm / 10, 65535));
      b.numSv = fix.numSv;
    }

    uint8_t packet[cfg::radio::kBeaconLength];
    codec.seal(b, packet);

    // 4) Transmitir y esperar el fin. Se limpia antes cualquier IRQ vieja para
    //    que no se confunda con el TX_DONE de este paquete.
    ulTaskNotifyValueClear(nullptr, kNotifyRadioIrq);
    const int16_t st = g_radio.startTransmit(packet, sizeof(packet));
    const bool done = (st == 0) && waitTxDone(plan.airUs + 50000);
    const uint32_t airUs = done ? static_cast<uint32_t>(g_radio.lastIrqUs() - tStart) : 0;
    g_radio.finishTransmit();
    g_radio.standbyXosc();

    // 5) Registro y estado.
    TxLog tx{};
    tx.tStartUs = tStart;
    tx.counter = counter;
    tx.itowMs = b.itowMs;
    tx.txAgeMs = (haveFix && qTx != TimeQuality::None)
                     ? static_cast<int32_t>(towDiffUs(towTx, static_cast<int64_t>(fix.itowMs) * 1000) / 1000)
                     : INT32_MIN;
    tx.airUs = airUs;
    tx.status = done ? st : static_cast<int16_t>(-999);
    tx.flags = b.flags;
    tx.slot = slot;
    tx.profile = activeProfileIndex();
    tx.timeQ = static_cast<uint8_t>(qTx);
    tx.powerDbm = activePowerDbm();
    LogRecord rec{};
    rec.type = LogType::Tx;
    rec.tx = tx;
    logPush(rec);

    ++status.txCount;
    status.counter = counter + 1;
    status.lastTxAgeMs = tx.txAgeMs;
    status.lastAirUs = airUs;
    status.lastStatus = tx.status;
    status.timeQ = tx.timeQ;
    status.slot = slot;
    status.slotCount = plan.slotCount;
    status.slotUs = plan.slotUs;
    status.airUs = plan.airUs;
    xQueueOverwrite(g_txStatusQ, &status);

    // 6) Contador: nunca se repite. Al agotar el bloque se reserva otro, justo
    //    después de transmitir para que la escritura en flash no demore nada.
    ++counter;
    if (counter >= counterLimit) {
      counter = persist::reserveCounterBlock();
      counterLimit = counter + cfg::beacon::kCounterBlock;
      logNote(NoteCode::CounterReserve, counter);
    }
    if (slot == 255) {
      lastUnsyncedTx = tStart;
    }
  }
}

void startTask() {
  xTaskCreatePinnedToCore(txTask, "link-tx", cfg::task::kStackRadio, nullptr, cfg::task::kPrioRadio, &g_linkTask,
                          cfg::task::kCoreSafety);
}

}  // namespace radiolink

#endif  // PANDA_ROLE_TREN
