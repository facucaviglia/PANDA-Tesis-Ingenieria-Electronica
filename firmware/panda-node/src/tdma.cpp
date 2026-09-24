#include "tdma.h"

#include <algorithm>

#include "config.h"
#include "gnss_manager.h"

namespace tdma {

SlotPlan makePlan(uint32_t airUs, uint16_t nodeId) {
  SlotPlan p{};
  p.airUs = airUs;
  // Aire más guarda, redondeado hacia arriba al ms para que los bordes de
  // ranura caigan en valores fáciles de leer en un analizador lógico.
  p.slotUs = ((airUs + cfg::tdma::kGuardUs + 999) / 1000) * 1000;
  const uint32_t fit = cfg::tdma::kFrameUs / p.slotUs;
  p.slotCount = static_cast<uint8_t>(std::max<uint32_t>(1, std::min<uint32_t>(fit, cfg::tdma::kMaxSlots)));
  p.mySlot = static_cast<uint8_t>(nodeId % p.slotCount);
  p.myOffsetUs = (cfg::tdma::kFirstSlotOffsetUs + p.mySlot * p.slotUs) % cfg::tdma::kFrameUs;
  return p;
}

int64_t nextSlotTow(const SlotPlan& plan, int64_t towNowUs, int64_t marginUs) {
  const int64_t frame = cfg::tdma::kFrameUs;
  const int64_t frameStart = (towNowUs / frame) * frame;
  int64_t t = frameStart + plan.myOffsetUs;
  while (t < towNowUs + marginUs) {
    t += frame;
  }
  // La semana GPS dura un número entero de tramas, así que el módulo conserva
  // la alineación también en el cambio de semana.
  return t % kGpsWeekUs;
}

}  // namespace tdma
