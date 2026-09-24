#include "imu_manager.h"

#include <Arduino.h>
#include <ImuDrv.hpp>
#include <SPI.h>
#include <cmath>
#include <esp_timer.h>

#include "board_pins.h"
#include "config.h"
#include "system_state.h"

ImuManager g_imu;

static SensorQMI8658 s_qmi;
static AccelerometerData s_acc[cfg::imu::kFifoCapacity];
static GyroscopeData s_gyr[cfg::imu::kFifoCapacity];

bool ImuManager::begin() {
  if (!s_qmi.begin(SPI, pins::kImuCs, pins::kSpiMosi, pins::kSpiMiso, pins::kSpiSck)) {
    Serial.println("[IMU] QMI8658 no responde por SPI");
    return false;
  }

  // Rangos elegidos para un tren:
  //  - ±4 g: la caja de un coche de pasajeros no pasa de 1 g en servicio, y
  //    queda margen para golpes. Resolución de 0,12 mg.
  //  - ±500 °/s: un tren gira a fracciones de °/s, pero en banco la placa se
  //    mueve con la mano y no queremos saturar.
  // El filtro pasabajos en MODE_3 (13 % del ODR, ~30 Hz) conserva la firma de
  // vibración de la marcha, que es útil para detectar movimiento.
  bool ok = true;
  ok &= s_qmi.configAccel(AccelFullScaleRange::FS_4G, cfg::imu::kAccelOdrHz, SensorQMI8658::LpfMode::MODE_3);
  ok &= s_qmi.configGyro(GyroFullScaleRange::FS_500_DPS, cfg::imu::kGyroOdrHz, SensorQMI8658::LpfMode::MODE_3);

  // FIFO en modo stream de 128 muestras: si alguna vez se llena, descarta las
  // más viejas y nunca frena al sensor.
  ok &= s_qmi.configFifo(SensorQMI8658::FifoMode::STREAM, SensorQMI8658::FifoSamples::SAMPLES_128, 16);
  ok &= s_qmi.enableAccel();
  ok &= s_qmi.enableGyro();
  if (!ok) {
    Serial.println("[IMU] Falló la configuración");
    return false;
  }

  Serial.printf("[IMU] QMI8658 OK, ODR %.1f Hz, FIFO 128\n", static_cast<double>(cfg::imu::kGyroOdrHz));
  g_stats.imuOnline.store(true);
  logNote(NoteCode::ImuOnline);
  return true;
}

void ImuManager::startTask() {
  if (!g_stats.imuOnline.load()) {
    return;
  }
  xTaskCreatePinnedToCore(taskEntry, "imu", cfg::task::kStackImu, this, cfg::task::kPrioImu, nullptr,
                          cfg::task::kCoreSafety);
}

void ImuManager::taskEntry(void* arg) {
  static_cast<ImuManager*>(arg)->taskLoop();
}

void ImuManager::taskLoop() {
  // La FIFO no trae marca de tiempo por muestra. Se asigna hacia atrás desde el
  // momento de la lectura con el período medido: la última muestra leída es la
  // más reciente. El error queda acotado a un período (~4,5 ms).
  float periodUs = 1e6f / cfg::imu::kGyroOdrHz;
  uint8_t warmup = cfg::imu::kWarmupReads;

  // Estadística en ventanas de 1 s para el detector de quietud.
  uint32_t winCount = 0;
  double winAccSum = 0.0;
  double winAccSq = 0.0;
  double winGyroSum = 0.0;
  int64_t winStart = esp_timer_get_time();
  uint32_t rateSamples = 0;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(cfg::imu::kReadPeriodMs));

    const uint16_t n = s_qmi.readFromFifo(s_acc, cfg::imu::kFifoCapacity, s_gyr, cfg::imu::kFifoCapacity);
    const int64_t tRead = esp_timer_get_time();
    if (n == 0) {
      continue;
    }
    if (warmup > 0) {
      --warmup;
      continue;
    }

    for (uint16_t i = 0; i < n; ++i) {
      ImuSample s{};
      s.tUs = tRead - static_cast<int64_t>((n - 1 - i) * periodUs);
      s.ax = s_acc[i].mps2.x;
      s.ay = s_acc[i].mps2.y;
      s.az = s_acc[i].mps2.z;
      s.gx = s_gyr[i].dps.x;
      s.gy = s_gyr[i].dps.y;
      s.gz = s_gyr[i].dps.z;

      LogRecord rec{};
      rec.type = LogType::Imu;
      rec.imu = s;
      logPush(rec);

      const double aNorm = std::sqrt(static_cast<double>(s.ax * s.ax + s.ay * s.ay + s.az * s.az));
      const double gNorm = std::sqrt(static_cast<double>(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz));
      winAccSum += aNorm;
      winAccSq += aNorm * aNorm;
      winGyroSum += gNorm;
      ++winCount;
    }
    rateSamples += n;
    g_stats.imuSamples.fetch_add(n, std::memory_order_relaxed);

    const int64_t elapsed = tRead - winStart;
    if (elapsed >= 1000000 && winCount > 0) {
      const double rate = rateSamples * 1e6 / static_cast<double>(elapsed);
      // Se actualiza el período con la tasa real del oscilador del sensor,
      // filtrado para no saltar por el jitter de la lectura.
      if (rate > 50.0) {
        periodUs = 0.9f * periodUs + 0.1f * static_cast<float>(1e6 / rate);
      }

      const double mean = winAccSum / winCount;
      const double var = std::max(0.0, winAccSq / winCount - mean * mean);
      const double gyroMean = winGyroSum / winCount;
      const bool still = std::sqrt(var) < cfg::imu::kStillAccStdMps2 && gyroMean < cfg::imu::kStillGyroMeanDps;

      g_stats.imuRateX10.store(static_cast<uint32_t>(rate * 10.0));
      g_stats.imuAccNormMg.store(static_cast<uint32_t>(mean / 9.80665 * 1000.0));
      g_stats.imuStill.store(still);

      winCount = 0;
      winAccSum = winAccSq = winGyroSum = 0.0;
      rateSamples = 0;
      winStart = tRead;
    }
  }
}
