#pragma once
// =============================================================================
// Consola por el USB serie.
//
// Comandos de una letra (seguidos de Enter) para el operador, y comandos con
// argumentos para el simulador de trenes de la PC (tools/sim_trenes.py):
//   T <tow_ms>                tiempo GPS de la semana según la PC
//   R <lat_e7> <lon_e7>       referencia del cruce solo en RAM (banco)
//   B <70 hex> [rssi] [snr]   beacon de 35 bytes como si llegara por radio
//
// Corre en su propia tarea del core 0 que lee cada 10 ms, así las líneas del
// simulador (hasta 20 por segundo) no desbordan el buffer del USB.
// =============================================================================

namespace console {

void startTask();

}  // namespace console
