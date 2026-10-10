#!/bin/sh
# Pruebas en la PC, sin placa ni PlatformIO. Desde firmware/panda-node:
#   sh test_pc/correr.sh
# 1. Compila los archivos del cruce con -Wall -Wextra para los tres roles.
# 2. Corre la prueba de la maqueta (secuencia de barrera e intermitencias).
# Las cabeceras de test_pc/stubs solo imitan lo mínimo del ESP32 y Arduino.
set -e
cd "$(dirname "$0")/.."
FLAGS="-std=gnu++17 -Wall -Wextra -Werror -Iinclude -isystem test_pc/stubs"
for rol in CRUCE TREN REGISTRADOR; do
  for f in src/crossing.cpp src/crossing_io.cpp; do
    g++ $FLAGS -fsyntax-only -DPANDA_ROLE_$rol "$f"
  done
done
g++ $FLAGS -fsyntax-only -DPANDA_ROLE_CRUCE src/console.cpp
echo "Compilación de prueba: OK"
OUT="${TMPDIR:-/tmp}/panda_prueba_maqueta"
g++ $FLAGS -DPANDA_ROLE_CRUCE test_pc/prueba_maqueta.cpp src/crossing_io.cpp -o "$OUT"
"$OUT"
