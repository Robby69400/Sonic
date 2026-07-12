#!/usr/bin/env bash
clear
set -euo pipefail

# ---------------------------------------------
# Usage:
#   ./compile-with-docker.sh [Preset] [CMake options...]
# Examples:
#   ./compile-with-docker.sh Custom
#   ./compile-with-docker.sh Bandscope -DENABLE_SPECTRUM=ON
#   ./compile-with-docker.sh Broadcast -DENABLE_FEAT_F4HWN_GAME=ON -DENABLE_NOAA=ON
#   ./compile-with-docker.sh All
# Default preset: "Custom"
# ---------------------------------------------

IMAGE=uvk1-uvk5v3

# Initialisation des variables par défaut
CLEAN_BUILD=false
EXTRA_ARGS=()
PRESET=""

# Boucle pour analyser TOUS les arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--clean)
      CLEAN_BUILD=true
      shift
      ;;
    USB|RS232|USB_SMOOTH_SPECTRUM|RS232_SMOOTH_SPECTRUM|All)
      PRESET="$1"
      shift
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

# Si aucun preset n'a été détecté dans les arguments, on met la valeur par défaut
PRESET=${PRESET:-USB}
# ---------------------------------------------
# Nettoyage si l'option est activée
# ---------------------------------------------
if [ "$CLEAN_BUILD" = true ]; then
  echo " 🧹 Cleaning build directory..."
  rm -rf build/
fi

# ---------------------------------------------
# Validate preset name
# ---------------------------------------------
if [[ ! "$PRESET" =~ ^(USB|RS232|USB_SMOOTH_SPECTRUM|RS232_SMOOTH_SPECTRUM|All)$ ]]; then
  echo "❌ Unknown preset: '$PRESET'"
  echo "Valid presets are: USB RS232 USB_SMOOTH_SPECTRUM RS232_SMOOTH_SPECTRUM All"
  exit 1
fi

# ---------------------------------------------
# Build the Docker image (only needed once)
# ---------------------------------------------
if [[ "$(docker images -q $IMAGE)" == "" ]]; then
  echo "Building Docker image..."
  docker build -t "$IMAGE" .
fi

# ---------------------------------------------
# Clean existing CMake cache to ensure toolchain reload
# ---------------------------------------------
# rm -rf build
export MSYS_NO_PATHCONV=1

# ---------------------------------------------
# Function to build one preset
# ---------------------------------------------
# ---------------------------------------------
# Function to build one preset
# ---------------------------------------------
build_preset() {
  local preset="$1"
  echo -e "\n 🚀 Building: ${preset}"
  docker run --rm -u $(id -u):$(id -g) -v "$PWD":/src -w /src "$IMAGE" \
  bash -c "cmake --preset ${preset} ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} && \
           cmake --build --preset ${preset} -j" \
  2>&1 | sed "s|/src/|C:/Perso/Sonic/|g" \
       | sed -E '/^[[:space:]]+[A-Za-z0-9_]+(:[A-Za-z]+)?=/d; /--( Configuring|Generating) done/d; /-- Build files have been written to/d'

  docker run --rm -v "$PWD":/src -w /src "$IMAGE" \
    arm-none-eabi-size ./build/${preset}/SONIC.${preset}.V34.elf

  echo "✅ Done: ${preset}"
}


# ---------------------------------------------
# Handle 'All' preset
# ---------------------------------------------
if [[ "$PRESET" == "All" ]]; then
  PRESETS=(USB RS232 USB_SMOOTH_SPECTRUM RS232_SMOOTH_SPECTRUM)
  for p in "${PRESETS[@]}"; do
    build_preset "$p"
  done
  echo ""
  echo "🎉 All presets built successfully!"
else
  build_preset "$PRESET"
fi

# ---------------------------------------------
# Automatic flash
# ---------------------------------------------

echo "⚡ Flashing USB firmware on COM14..."

# Vérification de l'existence du fichier avant de flasher
IFILE="./build/USB/SONIC.USB.V34.bin"

if [[ -f "$IFILE" ]]; then
    python flash.py "$IFILE" -p COM14
    echo "✅ Flash terminé avec succès !"
else
    echo "❌ Erreur : Le fichier binaire est introuvable : $IFILE"
    exit 1
fi
