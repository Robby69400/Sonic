#!/usr/bin/env bash
clear
set -euo pipefail

source load_settings.sh

# Initialize default variables
CLEAN_BUILD=false
EXTRA_ARGS=()
PRESET=""
FLASH=true

# Boucle pour analyser TOUS les arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-auto-flash)
      FLASH=false
      shift
      ;;
    -c|--clean)
      CLEAN_BUILD=true
      shift
      ;;
    S1K|NC4K|S4K|All)
      PRESET="$1"
      shift
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

# If no preset was detected in the arguments, use the default value
PRESET=${PRESET:-S4K}

# ---------------------------------------------
# Clean up if the option is enabled
# ---------------------------------------------
if [ "$CLEAN_BUILD" = true ]; then
  echo " 🧹 Cleaning build directory..."
  rm -rf build/
fi

# ---------------------------------------------
# Validate preset name
# ---------------------------------------------
if [[ ! "$PRESET" =~ ^(S1K|NC4K|S4K|All)$ ]]; then
  echo "❌ Unknown preset: '$PRESET'"
  echo "Valid presets are: S1K NC4K S4K All"
  exit 1
fi

# ---------------------------------------------
# Build the Docker image (only needed once)
# ---------------------------------------------
if [[ "$(docker images -q $IMAGE)" == "" ]]; then
  echo "Building Docker image..."
  docker build -t "$IMAGE" .
fi

export MSYS_NO_PATHCONV=1

# ---------------------------------------------
# Function to build one preset
# ---------------------------------------------
build_preset() {
  local preset="$1"

  local target
  case "$preset" in
    S1K) target="f4hwn.sonic1k.${VERSION_NO}" ;;
    NC4K) target="f4hwn.nochirpsonic4k.${VERSION_NO}" ;;
    *)     target="f4hwn.sonic4k.${VERSION_NO}" ;; # Default value
  esac
  echo -e "\n 🚀 Building: ${preset}"
  docker run \
    --rm \
    -u $(id -u):$(id -g) \
    -v "$PWD":/src \
    -w /src \
    -e VERSION_STRING_2=${VERSION_NO} \
    "$IMAGE" \
    bash -c "cmake --preset ${preset} -DTARGET=${target} -DVERSION_STRING_1=${VERSION_NO} -DVERSION_STRING_2=${VERSION_NO} ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} && \
             cmake --build --preset ${preset} -j" \
  2>&1 | sed "s|/src/|C:/Perso/Sonic/|g" \
       | sed -E '/^[[:space:]]+[A-Za-z0-9_]+(:[A-Za-z]+)?=/d; /--( Configuring|Generating) done/d; /-- Build files have been written to/d'

  docker run --rm -v "$PWD":/src -w /src "$IMAGE" \
    arm-none-eabi-size ./build/${preset}/${target}.elf

  echo "✅ Done: ${preset} : $(date +'%H:%M:%S')"
}

# ---------------------------------------------
# Function to flash one preset
# ---------------------------------------------
flash_preset() {
  local preset="$1"
  local target
  case "$preset" in
    S1K) target="f4hwn.sonic1k.${VERSION_NO}" ;;
    NC4K) target="f4hwn.nochirpsonic4k.${VERSION_NO}" ;;
    *)     target="f4hwn.sonic4k.${VERSION_NO}" ;; # Default value
  esac
  local ifile="./build/${preset}/${target}.bin"

  echo -e "\n⚡ Flashing ${preset} firmware on ${UPLOAD_PORT}..."

  if [[ -f "$ifile" ]]; then
      python flash.py "$ifile" -p ${UPLOAD_PORT}
      echo "✅ Flash ${preset} terminé avec succès !"
  else
      echo "❌ Erreur : Le fichier binaire est introuvable : $ifile"
      exit 1
  fi
}

# ---------------------------------------------
# Handle Build & Flash
# ---------------------------------------------
if [[ "$PRESET" == "All" ]]; then
  PRESETS=(S1K S4K)
  for p in "${PRESETS[@]}"; do
    build_preset "$p"
  done
  echo ""
  echo "🎉 All presets built successfully!"
  # If 'All' is compiled, flash only the S4K preset
  flash_preset "S4K"
else
  build_preset "$PRESET"
  if [ "$FLASH" = true ]; then
    flash_preset "$PRESET"
  fi;
fi
