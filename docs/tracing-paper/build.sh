#!/usr/bin/env bash
# build.sh - Genera estudio_completo.pdf a partir de estudio_completo.md
#
# Pipeline: markdown -> HTML (con MathJax) -> PDF via Chrome headless.
# No requiere instalar nada adicional: usa python3 (con markdown ya disponible)
# y Google Chrome, que ya están en el sistema.
#
# Uso:
#   ./build.sh

set -euo pipefail
cd "$(dirname "$0")"

python3 build.py
