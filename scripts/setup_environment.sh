#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
COPPELIASIM_ROOT="${COPPELIASIM_ROOT:-${REPO_ROOT}/.deps/CoppeliaSim}"
COPPELIASIM_URL="https://www.coppeliarobotics.com/files/V4_1_0/CoppeliaSim_Edu_V4_1_0_Ubuntu20_04.tar.xz"

if [[ ! -f "${COPPELIASIM_ROOT}/libcoppeliaSim.so" ]]; then
  if ! command -v curl >/dev/null 2>&1; then
    echo "curl is required to download CoppeliaSim." >&2
    exit 1
  fi

  mkdir -p "${COPPELIASIM_ROOT}"
  ARCHIVE_PATH="$(mktemp --suffix=.tar.xz)"
  trap 'rm -f "${ARCHIVE_PATH}"' EXIT
  curl --fail --location "${COPPELIASIM_URL}" --output "${ARCHIVE_PATH}"
  tar -xf "${ARCHIVE_PATH}" -C "${COPPELIASIM_ROOT}" --strip-components 1
fi

export COPPELIASIM_ROOT
export LD_LIBRARY_PATH="${COPPELIASIM_ROOT}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export QT_QPA_PLATFORM_PLUGIN_PATH="${COPPELIASIM_ROOT}"

"${PYTHON_BIN}" -m pip install -r "${REPO_ROOT}/third_party/PyRep/requirements.txt"
"${PYTHON_BIN}" -m pip install -e "${REPO_ROOT}/third_party/PyRep"
"${PYTHON_BIN}" -m pip install -r "${REPO_ROOT}/requirements.txt"
"${PYTHON_BIN}" -m pip install -e "${REPO_ROOT}"

echo
echo "Installation complete. Before using RLBench in a new shell, run:"
echo "  source scripts/env.sh"
