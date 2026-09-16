#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export COPPELIASIM_ROOT="${COPPELIASIM_ROOT:-${REPO_ROOT}/.deps/CoppeliaSim}"
export LD_LIBRARY_PATH="${COPPELIASIM_ROOT}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export QT_QPA_PLATFORM_PLUGIN_PATH="${COPPELIASIM_ROOT}"

if [[ ! -f "${COPPELIASIM_ROOT}/libcoppeliaSim.so" ]]; then
  echo "CoppeliaSim 4.1 was not found at ${COPPELIASIM_ROOT}." >&2
  echo "Run: bash scripts/setup_environment.sh" >&2
  return 1 2>/dev/null || exit 1
fi
