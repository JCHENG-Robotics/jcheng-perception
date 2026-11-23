#!/usr/bin/env bash
# Run the ROS-free core tests exactly as CI does.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
python3 -m pytest tests -q
