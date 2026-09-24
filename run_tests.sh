#!/usr/bin/env bash
# Run every test without the arm: pytest (backend against a fake servo bus) and the node tests
# (page kinematics vs arm_model.py, and a jsdom smoke test of the simulator page).
#
#   ./run_tests.sh [pytest args]      e.g. ./run_tests.sh -k playback -x
#
# Uses venv/ (created by run.sh) and installs the test requirements into it; node tests need
# node >= 18 and install their packages into tests/js/node_modules on first run.
set -euo pipefail
cd "$(dirname "$0")"
PY=venv/bin/python
[ -x "$PY" ] || { python3 -m venv venv; }
"$PY" -m pip install -q --disable-pip-version-check -r tests/requirements.txt
if command -v node >/dev/null && [ -f tests/js/package.json ]; then
    [ -d tests/js/node_modules ] || (cd tests/js && npm install --silent --no-audit --no-fund)
fi
exec "$PY" -m pytest "$@"
