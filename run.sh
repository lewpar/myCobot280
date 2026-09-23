#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$PROJECT_DIR/venv"
PYTHON="$VENV_DIR/bin/python"

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
BOLD='\033[1m'

banner() {
    echo -e "${CYAN}${BOLD}========================================${NC}"
    echo -e "${CYAN}${BOLD}    MyCobot280 Control Panel           ${NC}"
    echo -e "${CYAN}${BOLD}========================================${NC}"
    echo
}

prompt_port() {
    if [ -n "${MYCOBOT_PORT:-}" ]; then
        echo -e "${GREEN}Using serial port: ${BOLD}$MYCOBOT_PORT${NC} (from \$MYCOBOT_PORT)"
        return
    fi
    local default="/dev/ttyAMA0"
    echo
    read -r -p "  Serial port [$default]: " input
    MYCOBOT_PORT="${input:-$default}"
    echo -e "${GREEN}Using serial port: ${BOLD}$MYCOBOT_PORT${NC}"
    export MYCOBOT_PORT
}

check_deps() {
    if [ ! -f "$PYTHON" ]; then
        echo -e "${RED}Virtual environment not found at $VENV_DIR${NC}"
        echo "Creating one..."
        python3 -m venv "$VENV_DIR"
        echo -e "${GREEN}Created.${NC}"
    fi

    if ! "$PYTHON" -c "import fastapi" 2>/dev/null; then
        echo -e "${CYAN}Installing FastAPI dependencies...${NC}"
        "$PYTHON" -m pip install -r "$PROJECT_DIR/src/backend/requirements.txt" -q
        echo -e "${GREEN}Done.${NC}"
    fi

    "$PYTHON" -c "import serial" 2>/dev/null || {
        echo -e "${RED}pyserial not installed. Run: pip install pyserial${NC}"
        exit 1
    }
}

need_port() {
    [ -z "${MYCOBOT_PORT:-}" ] && prompt_port
}

run_backend() {
    need_port
    echo -e "${GREEN}Starting FastAPI backend on http://0.0.0.0:8000${NC}"
    echo -e "${GREEN}  Serial port: $MYCOBOT_PORT${NC}"
    echo -e "${GREEN}  API docs at http://localhost:8000/docs${NC}"
    echo -e "${GREEN}  IK simulator at http://localhost:8000/sim${NC}"
    echo
    check_deps
    trap '' INT
    # --reload restarts the server whenever a file changes: handy while editing, risky with the arm moving
    local reload=""
    [ "${MYCOBOT_DEV:-0}" = "1" ] && reload="--reload"
    $PYTHON -m uvicorn main:app --app-dir "$PROJECT_DIR/src/backend" --host 0.0.0.0 --port 8000 $reload
}

# ---- main ----
while [ $# -gt 0 ]; do
    case "$1" in
        --port)
            shift
            MYCOBOT_PORT="$1"
            export MYCOBOT_PORT
            shift
            ;;
        *)
            break
            ;;
    esac
done

case "${1:-backend}" in
    backend) banner; run_backend ;;
    help|-h|--help)
        echo "Usage: ./run.sh [backend] [--port /dev/ttyX]"
        echo ""
        echo "  Starts the FastAPI backend (REST API, /ws/arm and the IK simulator at /sim)."
        echo "  --port PATH    Serial port for the arm (default: /dev/ttyAMA0, or \$MYCOBOT_PORT)"
        ;;
    *) echo -e "${RED}Unknown option: $1${NC}"; exit 1 ;;
esac
