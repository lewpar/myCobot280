#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$PROJECT_DIR/venv"
PYTHON="$VENV_DIR/bin/python"
FRONTEND_DIR="$PROJECT_DIR/src/frontend"

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

    if [ ! -d "$FRONTEND_DIR/node_modules" ]; then
        echo -e "${CYAN}Installing React dependencies...${NC}"
        (cd "$FRONTEND_DIR" && npm install --silent)
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
    echo
    check_deps
    trap '' INT
    $PYTHON -m uvicorn main:app --app-dir "$PROJECT_DIR/src/backend" --host 0.0.0.0 --port 8000 --reload
}

run_frontend() {
    echo -e "${GREEN}Starting React frontend on http://localhost:5173${NC}"
    echo
    check_deps
    (cd "$FRONTEND_DIR" && npm run dev)
}

run_arm_server() {
    need_port
    echo -e "${GREEN}Starting TCP arm server on 0.0.0.0:5000${NC}"
    echo -e "${GREEN}  Serial port: $MYCOBOT_PORT${NC}"
    echo
    check_deps
    $PYTHON "$PROJECT_DIR/arm_server.py" --host 0.0.0.0 --port 5000 --serial-port "$MYCOBOT_PORT"
}

run_arm_client() {
    echo -e "${GREEN}Starting TCP arm client...${NC}"
    echo
    check_deps
    $PYTHON "$PROJECT_DIR/arm_client.py"
}

menu() {
    banner
    echo "  1) Run FastAPI backend"
    echo "  2) Run React frontend"
    echo "  3) Run TCP arm server   (arm_server.py)"
    echo "  4) Run TCP arm client   (arm_client.py)"
    echo "  5) Quit"
    echo
    read -r -p "  Choose [1-5]: " choice

    case "$choice" in
        1) run_backend ;;
        2) run_frontend ;;
        3) run_arm_server ;;
        4) run_arm_client ;;
        5) echo "Goodbye."; exit 0 ;;
        *) echo -e "${RED}Invalid choice${NC}"; exit 1 ;;
    esac
}

# ---- main ----
PORT_ARG=""

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

if [ $# -gt 0 ]; then
    case "$1" in
        backend)  run_backend ;;
        frontend) run_frontend ;;
        server)   run_arm_server ;;
        client)   run_arm_client ;;
        help|-h|--help)
            echo "Usage: ./run.sh [backend|frontend|server|client] [--port /dev/ttyX]"
            echo ""
            echo "  --port PATH    Serial port for the arm (default: /dev/ttyAMA0, or \$MYCOBOT_PORT)"
            echo "  No args        Interactive menu"
            ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; exit 1 ;;
    esac
else
    menu
fi
