#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$PROJECT_DIR/venv"
PYTHON="$VENV_DIR/bin/python"

if [ ! -x "$PYTHON" ]; then
    PYTHON="$(command -v python3 || true)"
fi

RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
BOLD='\033[1m'

banner() {
    echo -e "${CYAN}${BOLD}========================================${NC}"
    echo -e "${CYAN}${BOLD}        MyCobot280 Arm Control          ${NC}"
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
    export MYCOBOT_PORT
    echo -e "${GREEN}Using serial port: ${BOLD}$MYCOBOT_PORT${NC}"
}

check_deps() {
    if [ -z "$PYTHON" ]; then
        echo -e "${RED}python3 not found. Install Python 3 to continue.${NC}"
        exit 1
    fi

    "$PYTHON" -c "import serial" 2>/dev/null || {
        echo -e "${RED}pyserial is not installed.${NC}"
        echo -e "Install it with: ${BOLD}$PYTHON -m pip install pyserial${NC}"
        exit 1
    }
}

need_port() {
    [ -z "${MYCOBOT_PORT:-}" ] && prompt_port
}

run_server() {
    need_port
    echo -e "${GREEN}Starting TCP arm server on ${BOLD}${HOST:-0.0.0.0}:${TCP_PORT:-5000}${NC}"
    echo -e "${GREEN}  Serial port: $MYCOBOT_PORT${NC}"
    echo
    check_deps
    $PYTHON "$PROJECT_DIR/server.py" \
        --host "${HOST:-0.0.0.0}" \
        --port "${TCP_PORT:-5000}" \
        --serial-port "$MYCOBOT_PORT"
}

run_client() {
    echo -e "${GREEN}Starting TCP arm client...${NC}"
    echo
    check_deps
    $PYTHON "$PROJECT_DIR/client.py"
}

menu() {
    banner
    echo "  1) Run TCP arm server   (server.py)"
    echo "  2) Run TCP arm client   (client.py)"
    echo "  3) Quit"
    echo
    read -r -p "  Choose [1-3]: " choice

    case "$choice" in
        1) run_server ;;
        2) run_client ;;
        3) echo "Goodbye."; exit 0 ;;
        *) echo -e "${RED}Invalid choice${NC}"; exit 1 ;;
    esac
}

usage() {
    echo "Usage: ./run.sh [server|client] [--port /dev/ttyX] [--host 0.0.0.0] [--tcp-port 5000]"
    echo
    echo "  --port PATH     Serial port for the arm (default: /dev/ttyAMA0, or \$MYCOBOT_PORT)"
    echo "  --host HOST     Bind address for the server (default: 0.0.0.0)"
    echo "  --tcp-port N    TCP port for the server (default: 5000)"
    echo "  No args         Interactive menu"
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
        --host)
            shift
            HOST="$1"
            shift
            ;;
        --tcp-port)
            shift
            TCP_PORT="$1"
            shift
            ;;
        help|-h|--help)
            usage
            exit 0
            ;;
        *)
            break
            ;;
    esac
done

if [ $# -gt 0 ]; then
    case "$1" in
        server)  run_server ;;
        client)  run_client ;;
        *) echo -e "${RED}Unknown command: $1${NC}"; usage; exit 1 ;;
    esac
else
    menu
fi
