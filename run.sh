#!/usr/bin/env bash
# Start the myCobot 280 backend: REST API, the /ws/arm IK link and the simulator at /sim.
#
#   ./run.sh [backend] [--port /dev/ttyX] [--host ADDR] [--http-port N] [--dev]
#
# Settings come from, highest first: command-line options, the environment, src/backend/.env
# (read by the backend itself), then the defaults below.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKEND_DIR="$PROJECT_DIR/src/backend"
VENV_DIR="$PROJECT_DIR/venv"
PYTHON="$VENV_DIR/bin/python"
REQUIREMENTS="$BACKEND_DIR/requirements.txt"
ENV_FILE="$BACKEND_DIR/.env"

HOST="${MYCOBOT_HOST:-0.0.0.0}"
HTTP_PORT="${MYCOBOT_HTTP_PORT:-8000}"
DEV="${MYCOBOT_DEV:-0}"

if [ -t 1 ]; then
    RED=$'\033[0;31m'; GREEN=$'\033[0;32m'; YELLOW=$'\033[0;33m'; CYAN=$'\033[0;36m'; BOLD=$'\033[1m'; NC=$'\033[0m'
else
    RED=''; GREEN=''; YELLOW=''; CYAN=''; BOLD=''; NC=''
fi
info() { echo "${CYAN}$*${NC}"; }
ok()   { echo "${GREEN}$*${NC}"; }
warn() { echo "${YELLOW}Warning:${NC} $*"; }
die()  { echo "${RED}Error:${NC} $*" >&2; exit 1; }

usage() {
    cat <<EOF
Usage: ./run.sh [backend] [options]

Starts the FastAPI backend: REST API, the /ws/arm IK link and the simulator at /sim.

Options:
  --port PATH       Serial port for the arm (default: MYCOBOT_PORT from the environment or
                    src/backend/.env, else /dev/ttyAMA0)
  --host ADDR       Address to listen on (default: $HOST)
  --http-port N     HTTP port (default: $HTTP_PORT)
  --dev             Restart the server when a file changes (uvicorn --reload). Not while the arm is moving.
  -h, --help        Show this help

Environment: MYCOBOT_PORT, MYCOBOT_BAUD, MYCOBOT_PASSWORD, MYCOBOT_CORS_ORIGINS (also read from
src/backend/.env), MYCOBOT_HOST, MYCOBOT_HTTP_PORT, MYCOBOT_DEV=1.
EOF
}

# value of KEY in src/backend/.env, if set there (only used to report and check the settings)
env_file_value() {
    [ -f "$ENV_FILE" ] || return 0
    sed -n "s/^[[:space:]]*$1[[:space:]]*=[[:space:]]*//p" "$ENV_FILE" | tail -n 1 | sed 's/[[:space:]]*$//; s/^["'\'']//; s/["'\'']$//'
}

setup_venv() {
    if [ ! -x "$PYTHON" ]; then
        command -v python3 >/dev/null || die "python3 is not installed."
        info "Creating a virtual environment in $VENV_DIR ..."
        python3 -m venv "$VENV_DIR" || die "Could not create the virtual environment. On Raspberry Pi OS: sudo apt install python3-venv"
    fi
    # reinstall whenever requirements.txt changes, not only when the venv is new
    local stamp="$VENV_DIR/.requirements.sha256" want have=""
    want="$(sha256sum "$REQUIREMENTS" | cut -d' ' -f1)"
    [ -f "$stamp" ] && have="$(cat "$stamp")"
    if [ "$want" != "$have" ]; then
        info "Installing backend dependencies ..."
        "$PYTHON" -m pip install -q --disable-pip-version-check -r "$REQUIREMENTS" || die "pip install failed (see above)."
        echo "$want" > "$stamp"
        ok "Dependencies installed."
    fi
}

preflight() {
    local serial="${MYCOBOT_PORT:-$(env_file_value MYCOBOT_PORT)}"
    serial="${serial:-/dev/ttyAMA0}"
    echo "  Serial port   ${BOLD}$serial${NC}"
    if [ ! -e "$serial" ]; then
        warn "$serial does not exist. The backend will start without the arm (REST calls answer 503)."
    elif [ ! -r "$serial" ] || [ ! -w "$serial" ]; then
        warn "No permission to open $serial. Add yourself to its group and log in again: sudo usermod -aG $(stat -c %G "$serial") $USER"
    fi

    if [ ! -f "$ENV_FILE" ]; then
        warn "No src/backend/.env. Copy the example and set a password:  cp src/backend/.env.example src/backend/.env"
    fi
    if [ -z "${MYCOBOT_PASSWORD:-$(env_file_value MYCOBOT_PASSWORD)}" ]; then
        warn "MYCOBOT_PASSWORD is not set, so the backend makes up a new one on every start (printed below)."
    fi

    # only one program may own the serial port (and the HTTP port)
    if command -v ss >/dev/null && ss -ltn "sport = :$HTTP_PORT" 2>/dev/null | grep -q LISTEN; then
        die "Something is already listening on port $HTTP_PORT. Is the backend already running?"
    fi
    if [ "$DEV" = "1" ]; then
        warn "Dev mode: the server restarts when a file changes, which interrupts any motion in progress."
    fi
}

print_urls() {
    local addrs=() a
    if [ "$HOST" = "0.0.0.0" ] || [ "$HOST" = "::" ]; then
        addrs+=("localhost")
        for a in $(hostname -I 2>/dev/null); do [[ "$a" == *:* ]] || addrs+=("$a"); done
        addrs+=("$(hostname).local")
    else
        addrs+=("$HOST")
    fi
    echo
    for a in "${addrs[@]}"; do
        echo "  Simulator     ${BOLD}http://$a:$HTTP_PORT/sim${NC}"
    done
    echo "  API docs      http://${addrs[0]}:$HTTP_PORT/docs"
    echo
}

# ---- arguments (in any order) ----
while [ $# -gt 0 ]; do
    case "$1" in
        backend)        shift ;;
        --port)         [ $# -ge 2 ] || die "--port needs a path"; export MYCOBOT_PORT="$2"; shift 2 ;;
        --port=*)       export MYCOBOT_PORT="${1#*=}"; shift ;;
        --host)         [ $# -ge 2 ] || die "--host needs an address"; HOST="$2"; shift 2 ;;
        --host=*)       HOST="${1#*=}"; shift ;;
        --http-port)    [ $# -ge 2 ] || die "--http-port needs a number"; HTTP_PORT="$2"; shift 2 ;;
        --http-port=*)  HTTP_PORT="${1#*=}"; shift ;;
        --dev)          DEV=1; shift ;;
        help|-h|--help) usage; exit 0 ;;
        *)              usage >&2; echo >&2; die "Unknown option: $1" ;;
    esac
done
[[ "$HTTP_PORT" =~ ^[0-9]+$ ]] || die "--http-port must be a number, not '$HTTP_PORT'"

echo "${BOLD}myCobot 280 backend${NC}"
setup_venv
preflight
print_urls

reload=()
# uvicorn only treats an exclude as a directory when it is an existing path, so pass them absolute
[ "$DEV" = "1" ] && reload=(--reload --reload-dir "$PROJECT_DIR" --reload-exclude "$VENV_DIR" --reload-exclude "$PROJECT_DIR/tools")
# exec: uvicorn replaces this shell, so Ctrl+C and service managers signal it directly
exec "$PYTHON" -m uvicorn main:app --app-dir "$BACKEND_DIR" --host "$HOST" --port "$HTTP_PORT" "${reload[@]}"
