#!/usr/bin/env bash
# Start the myCobot 280 backend: REST API, the /ws/arm IK link and the webapp at the root (/).
#
#   ./run.sh [backend] [--sim] [--ik native|ikpy|pink] [--password PW] [--port /dev/ttyX] [--host ADDR] [--http-port N] [--dev]
#
# Run in a terminal, it asks which IK solver to use and the password for this session (Enter keeps the
# default shown); --ik / --password answer those up front, and --no-prompt skips the questions.
# Settings come from, highest first: command-line options, the answers, the environment,
# src/backend/.env (read by the backend itself), then the defaults below.
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
SIM="${MYCOBOT_SIM:-0}"   # --sim: a simulated arm instead of the serial port
IK=""            # --ik
PASSWORD=""      # --password
PASSWORD_SET=0
PROMPT=1         # ask in a terminal; --no-prompt, or no terminal, doesn't

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

Starts the FastAPI backend: REST API, the /ws/arm IK link and the webapp at the root (/).

Options:
  --sim             A simulated arm instead of the real one: try the page on any computer. Nothing moves
                    for real; its calibration is kept in sim_data/, apart from the real arm's.
  --ik ENGINE       IK solver: native (fast, the default), pink (Pink on Pinocchio, about 5x slower;
                    64-bit OS only) or ikpy (IKPy, about 25x slower). Installed on first use.
  --password PW     Password for this session (the page, /ws/arm and the REST API). It shows up in
                    your shell history and the process list: prefer typing it when asked.
  --no-prompt       Don't ask; use the options, then the environment and src/backend/.env
  --port PATH       Serial port for the arm (default: MYCOBOT_PORT from the environment or
                    src/backend/.env, else /dev/ttyAMA0)
  --host ADDR       Address to listen on (default: $HOST)
  --http-port N     HTTP port (default: $HTTP_PORT)
  --dev             Restart the server when a file changes (uvicorn --reload). Not while the arm is moving.
  -h, --help        Show this help

In a terminal it asks for the solver and the password unless they're given (or --no-prompt); Enter
keeps the default it shows. Without a terminal (a service) it never asks.

Environment (optional): MYCOBOT_PORT, MYCOBOT_BAUD, MYCOBOT_PASSWORD, MYCOBOT_IK, MYCOBOT_CORS_ORIGINS
(also read from src/backend/.env), MYCOBOT_HOST, MYCOBOT_HTTP_PORT, MYCOBOT_DEV=1.
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

# the optional IK libraries: installed the first time they're chosen (Pink needs Pinocchio, which only has
# 64-bit wheels: on 32-bit Raspberry Pi OS it can't be installed)
install_engine() {
    local module packages
    case "$1" in
        ikpy) module=ikpy; packages="ikpy" ;;
        pink) module=pink; packages="pin-pink daqp" ;;
        *)    return 0 ;;
    esac
    "$PYTHON" -c "import $module" 2>/dev/null && return 0
    info "Installing the $1 solver ($packages) ..."
    if ! "$PYTHON" -m pip install -q --disable-pip-version-check $packages; then
        [ "$1" = "pink" ] && [ "$(uname -m)" != "aarch64" ] && [ "$(uname -m)" != "x86_64" ] && \
            die "Pink needs a 64-bit OS (this is $(uname -m)): Pinocchio has no wheels for it. Use --ik native."
        die "Could not install the $1 solver (see above). Use --ik native."
    fi
    ok "Installed."
}

# the solver and the password for this session: the options, else the answers to two questions (in a
# terminal), else whatever the environment or src/backend/.env says. Passed on to the backend in its
# environment, so they never appear on its command line.
choose_settings() {
    local ask=0 current
    [ "$PROMPT" = "1" ] && [ -t 0 ] && [ -t 1 ] && ask=1

    current="${MYCOBOT_IK:-$(env_file_value MYCOBOT_IK)}"
    current="${current:-native}"
    if [ -z "$IK" ] && [ "$ask" = "1" ]; then
        local def=1 answer
        [ "$current" = "ikpy" ] && def=2
        [ "$current" = "pink" ] && def=3
        echo
        echo "  Which IK solver?"
        echo "    1) native   the built-in one: fast, fine on the Pi"
        echo "    2) ikpy     IKPy, about 25 times slower"
        echo "    3) pink     Pink (on Pinocchio), about 5 times slower; needs a 64-bit OS"
        while :; do
            read -r -p "  Choose [$def]: " answer
            case "${answer:-$def}" in
                1|native) IK=native; break ;;
                2|ikpy)   IK=ikpy; break ;;
                3|pink)   IK=pink; break ;;
                *)        echo "  Type 1, 2 or 3." ;;
            esac
        done
    fi
    IK="${IK:-$current}"
    case "$IK" in native|ikpy|pink) ;; *) die "--ik must be native, ikpy or pink, not '$IK'" ;; esac
    install_engine "$IK"
    export MYCOBOT_IK="$IK"

    current="${MYCOBOT_PASSWORD:-$(env_file_value MYCOBOT_PASSWORD)}"
    if [ "$PASSWORD_SET" = "0" ] && [ "$ask" = "1" ]; then
        local first second
        echo
        while :; do
            if [ -n "$current" ]; then
                read -r -s -p "  Password for this session (Enter keeps the saved one): " first; echo
            else
                read -r -s -p "  Password for this session (Enter makes one up): " first; echo
            fi
            [ -z "$first" ] && break
            read -r -s -p "  Type it again: " second; echo
            [ "$first" = "$second" ] && { PASSWORD="$first"; PASSWORD_SET=1; break; }
            echo "  They didn't match. Try again."
        done
    fi
    if [ "$PASSWORD_SET" = "1" ]; then
        [ -n "$PASSWORD" ] || die "--password can't be empty (leave it out to be asked, or to use the saved one)."
        export MYCOBOT_PASSWORD="$PASSWORD"
    fi
    echo
}

# no arm on the serial port and someone at the keyboard: offer the simulated one
offer_sim() {
    [ "$SIM" = "1" ] && return 0
    local serial="${MYCOBOT_PORT:-$(env_file_value MYCOBOT_PORT)}"
    serial="${serial:-/dev/ttyAMA0}"
    [ "$serial" = "sim" ] && { SIM=1; return 0; }
    [ -e "$serial" ] && return 0
    [ "$PROMPT" = "1" ] && [ -t 0 ] && [ -t 1 ] || return 0
    local answer
    echo
    read -r -p "  There's no arm on $serial. Start with a simulated arm instead? [Y/n] " answer
    case "${answer:-y}" in y|Y|yes|Yes) SIM=1 ;; esac
}

preflight() {
    local serial="${MYCOBOT_PORT:-$(env_file_value MYCOBOT_PORT)}"
    serial="${serial:-/dev/ttyAMA0}"
    if [ "$SIM" = "1" ]; then
        export MYCOBOT_SIM=1
        echo "  Arm           ${BOLD}simulated${NC} (nothing moves for real; calibration in sim_data/)"
        serial=""
    else
        echo "  Serial port   ${BOLD}$serial${NC}"
    fi
    echo "  IK solver     ${BOLD}$MYCOBOT_IK${NC}"
    if [ "$PASSWORD_SET" = "1" ]; then
        echo "  Password      ${BOLD}set for this session${NC}"
    elif [ -n "${MYCOBOT_PASSWORD:-$(env_file_value MYCOBOT_PASSWORD)}" ]; then
        echo "  Password      ${BOLD}the saved one${NC} (src/backend/.env or MYCOBOT_PASSWORD)"
    fi
    if [ -z "$serial" ]; then
        :
    elif [ ! -e "$serial" ]; then
        warn "$serial does not exist. The backend will start without the arm (REST calls answer 503)."
    elif [ ! -r "$serial" ] || [ ! -w "$serial" ]; then
        warn "No permission to open $serial. Add yourself to its group and log in again: sudo usermod -aG $(stat -c %G "$serial") $USER"
    fi

    if [ -z "${MYCOBOT_PASSWORD:-$(env_file_value MYCOBOT_PASSWORD)}" ]; then
        warn "No password given, so the backend makes one up for this run (printed below)."
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
        echo "  Webapp        ${BOLD}http://$a:$HTTP_PORT/${NC}"
    done
    echo "  API docs      http://${addrs[0]}:$HTTP_PORT/docs"
    echo
}

# ---- arguments (in any order) ----
while [ $# -gt 0 ]; do
    case "$1" in
        backend)        shift ;;
        --ik|--solver)  [ $# -ge 2 ] || die "$1 needs native or ikpy"; IK="$2"; shift 2 ;;
        --ik=*|--solver=*) IK="${1#*=}"; shift ;;
        --password)     [ $# -ge 2 ] || die "--password needs a value"; PASSWORD="$2"; PASSWORD_SET=1; shift 2 ;;
        --password=*)   PASSWORD="${1#*=}"; PASSWORD_SET=1; shift ;;
        --no-prompt|-y) PROMPT=0; shift ;;
        --sim|--simulated) SIM=1; shift ;;
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
case "$SIM" in 1|true|yes|on|TRUE|YES) SIM=1 ;; *) SIM=0 ;; esac
[[ "$HTTP_PORT" =~ ^[0-9]+$ ]] || die "--http-port must be a number, not '$HTTP_PORT'"

echo "${BOLD}myCobot 280 backend${NC}"
setup_venv
offer_sim
choose_settings
preflight
print_urls

reload=()
# uvicorn only treats an exclude as a directory when it is an existing path, so pass them absolute
[ "$DEV" = "1" ] && reload=(--reload --reload-dir "$PROJECT_DIR" --reload-exclude "$VENV_DIR" --reload-exclude "$PROJECT_DIR/tools")
# exec: uvicorn replaces this shell, so Ctrl+C and service managers signal it directly
exec "$PYTHON" -m uvicorn main:app --app-dir "$BACKEND_DIR" --host "$HOST" --port "$HTTP_PORT" "${reload[@]}"
