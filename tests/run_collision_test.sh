#!/bin/bash
#
# Teste de Collision Avoidance a partir de um .txtmap
# Uso: bash tests/run_collision_test.sh [arquivo.txtmap]
# Default: test.txtmap

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_DIR="$PROJECT_DIR/UAV-Collision-Avoidance"
PX4_DIR="/home/arthurgroll/Documents/estudos/IC/bolsa-ia-drones/PX4-Autopilot-ColAvoid"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
LOG_DIR="$PROJECT_DIR/logs"

# --- Args: [arquivo.txtmap] e flag opcional --px4-standard ---
#   --px4-standard  -> usa o baseline VFH+ vanilla (de livro) para comparacao
#   (sem a flag)     -> usa o nosso collision avoidance (VFH+ 3D turbinado)
TXTMAP=""
USE_VFH=0
for arg in "$@"; do
    case "$arg" in
        --px4-standard) USE_VFH=1 ;;
        -*) echo "AVISO: flag desconhecida '$arg' (a unica suportada e --px4-standard)" ;;
        *) [ -z "$TXTMAP" ] && TXTMAP="$arg" ;;
    esac
done
TXTMAP="${TXTMAP:-$PROJECT_DIR/_test.txtmap}"
export USE_VFH
MAP_NAME=$(basename "$TXTMAP" .txtmap)

MISSION_JSON="$SCRIPT_DIR/${MAP_NAME}_mission.json"
ENEMIES_JSON="$SCRIPT_DIR/${MAP_NAME}_enemies.json"
WORLD_SDF="$SCRIPT_DIR/${MAP_NAME}.sdf"
TEST_LAUNCH="$SCRIPT_DIR/test_launch.py"

FLIGHT_ALTITUDE=4.0

echo "=== Collision Avoidance Test ==="
echo "Map: $TXTMAP"
if [ "$USE_VFH" = "1" ]; then
    echo "Algoritmo: VFH+ vanilla (baseline, --px4-standard)"
else
    echo "Algoritmo: nosso (VFH+ 3D)"
fi
echo ""

# --------------------------------------------------
# Sudo: pede a senha UMA vez, de forma VISIVEL, e deixa em cache para os passos
# [0/6] e [1/6]. Sem isso, os 'sudo ... 2>/dev/null' abaixo escondem o prompt
# da senha e o script TRAVA esperando input invisivel.
# --------------------------------------------------
echo "[sudo] Pode pedir sua senha uma vez (limpeza de processos/permissoes)..."
if ! sudo -v; then
    echo "ERRO: sudo necessario para limpar processos/permissoes. Abortando."
    exit 1
fi

# --------------------------------------------------
# 0. Corrigir permissões
# --------------------------------------------------
echo "[0/6] Corrigindo permissões..."
sudo chown -R "$(whoami):$(whoami)" "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/install" "$LOG_DIR" 2>/dev/null || true
sudo chown -R "$(whoami):$(whoami)" "$WORKSPACE_DIR/log" 2>/dev/null || true
sudo chown -R "$(whoami):$(whoami)" "$PX4_DIR/Tools/simulation/gz/models" 2>/dev/null || true
sudo chown -R "$(whoami):$(whoami)" "$PX4_BUILD" 2>/dev/null || true
sudo rm -f /tmp/lidar_bridge_config_*.yaml /tmp/lidar_bridge_test_*.yaml 2>/dev/null || true
sudo rm -f /tmp/px4-sock-* 2>/dev/null || true
echo "       OK"

# --------------------------------------------------
# 1. Matar processos antigos
# --------------------------------------------------
echo "[1/6] Matando processos antigos..."
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
if pgrep -f MicroXRCEAgent > /dev/null 2>&1; then
    sudo -n pkill -9 -f MicroXRCEAgent 2>/dev/null || true
fi
if pgrep -f "gz sim" > /dev/null 2>&1; then
    sudo -n pkill -9 -f "gz sim" 2>/dev/null || true
fi
sleep 2
echo "       OK"

# --------------------------------------------------
# 2. Parse do mapa + rebuild
# --------------------------------------------------
echo "[2/6] Parseando mapa e buildando..."
cd "$PROJECT_DIR"
python3 tests/parse_txtmap.py "$TXTMAP" tests/

source /opt/ros/humble/setup.bash
cd "$WORKSPACE_DIR"
rm -rf build/px4_offboard install/px4_offboard
export GZ_VERSION=garden
colcon build --packages-select px4_offboard ros_gz_bridge ros_gz_interfaces --symlink-install 2>&1 | tail -3
source install/setup.bash

rsync -a --exclude='temp_repo' "$WORKSPACE_DIR/models/" "$PX4_DIR/Tools/simulation/gz/models/"
cp "$WORLD_SDF" "$PX4_DIR/Tools/simulation/gz/worlds/${MAP_NAME}.sdf"
echo "       OK"

# --------------------------------------------------
# 3. Iniciar MicroXRCEAgent + PX4 drones amigos + PX4 enemies
# --------------------------------------------------
echo "[3/6] Iniciando PX4..."
mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log

MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/microxrce.log" 2>&1 &
XRCE_PID=$!
echo "       MicroXRCEAgent PID: $XRCE_PID"

NUM_FRIENDLY=$(python3 -c "import json; print(len(json.load(open('$MISSION_JSON'))))")
NUM_ENEMIES=$(python3 -c "import json; print(len(json.load(open('$ENEMIES_JSON'))))")
ENEMY_START_INSTANCE=$((NUM_FRIENDLY + 2))

echo "       Drones amigos: $NUM_FRIENDLY | Enemies: $NUM_ENEMIES"

cd "$PX4_DIR"

# Spawnar drones amigos (instances 1..N)
for i in $(seq 0 $((NUM_FRIENDLY - 1))); do
    INSTANCE=$((i + 1))
    SPAWN=$(python3 -c "
import json
with open('$MISSION_JSON') as f:
    d = json.load(f)
s = d[$i]['spawn']
yaw = d[$i].get('spawn_yaw', 0.0)
print(f'{s[0]},{s[1]},{s[2]},0,0,{yaw}')
")
    echo "       Friendly $((i+1)): instance $INSTANCE at ($SPAWN)"
    PX4_SYS_AUTOSTART=4013 \
    PX4_GZ_WORLD=$MAP_NAME \
    PX4_GZ_MODEL_POSE="$SPAWN" \
    PX4_SIM_MODEL=gz_x500_lidar_3d \
    "$PX4_BUILD/bin/px4" -i $INSTANCE > "$LOG_DIR/px4_${INSTANCE}_friendly.log" 2>&1 &

    if [ $i -eq 0 ]; then
        echo "       Aguardando Gazebo (20s)..."
        sleep 20
    else
        sleep 3
    fi
done

# Spawnar enemies (instances ENEMY_START_INSTANCE..)
for i in $(seq 0 $((NUM_ENEMIES - 1))); do
    INSTANCE=$((ENEMY_START_INSTANCE + i))
    SPAWN=$(python3 -c "
import json
with open('$ENEMIES_JSON') as f:
    d = json.load(f)
s = d[$i]['spawn_pose']
yaw = d[$i].get('spawn_yaw', 0.0)
print(f'{s[0]},{s[1]},{s[2]},0,0,{yaw}')
")
    echo "       Enemy $((i+1)): instance $INSTANCE at ($SPAWN)"
    PX4_SYS_AUTOSTART=4013 \
    PX4_GZ_WORLD=$MAP_NAME \
    PX4_GZ_MODEL_POSE="$SPAWN" \
    PX4_SIM_MODEL=gz_x500_lidar_3d_enemy \
    "$PX4_BUILD/bin/px4" -i $INSTANCE > "$LOG_DIR/px4_${INSTANCE}_enemy.log" 2>&1 &
    sleep 3
done

echo "       Aguardando PX4 estabilizar (10s)..."
sleep 10

# --------------------------------------------------
# 4. Lançar controles
# --------------------------------------------------
echo "[4/6] Lançando controles..."
cd "$WORKSPACE_DIR"
source install/setup.bash

# Enemy MAVSDK
cd "$PROJECT_DIR"
python3 -u enemies/spawn_enemies.py --no-spawn --start-instance=$ENEMY_START_INSTANCE "$ENEMIES_JSON" > "$LOG_DIR/enemy_spawn.log" 2>&1 &
ENEMY_PID=$!
echo "       Enemy MAVSDK PID: $ENEMY_PID (start_instance=$ENEMY_START_INSTANCE)"

# Friendly ROS2
cd "$WORKSPACE_DIR"
ros2 launch "$TEST_LAUNCH" \
    mission_json:="$MISSION_JSON" \
    mission_mode:=true \
    world_name:="$MAP_NAME" &
LAUNCH_PID=$!
echo "       ROS2 Launch PID: $LAUNCH_PID"

echo ""
echo "=== Teste em execução ==="
echo "  Mapa:            $MAP_NAME"
echo "  Drones amigos:   $NUM_FRIENDLY (instances 1..$NUM_FRIENDLY)"
echo "  Enemies:         $NUM_ENEMIES (instances $ENEMY_START_INSTANCE..$((ENEMY_START_INSTANCE + NUM_ENEMIES - 1)))"
echo "  Altitude:        ${FLIGHT_ALTITUDE}m"
echo "  Logs em:         $LOG_DIR/"
echo "  Para encerrar:   Ctrl+C"
echo ""

cleanup() {
    echo ""
    echo "Encerrando processos..."
    kill $LAUNCH_PID $ENEMY_PID $XRCE_PID 2>/dev/null
    pkill -9 -f MicroXRCEAgent 2>/dev/null
    pkill -9 -f "bin/px4" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    echo "Encerrado."
}
trap cleanup EXIT INT TERM

wait $LAUNCH_PID
