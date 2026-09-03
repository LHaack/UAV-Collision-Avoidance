function setros() {
    source /opt/ros/humble/setup.bash
}

function setup() {
    source install/setup.bash
}




function clean_all() {
    rm -rf build/px4_offboard install/px4_offboard
    rm -rf build/px4_msgs install/px4_msgs
    echo "Cleaned build artifacts for px4_offboard and px4_msgs (FULL)"
}

function clean_offboard() {
    rm -rf build/px4_offboard install/px4_offboard
    echo "Cleaned px4_offboard build artifacts (FAST)"
}

function buildall() {
    clean_all
    colcon build --symlink-install && setup
}

function build() {
    clean_offboard
    colcon build --packages-select px4_offboard --symlink-install && setup
}

function remodel() {
    cp -r models/* ../../PX4-Autopilot-ColAvoid/Tools/simulation/gz/models/
}

function sim() {
    OPTIND=1

    local uav_number=1
    local spawn_configuration="l"
    local mission_mode="false"

    while getopts "n:c:m" opt; do
        case $opt in
            n) uav_number="$OPTARG" ;;
            c) spawn_configuration="$OPTARG" ;;
            m) mission_mode="true" ;;
        esac
    done

    ros2 launch px4_offboard offboard_velocity_control.launch.py \
        uav_number:=$uav_number \
        spawn_configuration:=$spawn_configuration \
        mission_mode:=$mission_mode
}


function loadmission() {
    # Get workspace root from script location (robust against cwd)
    # Note: When sourced, BASH_SOURCE[0] is the script path.
    local WORKSPACE_ROOT
    WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    local SRC="$WORKSPACE_ROOT/src/src_codes/launch/mission.json"
    local DEST="$WORKSPACE_ROOT/install/px4_offboard/share/px4_offboard/mission.json"
    
    # Create valid symlink
    ln -sf "$SRC" "$DEST"
    echo "Mission linked: $SRC -> $DEST"
}

function kgz() {
    ps aux | grep -E 'gz|px4|MicroXRCEAgent|ruby' | grep -v grep | awk '{print $2}' | xargs -r kill -9
    pkill -9 -f px4
    pkill -9 -f MicroXRCEAgent
}
