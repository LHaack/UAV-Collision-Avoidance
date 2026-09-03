# Rodar dentro de "UAV-Collision-Avoidance"

source macros.bash; clear; kgz; gnome-terminal --tab -- bash -c "cd ..; echo 'CWD:'; pwd; echo 'LS:'; ls -F; python3 enemies/spawn_enemies.py; exec bash"; sleep 2; setros && buildall && remodel && loadmission && sim -n 1 -c s -m