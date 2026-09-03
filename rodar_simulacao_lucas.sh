OBS: RODAR DENTRO DA PASTA "UAV-Collision-Avoidance" daqui de dentro (o caminho, a partir de onde se encontra esse script, é: ./UAV-Collision-Avoidance)

source macros.bash; clear; kgz; # gnome-terminal --tab -- bash -c "cd .. && source ./run.sh"; sleep 2; 
setros && buildall && remodel && loadmission && sim -n 1 -c s -m