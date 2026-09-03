gnome-terminal --tab -- bash -c "cd ../PX4-Autopilot-ColAvoid && PX4_GZ_WORLD=\"lawn\" PX4_GZ_MODEL_POSE=\"15.0,0.0\" PX4_SIM_INSTANCE=2 make px4_sitl gz_omnicopter"
sleep 12
python3 omnicopter_2.py
