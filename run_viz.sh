#!/bin/bash
# Script to run the 3D Lidar Visualizer with ROS 2 environment loaded

# Check if ROS 2 Humble is installed in standard path
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: /opt/ros/humble/setup.bash not found. Ensure ROS 2 is installed or sourced."
fi

# Run the python script
# Using 'python' assumes the virtual environment is active or 'python' resolves to the correct interpreter
echo "Starting Lidar 3D Visualizer..."
python lidar_3d_viz.py
