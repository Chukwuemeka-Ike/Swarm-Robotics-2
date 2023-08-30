#!/usr/bin/bash
# Run this to ensure the shell has the correct env variables for gazebo.

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo "Running simulation from $SCRIPT_DIR"

export GAZEBO_MODEL_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export GAZEBO_RESOURCE_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds
export SDF_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/models
export IGN_FILE_PATH=$SCRIPT_DIR/src/AssistiveRobot-SimulationFiles/lab_gazebo/worlds

source $SCRIPT_DIR/devel/setup.bash

# Roslaunches all the relevant nodes
roslaunch swarm2_launch all_sim.launch