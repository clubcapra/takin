#!/bin/bash

export TAKIN_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
export TAKIN_HOME=$TAKIN_DIR

#export GAZEBO_MODEL_PATH=$URIAL_DIR/src/capra_gazebo/models

alias takin='cd $TAKIN_HOME'

source /opt/ros/kinetic/setup.bash

source $TAKIN_HOME/devel/setup.bash

source $TAKIN_HOME/vendor/devel/setup.bash --extend

