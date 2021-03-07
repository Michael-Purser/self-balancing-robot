#!/bin/bash

export ROSCONSOLE_FORMAT='[${severity}] [${walltime}][${thread}][${node}][${function}:${line}]: ${message}'

CURRENT_DIR="$(dirname "${BASH_SOURCE[0]}")"
PROJECT_DIR_NAME="self_balancing_robot"

PROJECT_DIR=$CURRENT_DIR
while [ "$(basename "$PROJECT_DIR")" != "$PROJECT_DIR_NAME" ] && [ "$PROJECT_DIR" != "/" ];
do
    PROJECT_DIR="$(dirname "$PROJECT_DIR")"
done

if [ "$PROJECT_DIR" == "/" ];
then
    echo "ERROR: During execution of self_balancing_robot setup.bash, could not find directory '"$PROJECT_DIR_NAME"/' in the script's parent tree."
    echo "CONSEQUENCE: ROS_WORKSPACE has not been set."
    echo "CONSEQUENCE: Project aliases have not been sourced."
    echo "To solve these issues, make sure you have followed the setup instructions correctly and that you have not made any typos."
else
    export ROS_WORKSPACE="$PROJECT_DIR/src"
    source "$CURRENT_DIR/aliases.sh"
fi