#!/usr/bin/env bash

set -Eeuo pipefail

# Exemplary usage: ./launch_test.sh ./ball_holding/multi_robot_kid

if [ $# -lt 1 ]
then
    >&2 echo "Usage $0 <test_path> [--render]"
    exit 1
fi

if ! [ -d "$1" ]; then
    >&2 echo "Error: Expecting a folder for <test_path>"
    exit 1
fi

WEBOTS_OPTIONS=(--stdout --stderr --mode=fast)

if [ $# == 2 ]
then
    if [ "$2" != "--render" ]
    then
        >&2 echo "Unknown option $2"
        exit 1
    fi
else
    WEBOTS_OPTIONS+=(--no-rendering --minimize)
fi

if [ -z "${WEBOTS_HOME+x}" ]; then
  >&2 echo "Error: WEBOTS_HOME is not set"
  exit 1
fi

if [ -z "${JAVA_HOME+x}" ]; then
  >&2 echo "Error: JAVA_HOME is not set"
  exit 1
fi

if [ -z "${GAME_CONTROLLER_HOME+x}" ]; then
  >&2 echo "Error: GAME_CONTROLLER_HOME is not set"
  exit 1
fi

export PYTHONPATH="$WEBOTS_HOME/projects/samples/contests/robocup/controllers/referee"  # this should be removed once https://github.com/cyberbotics/webots/issues/3011 is fixed
# shellcheck disable=SC2003 # I don't care about this
if [ "$(expr substr "$(uname -s)" 1 10)" == "MSYS_NT-10" ]; then
  WEBOTS=webots
else
  WEBOTS=$WEBOTS_HOME/webots
fi

ROBOCUP_PATH=$WEBOTS_HOME/projects/samples/contests/robocup
WEBOTS_WORLD=$ROBOCUP_PATH/worlds/robocup.wbt

TEST_FOLDER=$(cd "$1" && pwd)
TEST_CLIENT=$ROBOCUP_PATH/controllers/player/test_client

# Automatically launch test clients, if applicable
if [[ -f "$TEST_FOLDER/clients.txt" ]]
then
    if pgrep test_client; then
        >&2 echo "Error: There are still clients running!"
        exit 1
    fi
    client_id=1
    while read -r line; do
        "$TEST_CLIENT" "$line" > "$TEST_FOLDER/client_$client_id.log" 2>&1 &
        ((client_id++))
    done < "$TEST_FOLDER/clients.txt"
fi

# Launch Webots
WEBOTS_ROBOCUP_TEST_SCENARIO=$TEST_FOLDER/test_scenario.json \
WEBOTS_ROBOCUP_GAME=$TEST_FOLDER/game.json \
"$WEBOTS" "${WEBOTS_OPTIONS[@]}" "$WEBOTS_WORLD"
