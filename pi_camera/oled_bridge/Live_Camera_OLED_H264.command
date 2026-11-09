#!/bin/zsh

# Compatibility wrapper. The preferred service now owns both camera outputs.
set -eu
readonly SCRIPT_DIR=${0:A:h}
exec "$SCRIPT_DIR/../../Live_Camera.command"
