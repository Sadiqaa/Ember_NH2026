#!/bin/zsh

# Compatibility wrapper. The maintained H.264 bridge supersedes the raw mode.
set -eu
readonly SCRIPT_DIR=${0:A:h}
exec "$SCRIPT_DIR/../../Live_Camera.command"
