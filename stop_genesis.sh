#!/bin/bash
# Stop the Genesis simulation
tmux kill-session -t g1_sim 2>/dev/null && echo "Genesis simulation stopped." || echo "No simulation running."
