#!/bin/bash
#
# A/B Test Script for gng_local_planner_rs
#
# This script runs navigation tests with and without dynamic obstacles
# to compare navigation performance.
#
# Usage:
#   ./run_ab_test.bash [goal_x] [goal_y] [timeout] [seed]
#
# Example:
#   ./run_ab_test.bash 3.0 0.0 60 42

set -e

# Default parameters
GOAL_X=${1:-3.0}
GOAL_Y=${2:-0.0}
TIMEOUT=${3:-60}
SEED=${4:-42}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "A/B Test: gng_local_planner_rs"
echo "=========================================="
echo "Goal: ($GOAL_X, $GOAL_Y)"
echo "Timeout: ${TIMEOUT}s"
echo "Obstacle seed: $SEED"
echo ""

# Cleanup function
cleanup() {
    echo "Cleaning up background processes..."
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gng_local_planner" 2>/dev/null || true
    sleep 2
}

trap cleanup EXIT

# Source ROS2 workspace
source /home/sarukiti/gng_ws/install/setup.bash

echo -e "${YELLOW}=== Test A: Without Obstacle ===${NC}"
echo ""

# Start simulation without obstacle
ros2 launch simple_diff_drive_sim sim.launch.py enable_obstacle:=false &
SIM_PID=$!
sleep 10

# Start planner
ros2 run gng_local_planner_rs gng_local_planner &
PLANNER_PID=$!
sleep 5

# Run navigation test
echo "Running navigation test..."
RESULT_A=$(ros2 run simple_diff_drive_sim navigation_test --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --timeout "$TIMEOUT" 2>/dev/null || echo '{"success": false, "error": "test_failed"}')

echo "Result A (no obstacle): $RESULT_A"

# Cleanup
kill $PLANNER_PID 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
sleep 5

echo ""
echo -e "${YELLOW}=== Test B: With Obstacle (seed=$SEED) ===${NC}"
echo ""

# Start simulation with obstacle
ros2 launch simple_diff_drive_sim sim.launch.py enable_obstacle:=true obstacle_seed:="$SEED" &
SIM_PID=$!
sleep 10

# Start planner
ros2 run gng_local_planner_rs gng_local_planner &
PLANNER_PID=$!
sleep 5

# Run navigation test
echo "Running navigation test..."
RESULT_B=$(ros2 run simple_diff_drive_sim navigation_test --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --timeout "$TIMEOUT" 2>/dev/null || echo '{"success": false, "error": "test_failed"}')

echo "Result B (with obstacle): $RESULT_B"

# Cleanup
kill $PLANNER_PID 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo -e "${GREEN}A/B Test Complete${NC}"
echo "=========================================="
echo ""
echo "Summary:"
echo "  Test A (no obstacle): $RESULT_A"
echo "  Test B (with obstacle, seed=$SEED): $RESULT_B"
echo ""

# Output combined JSON result
echo "Combined Result:"
echo "{\"test_a\": $RESULT_A, \"test_b\": $RESULT_B, \"goal\": [$GOAL_X, $GOAL_Y], \"seed\": $SEED}"
