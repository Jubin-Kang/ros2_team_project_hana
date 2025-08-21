#!/bin/bash
# Robot startup script with domain support

set -e

# Get robot ID from argument or default to robot_1
ROBOT_ID=${1:-robot_1}

echo "🤖 Starting Robot: $ROBOT_ID"

# Set domain based on robot ID (fallback if config not available)
case $ROBOT_ID in
    robot_1)
        export ROS_DOMAIN_ID=18
        ;;
    robot_2)  
        export ROS_DOMAIN_ID=19
        ;;
    *)
        export ROS_DOMAIN_ID=18  # Default
        ;;
esac

echo "📡 Robot $ROBOT_ID running on ROS Domain: $ROS_DOMAIN_ID"

# Navigate to robot directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Start robot node
echo "🚀 Starting robot node..."
exec python3 robot_nodes/robot_node.py $ROBOT_ID