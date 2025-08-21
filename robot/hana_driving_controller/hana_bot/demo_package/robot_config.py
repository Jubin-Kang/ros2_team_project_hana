# Robot Configuration
# This file contains robot-specific settings that don't require database access
# Used for distributed deployment where robots run on separate machines

# ROS2 Topics
ROS_CMD_TOPIC   = "user_cmd"
ROS_STAT_TOPIC  = "status"

# Default waypoints (will be overridden by coordinates from Fleet Manager)
PICKUP_ST1     = [0.33, -0.33, 0.707]  # Arm pickup station
SERVICE_ST1    = [0.1, 0.78, -0.707]   # Default service station (fallback)
CHARGING_ST    = [0.0, 0.0, 1.0]       # Charging dock

# Fleet Manager connection (for distributed deployment)
FLEET_MANAGER_HOST = "localhost"  # Change this to Fleet Manager's IP
ROS_DOMAIN_ID = 0  # Set same domain ID across all machines

# Robot camera configuration (per robot)
ROBOT_CAMERA_PORT = 5000  # Default port, can be overridden per robot