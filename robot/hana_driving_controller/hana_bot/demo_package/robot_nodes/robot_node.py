#!/usr/bin/env python3
# Fleet-compatible robot node with domain support
import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import os
import yaml

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from robot.nav2_waypoint_class import WaypointNavigator
from robot.fsm import DeliveryFSM

def load_fleet_config():
    """Load fleet configuration from YAML file"""
    # Go up two levels to reach project root from mobile_robot/robot_nodes/
    config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'fleet_config.yaml')
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Config file not found: {config_path}")
        return None

def set_robot_domain(robot_id, config):
    """Set ROS domain ID for robot based on configuration"""
    if config and 'robots' in config and robot_id in config['robots']:
        domain_id = config['robots'][robot_id]['domain_id']
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        print(f"Robot {robot_id} running on ROS domain {domain_id}")
        return domain_id
    else:
        # Default domain assignment based on user info
        # robot_1 -> domain 18, robot_2 -> domain 19
        domain_map = {'robot_1': 18, 'robot_2': 19}
        domain_id = domain_map.get(robot_id, 18)
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        print(f"Robot {robot_id} running on default domain {domain_id}")
        return domain_id

def main():
    # Get robot ID from command line argument or default to robot_1
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "robot_1"
    
    print(f"Starting robot node: {robot_id}")
    
    # Load configuration and set domain
    config = load_fleet_config()
    domain_id = set_robot_domain(robot_id, config)
    
    # Initialize ROS
    rclpy.init()
    
    # Create robot components with namespace
    wp_nav = WaypointNavigator()
    wp_nav._node_name = f"{robot_id}_waypoint_navigator"
    
    # FSM will get service station coordinates dynamically per task
    fsm = DeliveryFSM(robot_id, wp_nav)
    
    # Create executor and add nodes
    executor = MultiThreadedExecutor()
    executor.add_node(wp_nav)
    executor.add_node(fsm)
    
    try:
        print(f"Robot {robot_id} is ready on domain {domain_id}!")
        print(f"Listening on topics: /{robot_id}/user_cmd, /{robot_id}/status")
        executor.spin()
    except KeyboardInterrupt:
        print(f"Shutting down robot {robot_id}")
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()