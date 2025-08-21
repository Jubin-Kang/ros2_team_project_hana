#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json
import time
from typing import Dict, List
import threading
import yaml
import subprocess

# Import shared message definitions
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from shared.fleet_msgs import TaskRequest, TaskResponse, RobotState, TaskStatus, RobotStatus, TaskType

# Add web directory to path for database access
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'web'))
from task_db import get_next_pending_task, update_task_status, complete_task, get_robot_current_task, get_timeout_tasks, fail_task
from db import query_db, update_db

# Note: Using file-based communication for fall detection alerts

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        
        # Load configuration
        self.config = self.load_fleet_config()
        
        # Set central domain (Fleet Manager runs on Domain 0)
        os.environ['ROS_DOMAIN_ID'] = str(self.config['central_domain'])
        
        # Robot management (tasks now managed in database)
        self.robots: Dict[str, RobotState] = {}
        
        # Robot ID mapping (for database integration)
        self.robot_name_to_id = {}
        
        # Initialize robots from config
        for robot_name, robot_config in self.config['robots'].items():
            self.robot_name_to_id[robot_name] = robot_config['hana_bot_id']
            self.robots[robot_name] = RobotState(
                robot_id=robot_name,
                status=RobotStatus.IDLE
            )
            
        # Ensure robots exist in hana_bots table
        self._ensure_robots_in_db()
            
        # Domain Bridge process handle
        self.domain_bridge_process = None
        
        # Fleet-level communication
        self.task_request_sub = self.create_subscription(
            String, '/fleet/task_request', self.handle_task_request, 10)
        self.task_response_pub = self.create_publisher(
            String, '/fleet/task_response', 10)
            
        # Robot communication (per robot)
        self.robot_cmd_pubs = {}
        self.robot_status_subs = {}
        
        # Setup robot communication
        self.setup_robot_communication()
        
        # Initialize robot status for all robots
        for robot_id in self.robots.keys():
            self.robots[robot_id].last_update = time.time()
        
        # Status monitoring timer
        self.create_timer(1.0, self.monitor_fleet)
        
        # Task assignment timer (check for pending tasks every 2 seconds)
        self.create_timer(2.0, self.check_and_assign_tasks)
        
        # Task timeout checker (check for timeout tasks every 30 seconds)
        self.create_timer(30.0, self.check_task_timeouts)
        
        # Robot status broadcast (for web frontend)
        self.robot_status_pub = self.create_publisher(String, '/fleet/robot_status', 10)
        
        # Recovery: Reset stuck 'assigned' tasks on startup (one-time)
        self.recovery_timer = self.create_timer(5.0, self.recover_stuck_assignments)
        
        # Fall detection subscription
        self.fall_detection_sub = self.create_subscription(
            String, '/fall_detection_topic', self.handle_fall_detection, 10)
        
        # Fall detection alert state
        self.fall_alert_active = False
        self.fall_alert_timestamp = None
        
        # Battery status subscriptions
        self.battery_status = {
            "DP_09": {"level": 0, "timestamp": time.time()},  # Robot 1
            "DP_03": {"level": 0, "timestamp": time.time()}   # Robot 2
        }
        
        # Subscribe to battery topics
        self.battery_dp09_sub = self.create_subscription(
            Float32, '/DP_09/battery_present', 
            lambda msg: self.handle_battery_status(msg, "DP_09"), 10)
        self.battery_dp03_sub = self.create_subscription(
            Float32, '/DP_03/battery_present', 
            lambda msg: self.handle_battery_status(msg, "DP_03"), 10)
        
        self.get_logger().info("Fleet Manager initialized")
    
    def _sync_robot_states_with_db(self):
        """Synchronize robot states with database to prevent double assignments"""
        try:
            for robot_id, robot_state in self.robots.items():
                robot_db_id = self.robot_name_to_id.get(robot_id)
                if not robot_db_id:
                    continue
                
                # Get current task from database
                current_db_task = get_robot_current_task(robot_db_id)
                
                if current_db_task:
                    # Robot has task in database - update local state
                    if robot_state.current_task_id != str(current_db_task['task_id']):
                        self.get_logger().info(f"SYNC: {robot_id} DB task {current_db_task['task_id']} != local {robot_state.current_task_id}")
                        robot_state.current_task_id = str(current_db_task['task_id'])
                        robot_state.status = RobotStatus.BUSY
                else:
                    # No task in database - robot should be idle if not currently working
                    if robot_state.current_task_id and robot_state.status == RobotStatus.IDLE:
                        self.get_logger().info(f"SYNC: {robot_id} clearing completed task {robot_state.current_task_id}")
                        robot_state.current_task_id = None
                        
        except Exception as e:
            self.get_logger().error(f"Error syncing robot states: {e}")
    
    def recover_stuck_assignments(self):
        """Reset stuck 'assigned' tasks back to 'pending' for reprocessing"""
        try:
            # Find tasks that are 'assigned' but robot is idle (stuck assignments)
            with query_db() as cur:
                cur.execute("""
                    SELECT task_id, assigned_bot_id 
                    FROM tasks 
                    WHERE status = '할당'
                    AND created_at < NOW() - INTERVAL '2 minutes'
                """)
                stuck_tasks = cur.fetchall()
                
            # Process each stuck task
            for task_id, bot_id in stuck_tasks:
                # Check if robot is actually idle
                robot_name = None
                for rname, rid in self.robot_name_to_id.items():
                    if rid == bot_id:
                        robot_name = rname
                        break
                
                if robot_name and self.robots[robot_name].status == RobotStatus.IDLE:
                    self.get_logger().warn(f"🔄 Recovering stuck assignment: Task {task_id}")
                    # Reset to pending for reprocessing
                    with update_db() as cur:
                        cur.execute("""
                            UPDATE tasks 
                            SET status = '대기', assigned_bot_id = NULL
                            WHERE task_id = %s
                        """, (task_id,))
                    
                    # Clear local state
                    self.robots[robot_name].current_task_id = None
                        
        except Exception as e:
            self.get_logger().error(f"Error recovering stuck assignments: {e}")
        
        # Only run once at startup, then disable
        if hasattr(self, 'recovery_timer'):
            self.destroy_timer(self.recovery_timer)
            self.recovery_timer = None
    
    def setup_robot_communication(self):
        """Setup communication channels for each robot"""
        for robot_id in self.robots.keys():
            # Command publisher to robot (matches robot FSM topic)
            self.robot_cmd_pubs[robot_id] = self.create_publisher(
                String, f'/{robot_id}/user_cmd', 10)
            
            # Status subscriber from robot (matches robot FSM topic)
            self.robot_status_subs[robot_id] = self.create_subscription(
                String, f'/{robot_id}/status', 
                lambda msg, rid=robot_id: self.handle_robot_status(msg, rid), 10)
            
            self.get_logger().info(f"Setup communication for {robot_id}: /{robot_id}/user_cmd, /{robot_id}/status")
    
    def handle_task_request(self, msg: String):
        """Handle incoming task requests from web server"""
        try:
            data = json.loads(msg.data)
            db_task_id = data.get('db_task_id')
            
            self.get_logger().info(f"Received task request: {data['task_id']} (DB ID: {db_task_id})")
            
            # Task is already stored in database, just send acknowledgment
            response = TaskResponse(
                task_id=data['task_id'],
                status=TaskStatus.PENDING,
                message="Task queued successfully"
            )
            self.send_task_response(response)
            
            # Try to assign pending tasks immediately
            self.check_and_assign_tasks()
            
        except Exception as e:
            self.get_logger().error(f"Error handling task request: {e}")
    
    def check_and_assign_tasks(self):
        """Check for pending tasks and assign to available robots (FIFO)"""
        try:
            # First, synchronize robot states with database to avoid double assignments
            self._sync_robot_states_with_db()
            
            # Find available robots (IDLE and not having any current task in DB)
            available_robots = []
            for robot_id, robot_state in self.robots.items():
                robot_db_id = self.robot_name_to_id.get(robot_id)
                if not robot_db_id:
                    continue
                    
                # Check database for current task assignment
                current_db_task = get_robot_current_task(robot_db_id)
                
                if current_db_task:
                    # Robot has an active task in database - NOT available
                    self.get_logger().info(f"DEBUG: {robot_id} has active DB task {current_db_task['task_id']} - NOT available")
                    # Update local state to match database
                    self.robots[robot_id].status = RobotStatus.BUSY
                    self.robots[robot_id].current_task_id = str(current_db_task['task_id'])
                elif robot_state.status == RobotStatus.IDLE:
                    # Robot is idle and has no database task - available
                    available_robots.append(robot_id)
                    self.get_logger().info(f"DEBUG: {robot_id} is IDLE and no DB task - available")
            
            self.get_logger().info(f"DEBUG: Checking tasks - available_robots: {available_robots}")
            
            if not available_robots:
                self.get_logger().info("DEBUG: No available robots")
                return  # No available robots
            
            # Get next pending task from database (FIFO order)
            next_task = get_next_pending_task()
            self.get_logger().info(f"DEBUG: Next pending task: {next_task}")
            
            if not next_task:
                self.get_logger().info("DEBUG: No pending tasks found")
                return  # No pending tasks
            
            # Assign to first available robot
            robot_id = available_robots[0]
            robot_db_id = self.robot_name_to_id[robot_id]
            
            # Double-check: make sure this robot still doesn't have a task (race condition prevention)
            double_check_task = get_robot_current_task(robot_db_id)
            if double_check_task:
                self.get_logger().warn(f"RACE CONDITION PREVENTED: {robot_id} got task {double_check_task['task_id']} between checks")
                return
            
            # Update database first (atomically assign task)
            update_task_status(next_task['task_id'], 'assigned', robot_db_id)
            
            # Update robot state
            self.robots[robot_id].status = RobotStatus.BUSY
            self.robots[robot_id].current_task_id = str(next_task['task_id'])
            
            # Send command to robot with resident_id, task_type, item_id, and coordinates
            if next_task['task_type'] in ['배달', '호출']:
                # Get target coordinates and location name for the resident
                target_coords, location_name = self._get_service_station_coords(next_task['requester_resident_id'])
                
                cmd_data = {
                    "command": "order",
                    "resident_id": next_task['requester_resident_id'],
                    "task_type": next_task['task_type'],
                    "item_id": next_task.get('item_id'),  # Add item_id for robot arm
                    "target_coordinates": target_coords,
                    "location_name": location_name  # Add location name for parking lookup
                }
                cmd_msg = String(data=json.dumps(cmd_data))
                self.robot_cmd_pubs[robot_id].publish(cmd_msg)
                
                # Broadcast updated status to web frontend immediately after assignment
                # Use the actual robot status that will be set by FSM
                expected_status = "moving_to_user" if next_task['task_type'] == "호출" else "moving_to_arm"
                self.broadcast_robot_status(robot_id, expected_status, str(next_task['task_id']))
                
                self.get_logger().info(f"Assigned {next_task['task_type']} task {next_task['task_id']} to {robot_id} for resident {next_task['requester_resident_id']}")
            
        except Exception as e:
            self.get_logger().error(f"Error in task assignment: {e}")
    
    
    def handle_robot_status(self, msg: String, robot_id: str):
        """Handle status updates from robots"""
        status_text = msg.data
        self.get_logger().info(f"Robot {robot_id} status: {status_text}")
        
        robot_state = self.robots[robot_id]
        previous_status = robot_state.status
        
        # Update robot status based on message
        if status_text == "idle":
            robot_state.status = RobotStatus.IDLE
            # Note: Task completion is now handled in handle_confirm_request()
            # idle means robot returned to dock after completing all tasks
            
        elif status_text in ["moving_to_arm", "moving_to_user"]:
            robot_state.status = RobotStatus.BUSY
            
            # Update task status in database to 이동중
            if robot_state.current_task_id:
                try:
                    with update_db() as cur:
                        cur.execute("""
                            UPDATE tasks 
                            SET status = '이동중'
                            WHERE task_id = %s
                        """, (int(robot_state.current_task_id),))
                    self.get_logger().info(f"Updated task {robot_state.current_task_id} to 이동중 status")
                except Exception as e:
                    self.get_logger().error(f"Error updating task to moving: {e}")
        
        elif status_text == "picking":
            robot_state.status = RobotStatus.BUSY
            
            # Update task status in database to 집기중  
            if robot_state.current_task_id:
                try:
                    with update_db() as cur:
                        cur.execute("""
                            UPDATE tasks 
                            SET status = '집기중'
                            WHERE task_id = %s
                        """, (int(robot_state.current_task_id),))
                    self.get_logger().info(f"Updated task {robot_state.current_task_id} to 집기중 status")
                except Exception as e:
                    self.get_logger().error(f"Error updating task to picking: {e}")
        
        elif status_text == "waiting_confirm":
            robot_state.status = RobotStatus.BUSY
            
            # Robot arrived at user location - update DB to 수령대기 
            if robot_state.current_task_id:
                try:
                    # Map waiting_confirm to Korean DB status
                    with update_db() as cur:
                        cur.execute("""
                            UPDATE tasks 
                            SET status = '수령대기'
                            WHERE task_id = %s
                        """, (int(robot_state.current_task_id),))
                    self.get_logger().info(f"Updated task {robot_state.current_task_id} to 수령대기 status")
                except Exception as e:
                    self.get_logger().error(f"Error updating task to waiting_confirm: {e}")
        
        elif status_text == "returning_to_dock":
            robot_state.status = RobotStatus.BUSY
            # Clear current_task_id when returning to dock - task is complete
            if robot_state.current_task_id:
                self.get_logger().info(f"Clearing current_task_id {robot_state.current_task_id} as robot returns to dock")
                robot_state.current_task_id = None
        
        elif status_text == "navigation_failed":
            self.get_logger().error(f"Robot {robot_id} navigation failed!")
            self._handle_robot_failure(robot_id, robot_state)
        
        robot_state.last_update = time.time()
        
        # Update robot status in database
        self._update_robot_db_status(robot_id, status_text)
        
        # Broadcast robot status to web frontend
        self.broadcast_robot_status(robot_id, status_text, robot_state.current_task_id)
    
    def _handle_robot_failure(self, robot_id: str, robot_state):
        """Handle robot navigation failure"""
        try:
            # Mark current task as failed in database
            if robot_state.current_task_id:
                task_id = int(robot_state.current_task_id)
                self.get_logger().warn(f"Marking task {task_id} as failed due to navigation failure")
                fail_task(task_id, "Navigation failure")
                robot_state.current_task_id = None
            
            # Reset robot to idle
            robot_state.status = RobotStatus.IDLE
            # Update DB status
            self._update_robot_db_status(robot_id, "idle")
            
            # Try to assign next task if available
            self.get_logger().info("Attempting to assign next task after failure recovery")
            self.check_and_assign_tasks()
            
        except Exception as e:
            self.get_logger().error(f"Error handling robot failure: {e}")
    
    def handle_confirm_request(self, msg: String):
        """Handle confirmation requests from web server"""
        try:
            data = json.loads(msg.data)
            robot_id = data.get('robot_id')
            if not robot_id:
                # Find robot with waiting_confirm status if not specified
                for rid, robot_state in self.robots.items():
                    if robot_state.status == RobotStatus.BUSY and robot_state.current_task_id:
                        robot_id = rid
                        break
                if not robot_id:
                    robot_id = list(self.robots.keys())[0]  # Fallback to first robot
            
            robot_state = self.robots[robot_id]
            
            # Complete current task
            if robot_state.current_task_id:
                try:
                    complete_task(int(robot_state.current_task_id))
                    self.get_logger().info(f"Completed task {robot_state.current_task_id} on confirm")
                    robot_state.current_task_id = None
                except Exception as e:
                    self.get_logger().error(f"Error completing task on confirm: {e}")
            
            # Check if there's a next pending task
            next_task = get_next_pending_task()
            
            if next_task:
                # Assign next task immediately - robot goes directly to arm
                robot_db_id = self.robot_name_to_id[robot_id]
                update_task_status(next_task['task_id'], 'assigned', robot_db_id)
                
                robot_state.status = RobotStatus.BUSY
                robot_state.current_task_id = str(next_task['task_id'])
                
                # Send order command with resident_id, task_type, item_id, coordinates, and location name
                target_coords, location_name = self._get_service_station_coords(next_task['requester_resident_id'])
                
                cmd_data = {
                    "command": "order",
                    "resident_id": next_task['requester_resident_id'],
                    "task_type": next_task['task_type'],
                    "item_id": next_task.get('item_id'),  # Add item_id for robot arm
                    "target_coordinates": target_coords,
                    "location_name": location_name  # Add location name for parking lookup
                }
                cmd_msg = String(data=json.dumps(cmd_data))
                self.robot_cmd_pubs[robot_id].publish(cmd_msg)
                
                # Broadcast updated status to web frontend immediately after assignment
                # Use the actual robot status that will be set by FSM
                expected_status = "moving_to_user" if next_task['task_type'] == "호출" else "moving_to_arm"
                self.broadcast_robot_status(robot_id, expected_status, str(next_task['task_id']))
                
                self.get_logger().info(f"Assigned next {next_task['task_type']} task {next_task['task_id']} to {robot_id} for resident {next_task['requester_resident_id']} immediately after confirm")
            else:
                # No more tasks - send confirm to return to dock
                cmd_msg = String(data="confirm")
                self.robot_cmd_pubs[robot_id].publish(cmd_msg)
                self.get_logger().info(f"No more tasks, sent confirm to {robot_id} to return to dock")
                
        except Exception as e:
            self.get_logger().error(f"Error handling confirm request: {e}")
    
    def broadcast_robot_status(self, robot_id: str, status: str, task_id: str = None):
        """Broadcast robot status to web frontend"""
        try:
            status_data = {
                'robot_id': robot_id,
                'status': status,
                'current_task_id': task_id,
                'timestamp': time.time()
            }
            msg = String(data=json.dumps(status_data))
            self.robot_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error broadcasting robot status: {e}")
    
    def send_task_response(self, response: TaskResponse):
        """Send task response back to web server"""
        response_data = {
            'task_id': response.task_id,
            'status': response.status.value,
            'robot_id': response.robot_id,
            'message': response.message
        }
        msg = String(data=json.dumps(response_data))
        self.task_response_pub.publish(msg)
    
    def monitor_fleet(self):
        """Periodic fleet monitoring"""
        # Check for stale robot states (5 second timeout since heartbeat is every 2 seconds)
        current_time = time.time()
        for robot_id, robot_state in self.robots.items():
            if current_time - robot_state.last_update > 5.0:  # 5 seconds timeout
                if robot_state.status != RobotStatus.ERROR:
                    self.get_logger().warn(f"Robot {robot_id} appears offline")
                    robot_state.status = RobotStatus.ERROR
            elif robot_state.status == RobotStatus.ERROR and current_time - robot_state.last_update < 3.0:
                # Robot came back online
                self.get_logger().info(f"Robot {robot_id} is back online")
                robot_state.status = RobotStatus.IDLE
    
    def check_task_timeouts(self):
        """Check for tasks that have exceeded timeout and mark them as failed"""
        try:
            timeout_tasks = get_timeout_tasks(timeout_minutes=5)
            
            for task in timeout_tasks:
                task_id = task['task_id']
                minutes_elapsed = task['minutes_elapsed']
                robot_id = task['assigned_bot_id']
                
                self.get_logger().warn(f"Task {task_id} has timed out after {minutes_elapsed:.1f} minutes")
                
                # Reset task to pending state instead of failing it
                update_task_status(task_id, 'pending')
                
                # Reset robot to idle if it was assigned to this task
                for rid, robot_state in self.robots.items():
                    if robot_state.current_task_id == str(task_id):
                        robot_state.current_task_id = None
                        robot_state.status = RobotStatus.IDLE
                        self.get_logger().info(f"Reset {rid} to idle due to task timeout")
                        
                        # Send stop command to robot
                        try:
                            cmd_msg = String(data="stop")
                            if rid in self.robot_cmd_pubs:
                                self.robot_cmd_pubs[rid].publish(cmd_msg)
                                self.get_logger().info(f"Sent stop command to {rid} due to timeout")
                        except Exception as e:
                            self.get_logger().error(f"Failed to send stop command to {rid}: {e}")
                        
                        # Broadcast updated status
                        self.broadcast_robot_status(rid, "idle")
                        break
                
                self.get_logger().info(f"Task {task_id} reset to pending due to timeout")
            
            if timeout_tasks:
                # Try to assign new tasks after handling timeouts
                self.check_and_assign_tasks()
                
        except Exception as e:
            self.get_logger().error(f"Error checking task timeouts: {e}")
        
        # Debug: Log fleet status periodically 
        current_time = time.time()
        if int(current_time) % 5 == 0:  # Every 5 seconds
            for robot_id, robot_state in self.robots.items():
                self.get_logger().info(f"Robot {robot_id}: {robot_state.status.value}, last_update: {current_time - robot_state.last_update:.1f}s ago")
    
    def _get_service_station_coords(self, resident_id):
        """Get service station coordinates for the given resident"""
        try:
            with query_db() as cur:
                cur.execute("""
                    SELECT l.coordinates, l.location_name 
                    FROM residents r 
                    JOIN locations l ON r.service_station_id = l.location_id 
                    WHERE r.resident_id = %s
                """, (resident_id,))
                result = cur.fetchone()
                
                if result:
                    coordinates = result[0]  # PostgreSQL array format: {x,y,z}
                    location_name = result[1]  # e.g., "방1", "방2"
                    return self._parse_coordinates(coordinates), location_name
                else:
                    self.get_logger().warn(f"No service station found for resident {resident_id}, using default coordinates")
                    # return [0.1, 0.78, -0.707]  # Default SERVICE_ST1 coordinates
                    return None, None
        except Exception as e:
            self.get_logger().error(f"Error getting service station for resident {resident_id}: {e}, using default coordinates")
            # return [0.1, 0.78, -0.707]  # Fallback to default coordinates
            return None, None
    
    def _parse_coordinates(self, coords):
        """Parse PostgreSQL array format {x,y,z} to Python list [x,y,z]"""
        if isinstance(coords, list):
            return coords
        if isinstance(coords, str):
            # Remove braces and split by comma
            clean_coords = coords.strip('{}')
            return [float(x.strip()) for x in clean_coords.split(',')]
        return coords
    
    def _map_ros_status_to_db_status(self, ros_status: str) -> str:
        """Map ROS status to database hana_bots status"""
        status_map = {
            "idle": "대기중",
            "moving_to_arm": "작업중",
            "moving_to_user": "작업중", 
            "picking": "작업중",
            "waiting_confirm": "작업중",
            "returning_to_dock": "복귀중",
            "navigation_failed": "오프라인",
            "error": "오프라인"
        }
        return status_map.get(ros_status, "대기중")
    
    def _update_robot_db_status(self, robot_id: str, ros_status: str):
        """Update robot status in hana_bots table"""
        self.get_logger().info(f"🔄 _update_robot_db_status CALLED: {robot_id} → {ros_status}")
        try:
            robot_db_id = self.robot_name_to_id.get(robot_id)
            if not robot_db_id:
                self.get_logger().warn(f"No DB ID mapping for robot {robot_id}")
                return
                
            db_status = self._map_ros_status_to_db_status(ros_status)
            
            with update_db() as cur:
                cur.execute("""
                    UPDATE hana_bots 
                    SET status = %s 
                    WHERE hana_bot_id = %s
                """, (db_status, robot_db_id))
                
                if cur.rowcount > 0:
                    self.get_logger().info(f"📊 Updated {robot_id} (ID:{robot_db_id}) status: {ros_status} → {db_status}")
                else:
                    self.get_logger().warn(f"No robot found with ID {robot_db_id} in hana_bots table")
                    
        except Exception as e:
            self.get_logger().error(f"Failed to update robot {robot_id} DB status: {e}")
    
    def _ensure_robots_in_db(self):
        """Ensure all configured robots exist in hana_bots table"""
        try:
            for robot_name, robot_db_id in self.robot_name_to_id.items():
                # Check if robot exists (read-only)
                with query_db() as cur:
                    cur.execute("SELECT hana_bot_id FROM hana_bots WHERE hana_bot_id = %s", (robot_db_id,))
                    existing = cur.fetchone()
                
                if not existing:
                    # Insert new robot (write operation)
                    with update_db() as cur:
                        friendly_name = f"HANA ROBOT {robot_db_id}" if robot_db_id != 8 else "HANA PINKY"
                        
                        cur.execute("""
                            INSERT INTO hana_bots (hana_bot_id, bot_name, battery, status)
                            VALUES (%s, %s, %s, %s)
                            ON CONFLICT (hana_bot_id) DO UPDATE SET
                                bot_name = EXCLUDED.bot_name,
                                status = EXCLUDED.status
                        """, (robot_db_id, friendly_name, 100, "대기중"))
                        
                    self.get_logger().info(f"✅ Ensured {robot_name} (ID:{robot_db_id}) exists in hana_bots table as '{friendly_name}'")
                else:
                    self.get_logger().info(f"✓ {robot_name} (ID:{robot_db_id}) already exists in hana_bots table")
                        
        except Exception as e:
            self.get_logger().error(f"Failed to ensure robots in DB: {e}")
    
    def load_fleet_config(self):
        """Load fleet configuration from YAML file"""
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'fleet_config.yaml')
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_path}")
            # Return default config for single robot
            return {
                'central_domain': 129,
                'robots': {
                    'robot_1': {
                        'domain_id': 18,  # Correct domain
                        'hana_bot_id': 8,  # Correct robot mapping
                        'bridge_topics': []
                    }
                },
                'domain_bridge': {'enable': False}
            }
    
    def start_domain_bridge(self):
        """Start domain bridge if enabled in config"""
        if not self.config.get('domain_bridge', {}).get('enable', False):
            self.get_logger().info("Domain bridge disabled in config")
            return
            
        try:
            # Use static config file
            bridge_config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'domain.yaml')
            
            if not os.path.exists(bridge_config_path):
                self.get_logger().error(f"Domain bridge config not found: {bridge_config_path}")
                return
            
            # Start domain_bridge
            cmd = ['ros2', 'run', 'domain_bridge', 'domain_bridge', bridge_config_path]
            self.domain_bridge_process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            
            self.get_logger().info(f"Started domain bridge with config: {bridge_config_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start domain bridge: {e}")
    
    def handle_fall_detection(self, msg):
        """Handle fall detection alerts from fall_detection_topic"""
        try:
            message_data = msg.data.strip()
            self.get_logger().info(f"Received fall detection message: {message_data}")
            
            if message_data == "fall_detected":
                self.fall_alert_active = True
                self.fall_alert_timestamp = time.time()
                
                self.get_logger().warn("🚨 FALL DETECTED! Alert activated for admin dashboard")
                
                # Update web frontend fall alert state directly
                self._update_web_fall_alert_state(True, self.fall_alert_timestamp, "Fall detected! Immediate attention required.")
                
                # Broadcast fall alert to web frontend
                alert_msg = String()
                alert_msg.data = json.dumps({
                    "type": "fall_alert",
                    "active": True,
                    "timestamp": self.fall_alert_timestamp,
                    "message": "Fall detected! Immediate attention required."
                })
                
                # Use existing robot status publisher to broadcast alert
                self.robot_status_pub.publish(alert_msg)
                
            elif message_data == "fall_cleared":
                # Allow clearing the alert if needed
                self.fall_alert_active = False
                self.get_logger().info("Fall alert cleared")
                
                # Update web frontend fall alert state directly
                self._update_web_fall_alert_state(False, time.time(), "Fall alert cleared.")
                
                # Broadcast cleared alert
                alert_msg = String()
                alert_msg.data = json.dumps({
                    "type": "fall_alert",
                    "active": False,
                    "timestamp": time.time(),
                    "message": "Fall alert cleared."
                })
                self.robot_status_pub.publish(alert_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error handling fall detection: {e}")
    
    def _update_web_fall_alert_state(self, active, timestamp, message):
        """Update web fall alert state via file system"""
        try:
            # Create fall alert status file for web to read
            fall_alert_data = {
                "active": active,
                "timestamp": timestamp,
                "message": message,
                "acknowledged": False
            }
            
            # Write to temporary file that web can read
            temp_file_path = "/tmp/fall_alert_status.json"
            with open(temp_file_path, 'w') as f:
                json.dump(fall_alert_data, f)
            
            self.get_logger().info(f"Updated fall alert status file: active={active}")
            
        except Exception as e:
            self.get_logger().error(f"Error updating fall alert status file: {e}")
    
    def handle_battery_status(self, msg, robot_id):
        """Handle battery status messages from /DP_09/battery_present and /DP_03/battery_present"""
        try:
            # Parse battery level from Float32 message
            battery_data = msg.data
            
            # Convert float to integer percentage (assume 0.0-1.0 range or 0-100 range)
            try:
                # If the value is between 0-1, assume it's a percentage ratio
                if 0.0 <= battery_data <= 1.0:
                    battery_level = int(battery_data * 100)
                # If the value is between 1-100, assume it's already a percentage
                elif 1.0 < battery_data <= 100.0:
                    battery_level = int(battery_data)
                else:
                    # Clamp to valid range
                    battery_level = max(0, min(100, int(battery_data)))
                    
                battery_level = max(0, min(100, battery_level))  # Final clamp between 0-100
            except (ValueError, TypeError):
                self.get_logger().warn(f"Invalid battery data from {robot_id}: {battery_data}")
                return
            
            # Update battery status
            self.battery_status[robot_id] = {
                "level": battery_level,
                "timestamp": time.time()
            }
            
            # Log battery status
            color_status = "🔴" if battery_level <= 40 else "🟡" if battery_level <= 60 else "🟢"
            self.get_logger().info(f"🔋 {robot_id} Battery: {battery_level}% {color_status}")
            
            # Write to file for web access
            self._update_battery_status_file()
            
        except Exception as e:
            self.get_logger().error(f"Error handling battery status for {robot_id}: {e}")
    
    def _update_battery_status_file(self):
        """Update battery status file for web to read"""
        try:
            battery_file_path = "/tmp/battery_status.json"
            with open(battery_file_path, 'w') as f:
                json.dump(self.battery_status, f)
            
        except Exception as e:
            self.get_logger().error(f"Error updating battery status file: {e}")
    
    def stop_domain_bridge(self):
        """Stop domain bridge process"""
        if self.domain_bridge_process:
            try:
                self.domain_bridge_process.terminate()
                self.domain_bridge_process.wait(timeout=5)
                self.get_logger().info("Domain bridge stopped")
            except subprocess.TimeoutExpired:
                self.domain_bridge_process.kill()
                self.get_logger().warn("Domain bridge forcefully terminated")
            except Exception as e:
                self.get_logger().error(f"Error stopping domain bridge: {e}")
    

def main():
    rclpy.init()
    fleet_manager = FleetManager()
    
    # Subscribe to confirm requests
    fleet_manager.confirm_sub = fleet_manager.create_subscription(
        String, '/fleet/confirm_request', fleet_manager.handle_confirm_request, 10)
    
    # Start domain bridge
    fleet_manager.start_domain_bridge()
    
    try:
        fleet_manager.get_logger().info(f"Fleet Manager running on domain {fleet_manager.config['central_domain']}")
        fleet_manager.get_logger().info(f"Managing {len(fleet_manager.robots)} robots: {list(fleet_manager.robots.keys())}")
        rclpy.spin(fleet_manager)
    except KeyboardInterrupt:
        pass
    finally:
        fleet_manager.stop_domain_bridge()
        fleet_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()