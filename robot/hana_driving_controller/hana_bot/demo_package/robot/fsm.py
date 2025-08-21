from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import sys
import os

# Database access removed - coordinates now provided by Fleet Manager

from .arm import arm_pick, arm_place_on_robot
from .nav2_waypoint_class import WaypointNavigator
try:
    import robot_config as cfg
except ImportError:
    # Fallback to main config for backward compatibility
    import config as cfg

from enum import IntEnum, auto

class Step(IntEnum):
    IDLE          = auto()
    GO_TO_ARM     = auto()
    PICK          = auto()
    GO_TO_USER    = auto()
    WAIT_CONFIRM  = auto()
    GO_DOCK       = auto()

# ───────── ROS FSM 정의 ─────────
class DeliveryFSM(Node):
    def __init__(self, robot_name: str, wp_nav: WaypointNavigator):
        super().__init__(f"{robot_name}_delivery_fsm", namespace=robot_name)
        self.step = Step.IDLE
        self.waypoint_nav = wp_nav
        self.create_subscription(String, cfg.ROS_CMD_TOPIC, self.on_cmd, 10)
        self.cmd_pub   = self.create_publisher(String, cfg.ROS_CMD_TOPIC, 10)
        self.stat_pub  = self.create_publisher(String, cfg.ROS_STAT_TOPIC, 10)
        self.create_timer(0.5, self.loop)
        
        # Default service station coordinates (will be updated per delivery)
        self.service_station_coords = cfg.SERVICE_ST1
        self.get_logger().info(f"Initialized with default service station coordinates: {self.service_station_coords}")
        
        # *** CRITICAL FIX: Always start with clean state ***
        self.current_task_id = None
        self.get_logger().info("🔧 ROBOT STARTUP: Cleared any stuck task_id, starting fresh")
        
        # Send initial status after a delay
        self.initial_status_timer = self.create_timer(1.0, self.send_initial_status)
        
        # Periodic status heartbeat (every 2 seconds) - DISABLED to fix oscillation
        # self.create_timer(2.0, self.send_heartbeat)
    
    def _parse_coordinates(self, coords):
        """Parse PostgreSQL array format {x,y,z} to Python list [x,y,z]"""
        if isinstance(coords, list):
            return coords
        if isinstance(coords, str):
            # Remove braces and split by comma
            clean_coords = coords.strip('{}')
            return [float(x.strip()) for x in clean_coords.split(',')]
        return coords

    # ---------- 토픽 콜백 / 퍼블리시 ----------
    def on_cmd(self, msg: String):
        self.get_logger().info(f"🔔 RECEIVED MESSAGE: {msg.data}")
        self.get_logger().info(f"🤖 CURRENT STATE: {self.step.name}")
        
        try:
            # Try to parse as JSON first (new format with resident_id and coordinates)
            cmd_data = json.loads(msg.data)
            command = cmd_data.get("command")
            resident_id = cmd_data.get("resident_id")
            target_coordinates = cmd_data.get("target_coordinates")
            
            self.get_logger().info(f"🔍 JSON PARSED - command: {command}, resident_id: {resident_id}")
            
            if command == "order":
                # Store task ID for status reporting
                self.current_task_id = cmd_data.get("task_id")
                
                if target_coordinates:
                    # Use coordinates provided by Fleet Manager
                    self.service_station_coords = target_coordinates
                    self.get_logger().info(f"Updated target coordinates for resident {resident_id}: {target_coordinates}")
                elif resident_id:
                    # Fallback: try to get coordinates from database (for backward compatibility)
                    new_coords = self._get_service_station_coords(resident_id)
                    if new_coords:
                        self.service_station_coords = new_coords
                        self.get_logger().info(f"Fallback: Updated target for resident {resident_id}: {new_coords}")
                
                # Determine task type (delivery vs call)
                task_type = cmd_data.get("task_type", "배달")  # Default to delivery
                
                if self.step == Step.IDLE:
                    if task_type == "호출":
                        # Call task: go directly to user (skip arm/pick)
                        self.set_step(Step.GO_TO_USER)
                    else:
                        # Delivery task: go to arm first
                        self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.WAIT_CONFIRM:
                    # New task assigned immediately after confirm
                    if task_type == "호출":
                        self.set_step(Step.GO_TO_USER)
                    else:
                        self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.GO_DOCK:
                    # Interrupt dock return and start new task immediately
                    self.get_logger().info(f"Interrupting dock return for new {task_type} task")
                    if task_type == "호출":
                        self.set_step(Step.GO_TO_USER)
                    else:
                        self.set_step(Step.GO_TO_ARM)
                        
            elif command == "confirm":
                # Handle JSON format confirm command
                self.get_logger().info(f"🔔 *** CONFIRM COMMAND DETECTED *** in state {self.step.name}")
                self.get_logger().info(f"🔍 Step check: self.step = {self.step}, Step.WAIT_CONFIRM = {Step.WAIT_CONFIRM}")
                self.get_logger().info(f"🔍 Equality check: self.step == Step.WAIT_CONFIRM = {self.step == Step.WAIT_CONFIRM}")
                
                if self.step == Step.WAIT_CONFIRM:
                    # Normal case: robot is waiting for confirmation
                    old_task_id = self.current_task_id
                    self.current_task_id = None
                    self.get_logger().info(f"✅ *** CONFIRM PROCESSED (NORMAL) *** - cleared task {old_task_id}, returning to dock")
                    self.set_step(Step.GO_DOCK)
                    self.get_logger().info(f"🏠 *** STATE CHANGED TO GO_DOCK ***")
                    
                elif self.step == Step.IDLE:
                    # Edge case: confirm received when idle (state sync issue)
                    # This can happen due to Fleet Manager/Robot state desync
                    old_task_id = self.current_task_id
                    self.current_task_id = None
                    self.get_logger().warn(f"⚠️ *** CONFIRM RECEIVED IN IDLE *** - clearing task {old_task_id}, staying idle")
                    self.get_logger().info(f"🔄 State sync recovered - robot stays IDLE")
                    # Stay in IDLE state to break the oscillation loop
                    
                else:
                    # Other states: log error but try to handle gracefully
                    self.get_logger().error(f"❌ *** CONFIRM IN UNEXPECTED STATE *** - current: {self.step.name}")
                    # Clear task ID anyway to reset state
                    old_task_id = self.current_task_id
                    self.current_task_id = None
                    self.get_logger().info(f"🔄 Emergency reset - cleared task {old_task_id}, going to IDLE")
                    self.set_step(Step.IDLE)
                    
        except json.JSONDecodeError:
            self.get_logger().info(f"🔄 JSON PARSE FAILED - trying string format")
            
            # Fallback to old string format
            if msg.data == "order":
                # Clear task ID for old format messages
                self.current_task_id = None
                if self.step == Step.IDLE:
                    self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.WAIT_CONFIRM:
                    # New task assigned immediately after confirm - go directly to arm
                    self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.GO_DOCK:
                    # Interrupt dock return and start new task immediately
                    self.get_logger().info("Interrupting dock return for new task (fallback)")
                    self.set_step(Step.GO_TO_ARM)
        
        # Handle confirm command (always string format) - OUTSIDE try-except block
        if msg.data == "confirm":
            self.get_logger().info(f"🔔 *** STRING CONFIRM DETECTED *** in state {self.step.name}")
            
            if self.step == Step.WAIT_CONFIRM:
                # Normal case: robot is waiting for confirmation
                old_task_id = self.current_task_id
                self.current_task_id = None
                self.get_logger().info(f"✅ *** STRING CONFIRM PROCESSED (NORMAL) *** - cleared task {old_task_id}, returning to dock")
                self.set_step(Step.GO_DOCK)
                self.get_logger().info(f"🏠 *** STATE CHANGED TO GO_DOCK (STRING) ***")
                
            elif self.step == Step.IDLE:
                # Edge case: confirm received when idle (state sync issue)
                old_task_id = self.current_task_id
                self.current_task_id = None
                self.get_logger().warn(f"⚠️ *** STRING CONFIRM IN IDLE *** - clearing task {old_task_id}, staying idle")
                self.get_logger().info(f"🔄 State sync recovered (STRING) - robot stays IDLE")
                # Stay in IDLE state to break the oscillation loop
                
            else:
                # Other states: log error but handle gracefully
                self.get_logger().error(f"❌ *** STRING CONFIRM IN UNEXPECTED STATE *** - current: {self.step.name}")
                old_task_id = self.current_task_id
                self.current_task_id = None
                self.get_logger().info(f"🔄 Emergency reset (STRING) - cleared task {old_task_id}, going to IDLE")
                self.set_step(Step.IDLE)
    
    def _get_service_station_coords(self, resident_id):
        """Fallback method - coordinates should be provided by Fleet Manager"""
        self.get_logger().warn(f"Fallback: No coordinates provided for resident {resident_id}. Using default.")
        return cfg.SERVICE_ST1  # Return default coordinates
    
    def _handle_navigation_failure(self):
        """Handle navigation failure - reset to IDLE and report failure"""
        self.get_logger().error(f"Navigation failed in step {self.step.name}. Resetting to IDLE.")
        
        # Reset navigation state
        self.waypoint_nav.hana_nav2_state = "None"
        
        # Report failure status to fleet manager
        self.pub_status("navigation_failed")
        
        # Reset to IDLE state
        self.set_step(Step.IDLE)

    def pub_status(self, text: str):
        # *** CRITICAL FIX: Force clear task_id when reporting idle ***
        task_id = getattr(self, 'current_task_id', None)
        
        if text == "idle":
            # Always clear task_id when going to idle
            if task_id is not None:
                self.get_logger().info(f"🔧 CLEARING task {task_id} because status is idle")
                self.current_task_id = None
                task_id = None
        elif text == "waiting_confirm" and task_id is None:
            # Don't report waiting_confirm without a task
            self.get_logger().error(f"🚨 CANNOT wait_confirm without task - forcing idle")
            text = "idle"
            
        # Create JSON status message for Fleet Manager compatibility
        status_data = {
            "status": text,
            "current_task_id": task_id,
            "timestamp": rclpy.clock.Clock().now().nanoseconds / 1e9
        }
        status_json = json.dumps(status_data)
        self.stat_pub.publish(String(data=status_json))
        self.get_logger().info(f"[STATUS] {text} (task: {task_id})")

    # ---------- 상태 전이 ----------
    def set_step(self, nxt: int):
        old_step = self.step
        self.step = nxt
        self.waypoint_nav.hana_nav2_state = "None"

        self.get_logger().info(f"🔄 STATE TRANSITION: {old_step.name if hasattr(old_step, 'name') else old_step} → {nxt.name}")

        match self.step:
            case Step.IDLE:
                # CRITICAL: Always clear task when going idle
                if getattr(self, 'current_task_id', None) is not None:
                    self.get_logger().info(f"🔧 IDLE transition: clearing task {self.current_task_id}")
                    self.current_task_id = None
                self.pub_status("idle")
            case Step.GO_TO_ARM:
                self.pub_status("moving_to_arm")
                self.waypoint_nav.send_goal(cfg.PICKUP_ST1)
            case Step.PICK:
                self.pub_status("picking")
                arm_pick("vitamin")
                arm_place_on_robot()
                self.set_step(Step.GO_TO_USER)
            case Step.GO_TO_USER:
                self.pub_status("moving_to_user")
                self.waypoint_nav.send_goal(self.service_station_coords)
            case Step.WAIT_CONFIRM:
                # Only allow WAIT_CONFIRM if we have a valid task
                if getattr(self, 'current_task_id', None) is not None:
                    self.pub_status("waiting_confirm")
                else:
                    self.get_logger().error(f"🚨 CANNOT WAIT_CONFIRM without task - forcing IDLE")
                    self.set_step(Step.IDLE)
            case Step.GO_DOCK:
                self.pub_status("returning_to_dock")
                self.waypoint_nav.send_goal(cfg.CHARGING_ST)

    def loop(self):
        # *** DEBUG: Log navigation state changes ***
        if hasattr(self, '_last_nav_state') and self._last_nav_state != self.waypoint_nav.hana_nav2_state:
            self.get_logger().debug(f"🗺️ Nav state changed: {self._last_nav_state} → {self.waypoint_nav.hana_nav2_state}")
        self._last_nav_state = self.waypoint_nav.hana_nav2_state
        
        if self.step == Step.GO_TO_ARM  and self.waypoint_nav.hana_nav2_state == "Done":
            self.set_step(Step.PICK)
        elif self.step == Step.GO_TO_USER and self.waypoint_nav.hana_nav2_state == "Done":
            self.set_step(Step.WAIT_CONFIRM)
        elif self.step == Step.GO_DOCK   and self.waypoint_nav.hana_nav2_state == "Done":
            self.set_step(Step.IDLE)
            
        # *** CRITICAL FIX: Prevent bogus transitions when IDLE ***
        elif self.step == Step.IDLE and self.waypoint_nav.hana_nav2_state == "Done":
            # Robot is IDLE but nav thinks it's "Done" - this is a stale state
            self.get_logger().warn(f"🚨 IDLE robot but nav=Done - clearing task_id and resetting nav state")
            self.current_task_id = None
            self.waypoint_nav.hana_nav2_state = "None"  # Reset navigation state
        
        # Handle navigation failures
        elif self.waypoint_nav.hana_nav2_state == "Failed":
            self._handle_navigation_failure()
    
    def send_initial_status(self):
        """Send initial status to fleet manager"""
        self.pub_status("idle")
        # Cancel the timer after first call
        self.initial_status_timer.cancel()
    
    def send_heartbeat(self):
        """Send periodic status update to keep connection alive"""
        # Send current status based on step
        status_map = {
            Step.IDLE: "idle",
            Step.GO_TO_ARM: "moving_to_arm", 
            Step.PICK: "picking",
            Step.GO_TO_USER: "moving_to_user",
            Step.WAIT_CONFIRM: "waiting_confirm",
            Step.GO_DOCK: "returning_to_dock"
        }
        
        current_status = status_map.get(self.step, "idle")
        current_task = getattr(self, 'current_task_id', None)
        
        # *** DEBUG: Log detailed state info ***
        self.get_logger().info(f"💓 HEARTBEAT: step={self.step.name}, status={current_status}, task={current_task}, nav={self.waypoint_nav.hana_nav2_state}")
        
        # *** CRITICAL: AGGRESSIVE task_id cleanup ***
        if self.step == Step.IDLE:
            # Always clear task_id when in IDLE, regardless of previous state
            if current_task is not None:
                self.current_task_id = None
                self.get_logger().info(f"🔧 CLEARED STUCK TASK {current_task} in IDLE heartbeat")
                current_task = None
        
        # *** EMERGENCY FIX: If stuck in WAIT_CONFIRM without valid task ***
        if self.step == Step.WAIT_CONFIRM and current_task is None:
            self.get_logger().error(f"🚨 STUCK in WAIT_CONFIRM without task - forcing to IDLE")
            self.set_step(Step.IDLE)
            current_status = "idle"
        
        # Create JSON status message for Fleet Manager compatibility
        status_data = {
            "status": current_status,
            "current_task_id": current_task,
            "timestamp": rclpy.clock.Clock().now().nanoseconds / 1e9
        }
        status_json = json.dumps(status_data)
        self.stat_pub.publish(String(data=status_json))
