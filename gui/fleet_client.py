# Fleet communication client for web server
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import uuid
import threading
import time
from typing import Dict, Callable

class FleetClient(Node):
    def __init__(self):
        super().__init__('web_fleet_client')
        
        # Set domain ID for central server (same as Fleet Manager)
        import os
        os.environ['ROS_DOMAIN_ID'] = '129'
        
        # Publishers to fleet manager
        self.task_request_pub = self.create_publisher(String, '/fleet/task_request', 10)
        self.confirm_request_pub = self.create_publisher(String, '/fleet/confirm_request', 10)
        
        # Subscriber for task responses
        self.task_response_sub = self.create_subscription(
            String, '/fleet/task_response', self.handle_task_response, 10)
        
        # Subscriber for robot status updates
        self.robot_status_sub = self.create_subscription(
            String, '/fleet/robot_status', self.handle_robot_status, 10)
        
        # Track pending requests
        self.pending_requests: Dict[str, Callable] = {}
        self.response_timeout = 5.0  # 5 seconds
        
        self.get_logger().info("Fleet Client initialized")
    
    def handle_task_response(self, msg: String):
        """Handle responses from fleet manager"""
        try:
            data = json.loads(msg.data)
            task_id = data['task_id']
            
            if task_id in self.pending_requests:
                callback = self.pending_requests.pop(task_id)
                callback(data)
            else:
                self.get_logger().warn(f"Received response for unknown task: {task_id}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling task response: {e}")
    
    def handle_robot_status(self, msg: String):
        """Handle robot status updates from fleet manager"""
        try:
            data = json.loads(msg.data)
            robot_id = data['robot_id']
            status = data['status']
            task_id = data.get('current_task_id')
            
            # Update status cache
            from .routes.status import update_robot_status_cache
            update_robot_status_cache(robot_id, status, task_id)
            
            self.get_logger().info(f"Updated status cache: {robot_id} -> {status}")
            
        except Exception as e:
            self.get_logger().error(f"Error handling robot status: {e}")
    
    def send_delivery_task(self, resident_id: str, items: list, callback: Callable, db_task_id: int = None):
        """Send delivery task to fleet manager"""
        task_id = str(db_task_id) if db_task_id else str(uuid.uuid4())
        
        task_data = {
            'task_id': task_id,
            'db_task_id': db_task_id,
            'task_type': 'delivery',
            'resident_id': resident_id,
            'items': items,
            'priority': 1
        }
        
        # Store callback for response
        self.pending_requests[task_id] = callback
        
        # Send request
        msg = String(data=json.dumps(task_data))
        self.task_request_pub.publish(msg)
        
        self.get_logger().info(f"Sent delivery task: {task_id} (DB ID: {db_task_id})")
        
        # Set timeout
        threading.Timer(self.response_timeout, self._timeout_callback, args=[task_id]).start()
        
        return task_id
    
    def send_call_task(self, resident_id: str, callback: Callable, db_task_id: int = None):
        """Send call task to fleet manager"""
        task_id = str(db_task_id) if db_task_id else str(uuid.uuid4())
        
        task_data = {
            'task_id': task_id,
            'db_task_id': db_task_id,
            'task_type': 'call',
            'resident_id': resident_id,
            'priority': 1
        }
        
        # Store callback for response
        self.pending_requests[task_id] = callback
        
        # Send request
        msg = String(data=json.dumps(task_data))
        self.task_request_pub.publish(msg)
        
        self.get_logger().info(f"Sent call task: {task_id} (DB ID: {db_task_id})")
        
        # Set timeout
        threading.Timer(self.response_timeout, self._timeout_callback, args=[task_id]).start()
        
        return task_id
    
    def send_confirm_request(self, robot_id: str = None):
        """Send confirmation request to fleet manager"""
        confirm_data = {}
        if robot_id:
            confirm_data['robot_id'] = robot_id
        
        msg = String(data=json.dumps(confirm_data))
        self.confirm_request_pub.publish(msg)
        
        self.get_logger().info(f"Sent confirm request for {robot_id}")
    
    def _timeout_callback(self, task_id: str):
        """Handle request timeout"""
        if task_id in self.pending_requests:
            callback = self.pending_requests.pop(task_id)
            # Call callback with timeout error
            callback({
                'task_id': task_id,
                'status': 'timeout',
                'message': 'Request timed out'
            })

# Global fleet client instance
_fleet_client = None
_fleet_client_lock = threading.Lock()

def get_fleet_client():
    """Get singleton fleet client instance"""
    global _fleet_client
    
    with _fleet_client_lock:
        if _fleet_client is None:
            if not rclpy.ok():
                rclpy.init()
            _fleet_client = FleetClient()
            
            # Start spinning in background thread
            executor_thread = threading.Thread(target=_spin_fleet_client, daemon=True)
            executor_thread.start()
    
    return _fleet_client

def _spin_fleet_client():
    """Background thread to spin the fleet client"""
    global _fleet_client
    try:
        rclpy.spin(_fleet_client)
    except Exception as e:
        print(f"Fleet client spin error: {e}")