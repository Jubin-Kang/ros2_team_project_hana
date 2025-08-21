from flask import Blueprint, jsonify, request, session
from ..fleet_client import get_fleet_client
from ..db import query_db
from ..task_db import get_active_tasks_by_bot
import json
import threading
import time

bp = Blueprint("status", __name__)

# Global variable to store latest robot status (support multi-robot)
_robot_status_cache = {
    "robot_1": {
        "status": "idle",
        "current_task_id": None,
        "last_update": time.time()
    },
    "robot_2": {
        "status": "idle", 
        "current_task_id": None,
        "last_update": time.time()
    }
}

def update_robot_status_cache(robot_id: str, status: str, task_id: str = None):
    """Update robot status cache"""
    global _robot_status_cache
    _robot_status_cache[robot_id] = {
        "status": status,
        "current_task_id": task_id,
        "last_update": time.time()
    }

def get_robot_status_cache(robot_id: str = None):
    """Get robot status from cache"""
    global _robot_status_cache
    if robot_id:
        return _robot_status_cache.get(robot_id)
    return _robot_status_cache

def initialize_robot_status_from_db():
    """Initialize robot status cache from database on server startup"""
    global _robot_status_cache
    
    try:
        # Bot ID to robot name mapping
        bot_id_to_robot = {8: "robot_1", 9: "robot_2", 13: "robot_3"}
        
        # Get active tasks from database
        active_tasks = get_active_tasks_by_bot()
        
        print(f"🔄 Initializing robot status cache from DB...")
        print(f"📋 Found active tasks: {active_tasks}")
        
        # First, reset all robots to idle
        for bot_id, robot_id in bot_id_to_robot.items():
            _robot_status_cache[robot_id] = {
                "status": "idle",
                "current_task_id": None,
                "last_update": time.time()
            }
        
        for bot_id, task in active_tasks.items():
            if bot_id in bot_id_to_robot:
                robot_id = bot_id_to_robot[bot_id]
                
                # Map database status to robot status
                status_map = {
                    '할당': 'assigned',
                    '이동중': 'moving_to_user',  # 기본값은 사용자로 이동 중
                    '집기중': 'picking',
                    '수령대기': 'waiting_confirm'
                }
                
                # Determine robot status based on task
                db_status = task['status']
                robot_status = status_map.get(db_status, 'idle')
                
                # For delivery tasks in '이동중' state, check elapsed time to determine if arrived
                if task['task_type'] == '배달' and db_status == '이동중':
                    # Calculate elapsed time since task creation
                    from datetime import datetime, timezone
                    created_at = task['created_at']
                    if hasattr(created_at, 'replace'):
                        if created_at.tzinfo is None:
                            created_at = created_at.replace(tzinfo=timezone.utc)
                    
                    elapsed_minutes = (datetime.now(timezone.utc) - created_at).total_seconds() / 60
                    
                    # If task has been running for more than 3 minutes, assume robot arrived
                    # This is a heuristic to handle server restart scenarios
                    if elapsed_minutes > 3:
                        robot_status = 'waiting_confirm'
                        print(f"🕐 Task {task['task_id']} elapsed {elapsed_minutes:.1f} min - assuming arrived")
                    else:
                        robot_status = 'moving_to_arm'  # Still picking up items
                elif task['task_type'] == '호출' and db_status == '이동중':
                    robot_status = 'moving_to_user'
                
                # Update cache
                _robot_status_cache[robot_id] = {
                    "status": robot_status,
                    "current_task_id": str(task['task_id']),
                    "last_update": time.time()
                }
                
                print(f"🤖 {robot_id}: {robot_status} (task_id: {task['task_id']}, db_status: {db_status})")
        
        print(f"✅ Robot status cache initialized successfully")
        print(f"📊 Final cache state: {_robot_status_cache}")
        
    except Exception as e:
        print(f"❌ Error initializing robot status cache: {e}")
        import traceback
        traceback.print_exc()
        # Keep default values if initialization fails

@bp.route("/robot_status", methods=["GET"])
def get_robot_status():
    """Get current robot status"""
    try:
        return jsonify({
            "success": True,
            "robots": _robot_status_cache
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/force_cache_refresh", methods=["POST"])
def force_cache_refresh():
    """Force refresh robot status cache from database"""
    try:
        initialize_robot_status_from_db()
        return jsonify({
            "success": True,
            "message": "Cache refreshed successfully",
            "cache": _robot_status_cache
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/robot_status/<robot_id>", methods=["GET"])
def get_specific_robot_status(robot_id: str):
    """Get status of specific robot"""
    try:
        if robot_id in _robot_status_cache:
            robot_info = _robot_status_cache[robot_id]
            current_task_id = robot_info["current_task_id"]
            
            # Check if current task belongs to logged-in user
            is_my_order = False
            resident_id = request.args.get('resident_id') or session.get('resident_id')
            
            print(f"DEBUG: current_task_id={current_task_id}, resident_id={resident_id}")
            if current_task_id and resident_id:
                try:
                    with query_db() as cur:
                        cur.execute("""
                            SELECT requester_resident_id 
                            FROM tasks 
                            WHERE task_id = %s
                        """, (int(current_task_id),))
                        result = cur.fetchone()
                        
                        print(f"DEBUG: task owner={result[0] if result else None}, checking resident_id={resident_id}")
                        if result and result[0] == int(resident_id):
                            is_my_order = True
                except Exception as e:
                    print(f"Error checking task ownership: {e}")
            else:
                print(f"DEBUG: No resident_id found")
            
            return jsonify({
                "success": True,
                "robot_id": robot_id,
                "status": robot_info["status"],
                "current_task_id": current_task_id,
                "is_waiting_confirm": robot_info["status"] == "waiting_confirm" and is_my_order,
                "is_my_order": is_my_order,
                "last_update": robot_info["last_update"]
            })
        else:
            return jsonify({
                "success": False,
                "error": "Robot not found"
            }), 404
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500