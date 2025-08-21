# Fleet monitoring dashboard
from flask import Blueprint, render_template, jsonify, request
from ..task_db import get_pending_tasks, get_robot_current_task
from ..routes.status import get_robot_status_cache
from ..fleet_client import get_fleet_client
import psycopg2
from psycopg2.extras import DictCursor
import config as cfg

bp = Blueprint('fleet_monitor', __name__)

# Global fall detection alert state
_fall_alert_state = {
    "active": False,
    "timestamp": None,
    "message": "",
    "acknowledged": False
}

def update_fall_alert_state(active, timestamp=None, message="", acknowledged=False):
    """Update fall detection alert state"""
    global _fall_alert_state
    _fall_alert_state = {
        "active": active,
        "timestamp": timestamp,
        "message": message,
        "acknowledged": acknowledged
    }

def get_fall_alert_state():
    """Get current fall detection alert state"""
    global _fall_alert_state
    return _fall_alert_state.copy()

def get_db_connection():
    """Get database connection"""
    return psycopg2.connect(**cfg.DB_DSN)

@bp.route('/test')
def fleet_dashboard():
    """Fleet monitoring dashboard page"""
    return render_template('fleet_dashboard.html')

@bp.route('/test2')
def advanced_fleet_dashboard():
    """Advanced fleet analytics dashboard"""
    return render_template('advanced_fleet_dashboard.html')

@bp.route('/test3')
def admin_dashboard():
    """HANA Admin dashboard page"""
    return render_template('admin_dashboard.html')

@bp.route('/test4')
def admin_dashboard_white():
    """HANA Admin dashboard page - White Theme"""
    return render_template('admin_dashboard_white.html')

@bp.route('/api/fleet/status')
def fleet_status_api():
    """API endpoint for fleet status data"""
    try:
        # Get robot status
        robots = []
        robot_ids = ['robot_1', 'robot_2']  # Multi-robot support
        
        for robot_id in robot_ids:
            # Get robot status from cache
            status_data = get_robot_status_cache(robot_id)
            
            # Get current task for robot
            robot_db_id = 8 if robot_id == 'robot_1' else (9 if robot_id == 'robot_2' else None)  # Correct robot mapping
            current_task = None
            if robot_db_id:
                current_task = get_robot_current_task(robot_db_id)

            # Get real-time battery level
            real_time_battery = get_battery_level_for_robot(robot_db_id) if robot_db_id else None
            
            # Get DB battery as fallback
            db_battery = None
            if robot_db_id:
                try:
                    with get_db_connection() as conn:
                        with conn.cursor() as cur:
                            cur.execute("SELECT battery FROM hana_bots WHERE hana_bot_id = %s", (robot_db_id,))
                            result = cur.fetchone()
                            db_battery = result[0] if result else None
                except Exception as e:
                    print(f"Error getting DB battery for robot {robot_db_id}: {e}")
            
            battery_level = real_time_battery if real_time_battery is not None else db_battery
            
            robots.append({
                'id': robot_id,
                'name': 'HANA PINKY' if robot_id == 'robot_1' else ('HANA ROBOT 2' if robot_id == 'robot_2' else robot_id),
                'status': status_data.get('status', 'idle') if status_data else 'offline',
                'battery': battery_level,
                'battery_real_time': real_time_battery is not None,
                'current_task': current_task,
                'last_update': status_data.get('last_update') if status_data else None,
                'is_waiting_confirm': status_data.get('is_waiting_confirm', False) if status_data else False
            })
        
        # Get task queue
        pending_tasks = get_pending_tasks()
        
        # Get recent completed tasks
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT t.task_id, t.task_type, t.status, t.requester_resident_id, 
                           t.item_id, t.assigned_bot_id, t.created_at, t.completed_at,
                           r.name as requester_name, i.item_type as item_type
                    FROM tasks t
                    LEFT JOIN residents r ON t.requester_resident_id = r.resident_id
                    LEFT JOIN items i ON t.item_id = i.item_id
                    WHERE t.status IN ('완료', '실패')
                    ORDER BY t.completed_at DESC, t.created_at DESC
                    LIMIT 10
                """)
                completed_tasks = [dict(row) for row in cur.fetchall()]
        
        # Get active tasks
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT t.task_id, t.task_type, t.status, t.requester_resident_id, 
                           t.item_id, t.assigned_bot_id, t.created_at,
                           r.name as requester_name, i.item_type as item_type,
                           b.bot_name
                    FROM tasks t
                    LEFT JOIN residents r ON t.requester_resident_id = r.resident_id
                    LEFT JOIN items i ON t.item_id = i.item_id
                    LEFT JOIN hana_bots b ON t.assigned_bot_id = b.hana_bot_id
                    WHERE t.status IN ('할당', '이동중', '집기중', '수령대기')
                    ORDER BY t.created_at ASC
                """)
                active_tasks = [dict(row) for row in cur.fetchall()]
        
        # Calculate fleet statistics
        total_tasks_today = 0
        completed_tasks_today = 0
        
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT COUNT(*) FROM tasks 
                    WHERE DATE(created_at) = CURRENT_DATE
                """)
                total_tasks_today = cur.fetchone()[0]
                
                cur.execute("""
                    SELECT COUNT(*) FROM tasks 
                    WHERE DATE(created_at) = CURRENT_DATE AND status = '완료'
                """)
                completed_tasks_today = cur.fetchone()[0]
        
        return jsonify({
            'success': True,
            'data': {
                'robots': robots,
                'task_queue': {
                    'pending': pending_tasks,
                    'active': active_tasks,
                    'completed': completed_tasks
                },
                'statistics': {
                    'total_tasks_today': total_tasks_today,
                    'completed_tasks_today': completed_tasks_today,
                    'success_rate': round(completed_tasks_today / total_tasks_today * 100, 1) if total_tasks_today > 0 else 0,
                    'pending_count': len(pending_tasks),
                    'active_count': len(active_tasks)
                }
            }
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fleet/analytics')
def fleet_analytics_api():
    """API endpoint for advanced fleet analytics"""
    try:
        # Get period parameter from query string
        period = request.args.get('period', '24h')
        
        # Determine time interval and grouping based on period
        if period == '24h':
            time_interval = '24 hours'
            group_by_clause = 'EXTRACT(HOUR FROM created_at)'
            select_clause = 'EXTRACT(HOUR FROM created_at) as hour'
            order_by_clause = 'hour'
        elif period == '7d':
            time_interval = '7 days'
            group_by_clause = 'DATE(created_at)'
            select_clause = 'DATE(created_at) as date'
            order_by_clause = 'date'
        elif period == '30d':
            time_interval = '30 days'
            group_by_clause = 'DATE(created_at)'
            select_clause = 'DATE(created_at) as date'
            order_by_clause = 'date'
        else:
            time_interval = '24 hours'
            group_by_clause = 'EXTRACT(HOUR FROM created_at)'
            select_clause = 'EXTRACT(HOUR FROM created_at) as hour'
            order_by_clause = 'hour'
        
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                # Time-based task distribution
                if period == '24h':
                    cur.execute(f"""
                        SELECT 
                            {select_clause},
                            COUNT(*) as task_count,
                            SUM(CASE WHEN status = '완료' THEN 1 ELSE 0 END) as completed_count,
                            SUM(CASE WHEN status = '실패' THEN 1 ELSE 0 END) as failed_count
                        FROM tasks 
                        WHERE created_at >= NOW() - INTERVAL '{time_interval}'
                        GROUP BY {group_by_clause}
                        ORDER BY {order_by_clause}
                    """)
                else:
                    # For 7d and 30d, use field names that match daily_trends structure
                    cur.execute(f"""
                        SELECT 
                            {select_clause},
                            COUNT(*) as total_tasks,
                            SUM(CASE WHEN status = '완료' THEN 1 ELSE 0 END) as completed_tasks,
                            SUM(CASE WHEN status = '실패' THEN 1 ELSE 0 END) as failed_tasks
                        FROM tasks 
                        WHERE created_at >= NOW() - INTERVAL '{time_interval}'
                        GROUP BY {group_by_clause}
                        ORDER BY {order_by_clause} DESC
                    """)
                time_stats = [dict(row) for row in cur.fetchall()]
                print(f"📊 Analytics API: period={period}, returned {len(time_stats)} records")

                # Robot performance metrics
                cur.execute("""
                    SELECT 
                        b.bot_name,
                        b.hana_bot_id,
                        COUNT(t.task_id) as total_tasks,
                        SUM(CASE WHEN t.status = '완료' THEN 1 ELSE 0 END) as completed_tasks,
                        SUM(CASE WHEN t.status = '실패' THEN 1 ELSE 0 END) as failed_tasks,
                        ROUND(AVG(EXTRACT(EPOCH FROM (t.completed_at - t.created_at))/60), 2) as avg_completion_minutes
                    FROM hana_bots b
                    LEFT JOIN tasks t ON b.hana_bot_id = t.assigned_bot_id 
                        AND t.created_at >= NOW() - INTERVAL '7 days'
                    GROUP BY b.bot_name, b.hana_bot_id
                    ORDER BY total_tasks DESC
                """)
                robot_performance = [dict(row) for row in cur.fetchall()]

                # Top requesters (last 7 days)
                cur.execute("""
                    SELECT 
                        r.name,
                        r.resident_id,
                        COUNT(t.task_id) as order_count,
                        SUM(CASE WHEN t.status = '완료' THEN 1 ELSE 0 END) as completed_orders
                    FROM residents r
                    LEFT JOIN tasks t ON r.resident_id = t.requester_resident_id 
                        AND t.created_at >= NOW() - INTERVAL '7 days'
                    WHERE t.task_id IS NOT NULL
                    GROUP BY r.name, r.resident_id
                    ORDER BY order_count DESC
                    LIMIT 10
                """)
                top_requesters = [dict(row) for row in cur.fetchall()]

                # Popular items (last 7 days)
                cur.execute("""
                    SELECT 
                        i.item_type,
                        i.item_id,
                        COUNT(t.task_id) as order_count,
                        SUM(CASE WHEN t.status = '완료' THEN 1 ELSE 0 END) as delivered_count
                    FROM items i
                    LEFT JOIN tasks t ON i.item_id = t.item_id 
                        AND t.created_at >= NOW() - INTERVAL '7 days'
                    WHERE t.task_id IS NOT NULL
                    GROUP BY i.item_type, i.item_id
                    ORDER BY order_count DESC
                """)
                popular_items = [dict(row) for row in cur.fetchall()]

                # Daily trend (last 30 days)
                cur.execute("""
                    SELECT 
                        DATE(created_at) as date,
                        COUNT(*) as total_tasks,
                        SUM(CASE WHEN status = '완료' THEN 1 ELSE 0 END) as completed_tasks,
                        SUM(CASE WHEN status = '실패' THEN 1 ELSE 0 END) as failed_tasks,
                        ROUND(AVG(EXTRACT(EPOCH FROM (completed_at - created_at))/60), 2) as avg_completion_minutes
                    FROM tasks 
                    WHERE created_at >= NOW() - INTERVAL '30 days'
                    GROUP BY DATE(created_at)
                    ORDER BY date DESC
                    LIMIT 30
                """)
                daily_trends = [dict(row) for row in cur.fetchall()]

                # System performance metrics
                cur.execute("""
                    SELECT 
                        COUNT(*) as total_tasks_today,
                        SUM(CASE WHEN status = '완료' THEN 1 ELSE 0 END) as completed_today,
                        SUM(CASE WHEN status = '실패' THEN 1 ELSE 0 END) as failed_today,
                        SUM(CASE WHEN status IN ('대기', '할당', '이동중', '집기중', '수령대기') THEN 1 ELSE 0 END) as active_today,
                        ROUND(AVG(CASE WHEN status = '완료' AND completed_at IS NOT NULL 
                                      THEN EXTRACT(EPOCH FROM (completed_at - created_at))/60 END), 2) as avg_completion_time,
                        COUNT(DISTINCT requester_resident_id) as unique_users_today,
                        COUNT(DISTINCT assigned_bot_id) as active_robots_today
                    FROM tasks 
                    WHERE DATE(created_at) = CURRENT_DATE
                """)
                system_metrics = dict(cur.fetchone())

        # Structure response based on period
        response_data = {
            'robot_performance': robot_performance,
            'top_requesters': top_requesters,
            'popular_items': popular_items,
            'daily_trends': daily_trends,
            'system_metrics': system_metrics
        }
        
        # Add time-based data with appropriate key
        if period == '24h':
            response_data['hourly_distribution'] = time_stats
        else:
            # For 7d and 30d, put the data in daily_trends for frontend processing
            response_data['daily_trends'] = time_stats
        
        return jsonify({
            'success': True,
            'data': response_data
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fleet/control', methods=['POST'])
def fleet_control_api():
    """API endpoint for fleet control actions"""
    try:
        data = request.get_json()
        action = data.get('action')
        target_id = data.get('target_id')
        
        if action == 'cancel_task':
            # Cancel specific task
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        UPDATE tasks 
                        SET status = '실패', completed_at = CURRENT_TIMESTAMP
                        WHERE task_id = %s AND status NOT IN ('완료', '실패')
                    """, (target_id,))
                    conn.commit()
            
            return jsonify({
                'success': True,
                'message': f'Task #{target_id} cancelled successfully'
            })
            
        elif action == 'reset_robot':
            # Reset robot to idle (this would need ROS integration for real implementation)
            return jsonify({
                'success': True,
                'message': f'Robot {target_id} reset command sent'
            })
            
        elif action == 'fail_all_incomplete':
            # Fail all incomplete tasks
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        UPDATE tasks 
                        SET status = '실패', completed_at = CURRENT_TIMESTAMP
                        WHERE status NOT IN ('완료', '실패')
                    """)
                    affected_rows = cur.rowcount
                    conn.commit()
            
            return jsonify({
                'success': True,
                'message': f'{affected_rows} incomplete tasks marked as failed'
            })
            
        else:
            return jsonify({
                'success': False,
                'error': 'Unknown action'
            })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

# GPT가 수정함 0730

def get_real_time_battery_data():
    """Get real-time battery data from Fleet Manager"""
    import os
    import json
    
    battery_file_path = "/tmp/battery_status.json"
    battery_data = {}
    
    if os.path.exists(battery_file_path):
        try:
            with open(battery_file_path, 'r') as f:
                battery_data = json.load(f)
        except Exception as e:
            print(f"Error reading battery status file: {e}")
    
    return battery_data

def get_battery_level_for_robot(robot_id):
    """Get real-time battery level for specific robot"""
    battery_data = get_real_time_battery_data()
    
    # Map robot database ID to battery topic ID
    battery_id_map = {
        8: 'DP_09',  # robot_1 (HANA PINKY)
        9: 'DP_03'   # robot_2 (HANA ROBOT 9)
    }
    
    battery_id = battery_id_map.get(robot_id)
    if battery_id and battery_id in battery_data:
        battery_info = battery_data[battery_id]
        level = battery_info.get('level', 0)
        timestamp = battery_info.get('timestamp', 0)
        
        # Check if data is recent (within last 30 seconds)
        import time
        data_age = time.time() - timestamp
        
        return level if data_age < 30 else None
    
    return None

@bp.route('/api/admin/robots')
def admin_robots_api():
    """API endpoint for all robot data"""
    try:
        robots = []
        # Multi-robot support: robot_1 (ID:8) and robot_2 (ID:9)
        # Query the database for all registered robots and their real-time status
        # from the fleet manager. Combine DB info with cached ROS status.
        
        # Get all robots from DB
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT hana_bot_id, bot_name, battery, status
                    FROM hana_bots
                    ORDER BY hana_bot_id ASC
                """)
                db_robots = {r['hana_bot_id']: dict(r) for r in cur.fetchall()}

        # Get real-time status from cache (from fleet_client)
        from ..routes.status import get_robot_status_cache
        ros_status_cache = get_robot_status_cache()

        for bot_id, db_robot in db_robots.items():
            robot_id_str = f"robot_1" if bot_id == 8 else (f"robot_2" if bot_id == 9 else str(bot_id)) # Correct robot mapping
            ros_status = ros_status_cache.get(robot_id_str, {})
            
            current_task = None
            if bot_id:
                current_task = get_robot_current_task(bot_id)

            # Get real-time battery level
            real_time_battery = get_battery_level_for_robot(bot_id)
            battery_level = real_time_battery if real_time_battery is not None else db_robot['battery']

            robots.append({
                'id': bot_id,
                'name': db_robot['bot_name'],
                'battery': battery_level,
                'battery_real_time': real_time_battery is not None,
                'status': ros_status.get('status', db_robot['status'] or 'offline'), # Prefer ROS status
                'current_task': current_task,
                'last_update': ros_status.get('last_update')
            })
        
        return jsonify({
            'success': True,
            'data': robots
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/admin/tasks')
def admin_tasks_api():
    """API endpoint for all task data"""
    try:
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT 
                        t.task_id, t.task_type, t.status, t.created_at, t.completed_at,
                        r.name as requester_name,
                        i.item_type as item_type,
                        b.bot_name
                    FROM tasks t
                    LEFT JOIN residents r ON t.requester_resident_id = r.resident_id
                    LEFT JOIN items i ON t.item_id = i.item_id
                    LEFT JOIN hana_bots b ON t.assigned_bot_id = b.hana_bot_id
                    ORDER BY t.created_at DESC
                """)
                tasks = [dict(row) for row in cur.fetchall()]
        return jsonify({
            'success': True,
            'data': tasks
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/admin/users')
def admin_users_api():
    """API endpoint for all user data"""
    try:
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT resident_id, name, gender, birth_date, login_id
                    FROM residents
                    ORDER BY resident_id ASC
                """)
                users = [dict(row) for row in cur.fetchall()]
        return jsonify({
            'success': True,
            'data': users
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/admin/items')
def admin_items_api():
    """API endpoint for all item data"""
    try:
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT item_id, item_type, item_quantity
                    FROM items
                    ORDER BY item_id ASC
                """)
                items = [dict(row) for row in cur.fetchall()]
        return jsonify({
            'success': True,
            'data': items
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fall_detection/status')
def fall_detection_status():
    """Get current fall detection alert status"""
    try:
        # First check file system for Fleet Manager updates
        import os
        import json
        temp_file_path = "/tmp/fall_alert_status.json"
        
        if os.path.exists(temp_file_path):
            try:
                with open(temp_file_path, 'r') as f:
                    file_data = json.load(f)
                
                # Update global state from file
                update_fall_alert_state(
                    file_data.get('active', False),
                    file_data.get('timestamp'),
                    file_data.get('message', ''),
                    file_data.get('acknowledged', False)
                )
                
                # Return file data
                return jsonify({
                    'success': True,
                    'data': file_data
                })
            except Exception as file_error:
                print(f"Error reading fall alert file: {file_error}")
        
        # Fallback to global state
        return jsonify({
            'success': True,
            'data': get_fall_alert_state()
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fall_detection/acknowledge', methods=['POST'])
def acknowledge_fall_alert():
    """Acknowledge fall detection alert"""
    try:
        import os
        import json
        temp_file_path = "/tmp/fall_alert_status.json"
        
        # Update file if exists
        if os.path.exists(temp_file_path):
            try:
                with open(temp_file_path, 'r') as f:
                    file_data = json.load(f)
                
                file_data['acknowledged'] = True
                
                with open(temp_file_path, 'w') as f:
                    json.dump(file_data, f)
                
            except Exception as file_error:
                print(f"Error updating fall alert file: {file_error}")
        
        # Update global state
        global _fall_alert_state
        _fall_alert_state['acknowledged'] = True
        
        return jsonify({
            'success': True,
            'message': 'Fall alert acknowledged',
            'data': get_fall_alert_state()
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fall_detection/clear', methods=['POST'])
def clear_fall_alert():
    """Clear fall detection alert"""
    try:
        import os
        temp_file_path = "/tmp/fall_alert_status.json"
        
        # Remove file
        if os.path.exists(temp_file_path):
            try:
                os.remove(temp_file_path)
                print(f"Removed fall alert status file")
            except Exception as file_error:
                print(f"Error removing fall alert file: {file_error}")
        
        # Update global state
        update_fall_alert_state(False, None, "", False)
        
        return jsonify({
            'success': True,
            'message': 'Fall alert cleared',
            'data': get_fall_alert_state()
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fall_detection/test', methods=['POST'])
def test_fall_alert():
    """Test fall detection alert (for development/testing)"""
    try:
        import time
        update_fall_alert_state(
            active=True, 
            timestamp=time.time(), 
            message="Test fall detection alert - Please check immediately!", 
            acknowledged=False
        )
        
        return jsonify({
            'success': True,
            'message': 'Test fall alert activated',
            'data': get_fall_alert_state()
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/battery/status')
def battery_status():
    """Get current robot battery status"""
    try:
        import os
        import json
        battery_file_path = "/tmp/battery_status.json"
        
        if os.path.exists(battery_file_path):
            try:
                with open(battery_file_path, 'r') as f:
                    battery_data = json.load(f)
                    
                return jsonify({
                    'success': True,
                    'data': battery_data
                })
            except Exception as file_error:
                print(f"Error reading battery status file: {file_error}")
        
        # Fallback: no data available
        return jsonify({
            'success': True,
            'data': {
                "DP_09": {"level": 0, "timestamp": 0},
                "DP_03": {"level": 0, "timestamp": 0}
            }
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/battery/test/<robot_id>/<int:level>', methods=['POST'])
def test_battery_level(robot_id, level):
    """Test battery level (for development/testing)"""
    try:
        import os
        import json
        import time
        
        battery_file_path = "/tmp/battery_status.json"
        
        # Load existing data or create new
        battery_data = {}
        if os.path.exists(battery_file_path):
            try:
                with open(battery_file_path, 'r') as f:
                    battery_data = json.load(f)
            except:
                pass
        
        # Update specific robot
        if robot_id in ["DP_09", "DP_03"]:
            battery_data[robot_id] = {
                "level": max(0, min(100, level)),
                "timestamp": time.time()
            }
            
            with open(battery_file_path, 'w') as f:
                json.dump(battery_data, f)
            
            return jsonify({
                'success': True,
                'message': f'Test battery level set: {robot_id} = {level}%',
                'data': battery_data
            })
        else:
            return jsonify({
                'success': False,
                'error': f'Invalid robot_id: {robot_id}. Use DP_09 or DP_03'
            })
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })