# Database operations for task management
import psycopg2
from psycopg2.extras import DictCursor
import config as cfg
from datetime import datetime
from typing import List, Dict, Optional

def get_db_connection():
    """Get database connection"""
    return psycopg2.connect(**cfg.DB_DSN)

def create_task(task_type: str, requester_resident_id: int, item_id: int = None, 
                target_location_id: int = None) -> int:
    """Create a new task in database and return task_id"""
    # Convert to Korean values for database
    db_task_type = '배달' if task_type == 'delivery' else '호출'
    
    with get_db_connection() as conn:
        with conn.cursor() as cur:
            cur.execute("""
                INSERT INTO tasks (task_type, status, requester_resident_id, item_id, target_location_id)
                VALUES (%s, %s, %s, %s, %s)
                RETURNING task_id
            """, (db_task_type, '대기', requester_resident_id, item_id, target_location_id))
            
            task_id = cur.fetchone()[0]
            conn.commit()
            return task_id

def get_pending_tasks() -> List[Dict]:
    """Get all pending tasks ordered by created_at (FIFO)"""
    with get_db_connection() as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            cur.execute("""
                SELECT task_id, task_type, status, requester_resident_id, item_id, 
                       target_location_id, assigned_bot_id, created_at
                FROM tasks 
                WHERE status = '대기'
                ORDER BY created_at ASC
            """)
            return [dict(row) for row in cur.fetchall()]

def get_next_pending_task() -> Optional[Dict]:
    """Get the next pending task (oldest first)"""
    with get_db_connection() as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            cur.execute("""
                SELECT task_id, task_type, status, requester_resident_id, item_id, 
                       target_location_id, assigned_bot_id, created_at
                FROM tasks 
                WHERE status = '대기'
                ORDER BY created_at ASC
                LIMIT 1
            """)
            row = cur.fetchone()
            return dict(row) if row else None

def update_task_status(task_id: int, status: str, assigned_bot_id: int = None):
    """Update task status and optionally assign bot"""
    # Convert English status to Korean
    status_map = {
        'pending': '대기',
        'assigned': '할당',
        'in_progress': '이동중',
        'completed': '완료',
        'failed': '실패'
    }
    db_status = status_map.get(status, status)
    
    with get_db_connection() as conn:
        with conn.cursor() as cur:
            if assigned_bot_id:
                cur.execute("""
                    UPDATE tasks 
                    SET status = %s, assigned_bot_id = %s
                    WHERE task_id = %s
                """, (db_status, assigned_bot_id, task_id))
            else:
                cur.execute("""
                    UPDATE tasks 
                    SET status = %s
                    WHERE task_id = %s
                """, (db_status, task_id))
            conn.commit()

def complete_task(task_id: int):
    """Mark task as completed with timestamp"""
    with get_db_connection() as conn:
        with conn.cursor() as cur:
            cur.execute("""
                UPDATE tasks 
                SET status = '완료', completed_at = CURRENT_TIMESTAMP
                WHERE task_id = %s
            """, (task_id,))
            conn.commit()

def get_task_by_id(task_id: int) -> Optional[Dict]:
    """Get task by ID"""
    with get_db_connection() as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            cur.execute("""
                SELECT task_id, task_type, status, requester_resident_id, item_id, 
                       target_location_id, assigned_bot_id, created_at, completed_at
                FROM tasks 
                WHERE task_id = %s
            """, (task_id,))
            row = cur.fetchone()
            return dict(row) if row else None

def get_robot_current_task(bot_id: int) -> Optional[Dict]:
    """Get current assigned task for a robot"""
    with get_db_connection() as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            cur.execute("""
                SELECT task_id, task_type, status, requester_resident_id, item_id, 
                       target_location_id, assigned_bot_id, created_at
                FROM tasks 
                WHERE assigned_bot_id = %s AND status IN ('할당', '이동중', '집기중', '수령대기')
                ORDER BY created_at ASC
                LIMIT 1
            """, (bot_id,))
            row = cur.fetchone()
            return dict(row) if row else None

def get_timeout_tasks(timeout_minutes: int = 5) -> List[Dict]:
    """Get tasks that have been active longer than timeout_minutes"""
    with get_db_connection() as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            cur.execute("""
                SELECT task_id, task_type, status, requester_resident_id, item_id, 
                       target_location_id, assigned_bot_id, created_at,
                       EXTRACT(EPOCH FROM (NOW() - created_at))/60 as minutes_elapsed
                FROM tasks 
                WHERE status IN ('할당', '이동중', '집기중')
                AND EXTRACT(EPOCH FROM (NOW() - created_at))/60 > %s
                ORDER BY created_at ASC
            """, (timeout_minutes,))
            return [dict(row) for row in cur.fetchall()]

def fail_task(task_id: int, reason: str = "Timeout"):
    """Mark task as failed with reason"""
    with get_db_connection() as conn:
        with conn.cursor() as cur:
            cur.execute("""
                UPDATE tasks 
                SET status = '실패', completed_at = CURRENT_TIMESTAMP
                WHERE task_id = %s
            """, (task_id,))
            conn.commit()

def get_active_tasks_by_bot() -> Dict[int, Optional[Dict]]:
    """Get currently active tasks for each bot"""
    with get_db_connection() as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            cur.execute("""
                SELECT task_id, task_type, status, requester_resident_id, 
                       assigned_bot_id, created_at
                FROM tasks 
                WHERE status IN ('할당', '이동중', '집기중', '수령대기')
                  AND assigned_bot_id IS NOT NULL
                ORDER BY created_at ASC
            """)
            
            # Bot ID별로 그룹화
            bot_tasks = {}
            for row in cur.fetchall():
                bot_id = row['assigned_bot_id']
                bot_tasks[bot_id] = dict(row)
                
            return bot_tasks