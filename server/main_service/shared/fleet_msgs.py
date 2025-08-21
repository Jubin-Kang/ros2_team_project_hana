# Fleet communication message definitions
from dataclasses import dataclass
from typing import List, Dict, Any
from enum import Enum

class TaskType(Enum):
    DELIVERY = "delivery"
    
class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned" 
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

class RobotStatus(Enum):
    IDLE = "idle"
    BUSY = "busy"
    ERROR = "error"
    CHARGING = "charging"

@dataclass
class TaskRequest:
    task_id: str
    task_type: TaskType
    resident_id: str 
    items: List[Dict[str, Any]]
    priority: int = 1

@dataclass
class TaskResponse:
    task_id: str
    status: TaskStatus
    robot_id: str = None
    message: str = ""
    
@dataclass
class RobotState:
    robot_id: str
    status: RobotStatus
    current_task_id: str = None
    position: List[float] = None
    last_update: float = 0.0