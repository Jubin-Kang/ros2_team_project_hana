# web/data_access.py
from typing import Optional, List, Dict
from .db import query_db, update_db


# ─────────────────────  Auth  ─────────────────────
def fetch_user(login_id: str, password: str) -> Optional[dict]:
    """
    로그인 ID / 비밀번호가 맞으면 {resident_id, name, role} dict 반환.
    """
    with query_db() as cur:
        cur.execute(
            """SELECT resident_id, name
               FROM residents
               WHERE login_id = %s
                 AND password = crypt(%s, password)""",
            (login_id, password)
        )
        row = cur.fetchone()
    
    if row is None:
        return None
    
    # 모든 사용자를 일반 사용자로 설정
    role = "user"
    
    return {"resident_id": row[0], "name": row[1], "role": role}


def insert_login_log(resident_id: int | None, success: bool, ip: str) -> None:
    with update_db() as cur:
        cur.execute(
            "INSERT INTO login_logs (resident_id, success, client_ip) VALUES (%s,%s,%s)",
            (resident_id, success, ip)
        )


# ─────────────────────  Items / Orders  ─────────────────────
def fetch_items() -> list[dict]:
    with query_db() as cur:
        cur.execute(
            "SELECT item_id, item_type, item_quantity FROM items ORDER BY item_id"
        )
        rows = cur.fetchall()
    return [{"id": r[0], "type": r[1], "quantity": r[2]} for r in rows]


def decrement_stock(items: list[dict]) -> None:
    """
    items = [{"id": 1, "quantity": 2}, ...]
    """
    with update_db() as cur:
        for it in items:
            cur.execute(
                "UPDATE items SET item_quantity = item_quantity - %s WHERE item_id = %s",
                (it["quantity"], it["id"])
            )
