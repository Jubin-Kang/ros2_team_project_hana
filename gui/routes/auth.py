# web/routes/auth.py
from flask import Blueprint, request, jsonify, render_template
from ..utils import get_client_ip
from ..data_access import fetch_user, insert_login_log   # <- 쿼리 함수만 임포트

bp = Blueprint("auth", __name__)

@bp.route("/", methods=["GET"])
@bp.route("/login", methods=["GET"])
def index():
    return render_template("login.html")


@bp.route("/login", methods=["POST"])
def login():
    data = request.get_json()
    login_id = data.get("login_id")
    password = data.get("password")
    if not login_id or not password:
        return jsonify({"error": "login_id와 password 필요"}), 400

    try:
        # 1) 사용자 조회
        user = fetch_user(login_id, password)     # ← data_access 함수
        success = user is not None
        rid = user["resident_id"] if success else None

        # 2) 로그인 로그 기록
        insert_login_log(rid, success, get_client_ip(request))

        # 3) 응답
        if success:
            return jsonify(user)          # {"resident_id": ..., "name": ...}
        else:
            return jsonify({"error": "로그인 실패"}), 401

    except Exception as e:
        return jsonify({"error": str(e)}), 500
