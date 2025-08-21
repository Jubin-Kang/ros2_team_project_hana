from flask import Blueprint, jsonify
from ..data_access import fetch_items   # 🔹 SQL 함수만 임포트

bp = Blueprint("items", __name__, url_prefix="/items")

@bp.route("", methods=["GET"])
def list_items():
    return jsonify(fetch_items())
