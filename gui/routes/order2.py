from flask import Blueprint, render_template, request, jsonify

order2_bp = Blueprint('order2', __name__)

@order2_bp.route('/order2')
def order2_page():
    return render_template('order2.html')

@order2_bp.route('/api/order2/submit', methods=['POST'])
def submit_order2():
    data = request.json
    # Here you would typically process the order data, e.g., save to DB
    print(f"Received order2 data: {data}")
    return jsonify({"message": "Order2 submitted successfully!", "data": data}), 200
