from flask import Flask
from .routes import auth, items, orders, status, fleet_monitor, order2, camera
from .routes.status import initialize_robot_status_from_db

def create_app():
    app = Flask(__name__, template_folder="templates", static_folder="static")
    
    # Disable static file caching for development
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
    
    @app.after_request
    def after_request(response):
        # Prevent caching of dynamic content and static files during development
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        return response
    
    app.register_blueprint(auth.bp)
    app.register_blueprint(items.bp)
    app.register_blueprint(orders.bp)
    app.register_blueprint(status.bp)
    app.register_blueprint(fleet_monitor.bp)
    app.register_blueprint(order2.order2_bp)
    app.register_blueprint(camera.bp)
    
    # Initialize robot status cache from database on startup
    with app.app_context():
        initialize_robot_status_from_db()
    
    return app
