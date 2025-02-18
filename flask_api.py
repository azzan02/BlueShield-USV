from flask import Flask, request, jsonify
from flask_sqlalchemy import SQLAlchemy
import datetime
from flask_cors import CORS

app = Flask(__name__)

# Enable CORS for all routes and origins
CORS(app)

# Database configuration (SQLite for simplicity)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///sensor_data.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# Define a model for storing sensor data
class SensorData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    node_id = db.Column(db.String(50), nullable=False)
    ph = db.Column(db.Float, nullable=False)
    ec = db.Column(db.Float, nullable=False)
    tds = db.Column(db.Float, nullable=False)
    do = db.Column(db.Float, nullable=False)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    altitude = db.Column(db.Float, nullable=False)
    speed = db.Column(db.Float, nullable=False)
    satellites = db.Column(db.Integer, nullable=False)
    timestamp = db.Column(db.String(50), nullable=False)

    def __repr__(self):
        return f"<SensorData {self.node_id} - {self.timestamp}>"

# Create the database and tables
with app.app_context():
    db.create_all()

# Route to receive data from the LoRa gateway
@app.route('/api/sensor-data', methods=['POST'])
def receive_sensor_data():
    data = request.json  # Assuming the gateway sends JSON data

    # Validate the incoming data
    required_fields = ["node_id", "pH", "EC", "TDS", "DO", "Lat", "Lon", "Alt", "Speed", "Sat"]
    if not data or not all(field in data for field in required_fields):
        return jsonify({"error": "Invalid data format"}), 400

    # Parse and store the data
    sensor_data = SensorData(
        node_id=data["node_id"],
        ph=float(data.get("pH", 0)),
        ec=float(data.get("EC", 0)),
        tds=float(data.get("TDS", 0)),
        do=float(data.get("DO", 0)),
        latitude=float(data.get("Lat", 0)),
        longitude=float(data.get("Lon", 0)),
        altitude=float(data.get("Alt", 0)),
        speed=float(data.get("Speed", 0)),
        satellites=int(data.get("Sat", 0)),
        timestamp=datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    )
    db.session.add(sensor_data)
    db.session.commit()

    return jsonify({"message": "Data received and stored successfully"}), 201

# Route to retrieve all sensor data
# @app.route('/api/sensor-data', methods=['GET'])
# def get_sensor_data():
#     sensor_data = SensorData.query.all()
#     result = []
#     for data in sensor_data:
#         result.append({
#             "id": data.id,
#             "node_id": data.node_id,
#             "pH": data.ph,
#             "EC": data.ec,
#             "TDS": data.tds,
#             "DO": data.do,
#             "Lat": data.latitude,
#             "Lon": data.longitude,
#             "Alt": data.altitude,
#             "Speed": data.speed,
#             "Sat": data.satellites,
#             "timestamp": data.timestamp
#         })
#     return jsonify(result), 200


@app.route('/api/sensor-data', methods=['GET'])
def get_sensor_data():
    
    result = []
    result.append({
        "id": 12,
        "node_id": 12,
        "pH": 7.2,
        "EC": 1.5,
        "TDS": 500,
        "DO": 6.8,
        "Lat": 33.684,
        "Lon": 12,
        "Alt": 12,
        "Speed": 12,
        "Sat": 12,
        "timestamp": 12
    })
    return jsonify(result), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)