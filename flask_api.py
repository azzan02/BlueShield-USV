from flask import Flask, request, jsonify
import sqlite3
import datetime

app = Flask(__name__)

# Initialize database
def init_db():
    conn = sqlite3.connect('lora_data.db')
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS sensor_data (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    ph REAL, ec REAL, tds REAL, do REAL,
                    latitude REAL, longitude REAL, altitude REAL, speed REAL,
                    satellites INTEGER, timestamp TEXT)''')
    conn.commit()
    conn.close()

init_db()

# Route to receive data from LoRa Gateway
@app.route('/data', methods=['POST'])
def receive_data():
    try:
        data = request.json  # Expecting JSON payload

        ph = float(data.get("pH", 0))
        ec = float(data.get("EC", 0))
        tds = float(data.get("TDS", 0))
        do = float(data.get("DO", 0))
        latitude = float(data.get("Lat", 0))
        longitude = float(data.get("Lon", 0))
        altitude = float(data.get("Alt", 0))
        speed = float(data.get("Speed", 0))
        satellites = int(data.get("Sat", 0))
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Store data in database
        conn = sqlite3.connect('lora_data.db')
        c = conn.cursor()
        c.execute("INSERT INTO sensor_data (ph, ec, tds, do, latitude, longitude, altitude, speed, satellites, timestamp) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                  (ph, ec, tds, do, latitude, longitude, altitude, speed, satellites, timestamp))
        conn.commit()
        conn.close()

        return jsonify({"message": "Data received successfully", "status": "success"}), 200

    except Exception as e:
        return jsonify({"message": "Error processing data", "error": str(e)}), 400

# Route to retrieve stored data
@app.route('/data', methods=['GET'])
def get_data():
    conn = sqlite3.connect('lora_data.db')
    c = conn.cursor()
    c.execute("SELECT * FROM sensor_data ORDER BY timestamp DESC LIMIT 50")  # Get latest 50 entries
    rows = c.fetchall()
    conn.close()

    data_list = []
    for row in rows:
        data_list.append({
            "id": row[0], "pH": row[1], "EC": row[2], "TDS": row[3], "DO": row[4],
            "Lat": row[5], "Lon": row[6], "Alt": row[7], "Speed": row[8], "Sat": row[9], "Timestamp": row[10]
        })

    return jsonify(data_list)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
