import socket
import json
import threading
from flask import Flask, jsonify

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4000
BUFFER_SIZE = 1024

# Stockage des données
latest_data = {}

# --- SERVEUR UDP ---
def udp_listener():
    """Écoute en UDP et stocke les dernières données reçues."""
    global latest_data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"🖥️ Serveur UDP en écoute sur {UDP_IP}:{UDP_PORT}")
    
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        try:
            latest_data = json.loads(data.decode('utf-8'))
            print(f"📥 Données reçues de {addr}: {latest_data}")
        except json.JSONDecodeError:
            print("❌ Erreur : données mal formatées")

# --- SERVEUR API ---
app = Flask(__name__)

@app.route('/data', methods=['GET'])
def get_data():
    """Retourne les dernières données IMU et GPS"""
    return jsonify(latest_data)

# --- LANCEMENT DES THREADS ---
udp_thread = threading.Thread(target=udp_listener, daemon=True)
udp_thread.start()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
