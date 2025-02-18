import socket
import threading
import serial
from pymavlink import mavutil
from pynmeagps import NMEAReader, NMEAMessage
import time

# --- CONFIGURATION ---
UDP_SERVER_IP = "127.0.0.1"  # Adresse locale pour le serveur UDP
UDP_SERVER_PORT = 4000  # Port d'envoi des données
PIXHAWK_PORT = "/dev/ttyACM0"
BAUDRATE_PIXHAWK = 115200
GPS_PORT = "/dev/ttyUSB0"
BAUDRATE_GPS = 4800

# Stockage des dernières données lues
latest_imu_data = {}
latest_gps_data = {}

# Flag d'arrêt global
stop_flag = False

# --- CONNEXION À LA PIXHAWK ---
print(f"Connexion à la Pixhawk sur {PIXHAWK_PORT} à {BAUDRATE_PIXHAWK} bauds...")
master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUDRATE_PIXHAWK)
master.wait_heartbeat()
print("✅ Connexion établie avec la Pixhawk!")

# Demander les flux de données IMU
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
    10,  # Fréquence en Hz
    1
)

# --- FONCTION 1 : LECTURE DES DONNÉES CAPTEURS ---
def read_sensors():
    """Lit les données des capteurs Pixhawk (IMU) et GPS en continu"""
    global latest_imu_data, latest_gps_data, stop_flag

    # Lancer la lecture GPS en parallèle
    def gps_reader():
        global latest_gps_data
        try:
            with serial.Serial(GPS_PORT, BAUDRATE_GPS, timeout=1) as ser:
                print(f"📡 Lecture des données GPS sur {GPS_PORT}...")
                nmea_reader = NMEAReader(ser)

                while not stop_flag:
                    try:
                        (raw_data, parsed_data) = nmea_reader.read()
                        if isinstance(parsed_data, NMEAMessage):
                            if parsed_data.msgID == "GGA":  
                                latest_gps_data = {
                                    "type": "GGA",
                                    "lat": parsed_data.lat,
                                    "lon": parsed_data.lon,
                                    "alt": parsed_data.alt
                                }
                            elif parsed_data.msgID == "VTG":  
                                latest_gps_data = {
                                    "type": "VTG",
                                    "speed": parsed_data.spd_over_grnd_kmph
                                }
                    except Exception as e:
                        print(f"❌ Erreur GPS : {e}")
        except serial.SerialException as e:
            print(f"❌ Erreur d'accès au port GPS : {e}")

        print("📴 Lecture GPS arrêtée.")

    # Lancer GPS en thread
    gps_thread = threading.Thread(target=gps_reader, daemon=True)
    gps_thread.start()

    # Lecture des données Pixhawk en boucle
    while not stop_flag:
        msg = master.recv_match(type=['RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3', 'ATTITUDE'], blocking=True)
        if msg:
            msg_type = msg.get_type()
            if msg_type in ['RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3']:
                latest_imu_data = {
                    "type": msg_type,
                    "acc": {"x": msg.xacc, "y": msg.yacc, "z": msg.zacc},
                    "gyro": {"x": msg.xgyro, "y": msg.ygyro, "z": msg.zgyro},
                    "mag": {"x": msg.xmag, "y": msg.ymag, "z": msg.zmag}
                }
            elif msg_type == 'ATTITUDE':
                latest_imu_data = {
                    "type": "ATTITUDE",
                    "roll": msg.roll,
                    "pitch": msg.pitch,
                    "yaw": msg.yaw
                }

        time.sleep(0.1)  # Petite pause pour éviter de monopoliser le CPU

# --- FONCTION 2 : ENVOI DES DONNÉES VIA UDP ---
def send_udp_data():
    """Envoie les données IMU et GPS via UDP"""
    global latest_imu_data, latest_gps_data, stop_flag

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"📡 Envoi des données en UDP vers {UDP_SERVER_IP}:{UDP_SERVER_PORT}")

    while not stop_flag:
        data = {
            "imu": latest_imu_data,
            "gps": latest_gps_data
        }
        sock.sendto(str(data).encode('utf-8'), (UDP_SERVER_IP, UDP_SERVER_PORT))
        print(f"📤 Données envoyées : {data}")
        time.sleep(1)  # Envoi toutes les secondes

    sock.close()
    print("📴 Connexion UDP fermée.")

# --- LANCEMENT DES THREADS ---
sensor_thread = threading.Thread(target=read_sensors, daemon=True)
udp_thread = threading.Thread(target=send_udp_data, daemon=True)

sensor_thread.start()
udp_thread.start()

try:
    sensor_thread.join()
    udp_thread.join()
except KeyboardInterrupt:
    print("\n🛑 Interruption détectée, arrêt des processus.")
    stop_flag = True

print("🏁 Programme terminé.")
