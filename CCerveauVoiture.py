from CDetection_Lidar import CDetection
from CGestion_Lidar import CGestion

import time
import csv
import datetime
import os

class CCerveau:
    def __init__(self):
        self.lidar = CDetection()
        self.gestion_lidar = CGestion()
        self.historique_obstacles = {}
        self.last_cmd_t = 0
        self.cmd_interval = 0.05 # 20Hz
        self.log_file = "lidar_logs.csv"
        self._init_log_file()
    
    def _init_log_file(self):
        """ Écrase l'ancien fichier et écrit l'en-tête """
        with open(self.log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Angle_Destination", "Distance_Max", "Obstacle_Proche_Dist", "Obstacle_Loin_Dist"])
    
    def log_data(self, angle_dest, dist_max, servo_cmd = "N/A", obs_proche_dist = "N/A", obs_loin_dist = "N/A"):
        """ Enregistre les données dans le fichier CSV """
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, f"{angle_dest:.1f}", dist_max, servo_cmd, obs_proche_dist, obs_loin_dist])

    def start_detection(self, signal, com):
        if self.lidar.start_lidar():
            com.start()
            print("Analyse active. Mode Navigation engagé. Logging actif.")
            try:
                scans = self.lidar.lidar.iter_scans()
                
                # Variables persistantes pour le log si un scan est vide
                angle_destination = 0.0
                dist_max = 0
                servo_val = 86

                for scan in scans:
                    try:
                        # 1. Filtrage
                        self.gestion_lidar.filtrer_tout_en_un(scan)
                        
                        # 2. Analyse des points
                        plus_proche = self.gestion_lidar.get_obstacle_proche()
                        plus_loin = self.gestion_lidar.get_obstacle_loin()

                        d_proche_str = plus_proche[2] if plus_proche else "N/A"
                        d_loin_str = plus_loin[2] if plus_loin else "N/A"

                        t_now = time.time()

                        # --- LOGIQUE DE SÉCURITÉ ---
                        if plus_proche:
                            q, a_proche, d_proche_val = plus_proche
                            if d_proche_val < 200:
                                print(f"!!! STOP : OBSTACLE à {d_proche_val}mm !!!")
                                com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')
                                com.send_command(0x07, b'\x40\x00\x00\x00\x00\x00')
                                self.log_data(0, 0, 86, d_proche_val, d_loin_str)
                                continue

                        # --- LOGIQUE DE NAVIGATION ---
                        if t_now - self.last_cmd_t > self.cmd_interval:
                            if plus_loin:
                                q, angle_destination, dist_max = plus_loin
                                servo_val = self.converssion_angle(angle_destination)

                                # Envoi commandes ESP32
                                trame_servo = bytes([servo_val, 0, 0, 0, 0, 0])
                                com.send_command(0x07, trame_servo)
                                com.send_command(0x05, b'\x19\x00\x00\x00\x00\x00')
                                
                                print(f"Dest: {angle_destination:.1f}° | Servo: {servo_val} | Dist: {dist_max}mm")

                            self.last_cmd_t = t_now
                        
                        # Logging systématique du scan
                        self.log_data(angle_destination, dist_max, d_proche_str, d_loin_str)

                        # Arrêt XBEE
                        msg = signal.read_signal()
                        if msg == "STOP": 
                            com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')
                            com.send_command(0x07, b'\x56\x00\x00\x00\x00\x00')
                            break

                    except ValueError:
                        self.lidar.lidar.clear_input()
                        continue
                        
            except Exception as e:
                print(f"Erreur majeure : {e}")
            finally:
                self.lidar.stop_lidar()


    def converssion_angle(self, angle):
        if angle > 180:
            angle_norm = angle - 360
        else:
            angle_norm = angle
        
        if angle_norm > 30:
            return 106
        elif angle_norm <-30:
            return 64
                
        servo_angle = 86 + (angle_norm * (42 / 180))
        
        servo_int = int(round(servo_angle))
        return max(64, min(106, servo_int))

    def stop_detection(self):
        self.lidar.stop_lidar()


