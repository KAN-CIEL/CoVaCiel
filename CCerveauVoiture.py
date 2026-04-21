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

        self.distance_arret = 300 # mm
        self.SEUIL_PENTE = 120
        self.SEUIL_MUR_INT = 550 # mm

        self.historique_obstacles = {}
        self.last_cmd_t = 0
        self.cmd_interval = 0.05 # 20Hz (50ms)
        self.etat_voie = "LIGNE DROITE"
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
                
                # Variables persistantes pour le log
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
                        gauche, droit = self.gestion_lidar.get_secteurs()

                        pente_gauche = self.gestion_lidar.pente_moyenne(gauche)
                        pente_droite = self.gestion_lidar.pente_moyenne(droit)

                        if abs(pente_gauche) < self.SEUIL_PENTE and abs(pente_droite) < self.SEUIL_PENTE:
                            self.etat_voie = "LIGNE_DROITE"
                        elif pente_droite > self.SEUIL_PENTE:
                            self.etat_voie = "COURBE_GAUCHE"
                        elif pente_droite < -self.SEUIL_PENTE:
                            self.etat_voie = "COURBE_DROITE"
                        else:
                            self.etat_voie = "LIGNE_DROITE"

                        d_proche_str = plus_proche[2] if plus_proche else "N/A"
                        d_loin_str = plus_loin[2] if plus_loin else "N/A"

                        t_now = time.time()

                        # --- LOGIQUE DE SÉCURITÉ ---
                        if plus_proche:
                            q, a_proche, d_proche_val = plus_proche
                            if d_proche_val < self.distance_arret:
                                print(f"!!! STOP : OBSTACLE À {d_proche_val}mm !!!")
                                com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')
                                com.send_command(0x07, b'\x56\x00\x00\x00\x00\x00') # 86 en hexa (centre)
                                self.log_data(0, 0, 86, d_proche_val, d_loin_str)
                                continue

                        # --- LOGIQUE DE NAVIGATION (20Hz) ---
                        if t_now - self.last_cmd_t > self.cmd_interval:
                            if plus_loin:
                                mur_exterieur = 1500
                                angle_base = 0

                                # Détection grand virage vide
                                if self.etat_voie == "COURBE_GAUCHE":
                                    if gauche and (sum(gauche)/len(gauche)) > mur_exterieur:
                                        angle_base = -30
                                elif self.etat_voie == "COURBE_DROITE":
                                    if droit and (sum(droit)/len(droit)) > mur_exterieur:
                                        angle_base = 30

                                # Calcul des composantes d'angle
                                angle_centre = self.calc_angle_centre(gauche, droit)
                                angle_virage = self.calc_angle_virage(self.etat_voie)
                                angle_decal = self.calc_angle_decalage(self.etat_voie, gauche, droit)

                                # Fusion des décisions
                                if angle_base != 0:
                                    angle_destination = angle_base + angle_decal
                                else:
                                    angle_destination = angle_centre + angle_virage + angle_decal

                                # Limitation finale (-30° à +30°)
                                angle_destination = max(-30, min(30, angle_destination))

                                # Conversion en valeur Servo (64-106)
                                servo_val = self.converssion_angle(angle_destination)

                                # ENVOI UNIQUE DES COMMANDES
                                trame_servo = bytes([servo_val, 0, 0, 0, 0, 0])
                                com.send_command(0x07, trame_servo)
                                com.send_command(0x05, b'\x1A\x00\x00\x00\x00\x00') # Vitesse constante

                                q, angle_loin, dist_max = plus_loin
                                print(f"Dest: {angle_destination:.1f}° | Servo: {servo_val} | Dist: {dist_max}mm")

                            self.last_cmd_t = t_now
                        
                        # Logging
                        self.log_data(angle_destination, dist_max, servo_val, d_proche_str, d_loin_str)

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
        """ Mappe -30°/+30° vers 64/106 (Centre 86) """
        angle_norm = max(-30, min(30, angle))
        # Ratio : (106-86)/30 = 0.666
        servo_angle = 86 + (angle_norm * (20 / 30))
        final_val = int(round(servo_angle))
        return max(64, min(106, final_val))
    
    def calc_angle_centre(self, gauche, droit):
        if not gauche or not droit:
            return 0
        d_g = sum(gauche)/len(gauche)
        d_d = sum(droit)/len(droit)
        if d_g > d_d:
            return -30 # Aller à gauche
        elif d_g < d_d:
            return 30  # Aller à droite
        return 0

    def calc_angle_virage(self, etat):
        if etat == "COURBE_GAUCHE":
            return -5
        if etat == "COURBE_DROITE":
            return 5
        return 0
    
    def calc_angle_decalage(self, etat, gauche, droit):
        if etat == "COURBE_GAUCHE":
            moy_d = sum(droit)/len(droit) if droit else 0
            if moy_d > self.SEUIL_MUR_INT:
                return 10
        if etat == "COURBE_DROITE":
            moy_g = sum(gauche)/len(gauche) if gauche else 0
            if moy_g > self.SEUIL_MUR_INT:
                return -10
        return 0

    def stop_detection(self):
        self.lidar.stop_lidar()


