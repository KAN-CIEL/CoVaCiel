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

        self.distance_arret = 200 # mm
        self.SEUIL_PENTE = 150
        self.SEUIL_MUR_INT = 450 # mm
        self.SEUIL_MUR_EXT = 300 # mm

        self.last_cmd_t = 0
        self.cmd_interval = 0.05 # 20Hz (50ms)
        self.etat_voie = "LIGNE_DROITE"
        self.log_file = "lidar_logs.csv"
        
        # Gain Proportionnel pour le centrage
        self.Kp = 0.024 
        self.Kd = 0.008
        self.last_erreur = 0
        self._init_log_file()
    
    def _init_log_file(self):
        """ Crée le fichier de log avec l'en-tête """
        with open(self.log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Etat", "Angle_Dest", "Servo", "Obs_Proche", "Obs_Loin"])
    
    def log_data(self, etat, angle_dest, servo_val, obs_proche_dist, obs_loin_dist):
        """ Enregistre les données dans le CSV """
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, etat, f"{angle_dest:.1f}", servo_val, obs_proche_dist, obs_loin_dist])

    def start_detection(self, signal, com):
        if self.lidar.start_lidar():
            com.start()
            print("Analyse active. Mode Navigation engagé. Logging actif.")
            try:
                scans = self.lidar.lidar.iter_scans()
                
                angle_destination = 0.0
                servo_val = 86

                for scan in scans:
                    try:
                        # 1. Filtrage et Analyse
                        self.gestion_lidar.filtrer_tout_en_un(scan)
                        
                        plus_proche = self.gestion_lidar.get_obstacle_proche()
                        plus_loin = self.gestion_lidar.get_obstacle_loin()
                        gauche, droit = self.gestion_lidar.get_secteurs()

                        pente_droite = self.gestion_lidar.pente_moyenne(droit)

                        # Détection de l'état de la voie
                        if pente_droite > self.SEUIL_PENTE:
                            self.etat_voie = "COURBE_GAUCHE"
                        elif pente_droite < -self.SEUIL_PENTE:
                            self.etat_voie = "COURBE_DROITE"
                        else:
                            self.etat_voie = "LIGNE_DROITE"

                        t_now = time.time()

                        # --- LOGIQUE DE SÉCURITÉ (STOP) ---
                        if plus_proche:
                            _, _, d_proche_val = plus_proche
                            if d_proche_val < self.distance_arret:
                                print(f"!!! STOP : {d_proche_val}mm !!!")
                                com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')
                                com.send_command(0x07, b'\x56\x00\x00\x00\x00\x00')
                                continue

                        # --- LOGIQUE DE NAVIGATION OPTIMISÉE (20Hz) ---
                        if t_now - self.last_cmd_t > self.cmd_interval:
                            
                            # Calcul de l'erreur de centrage
                            angle_centre = self.calc_angle_centre(gauche, droit)
                            
                            # Détermination de la cible selon l'état de la voie
                            if self.etat_voie == "LIGNE_DROITE":
                                target = angle_centre
                            elif self.etat_voie == "COURBE_GAUCHE":
                                target = -25 + angle_centre
                            elif self.etat_voie == "COURBE_DROITE":
                                target = 25 + angle_centre
                            
                            # Lissage de l'angle (Filtre passe-bas : 60% ancienne valeur, 40% nouvelle)
                            angle_destination = (angle_destination * 0.6) + (target * 0.4)

                            # Bornage final et Conversion
                            angle_destination = max(-25, min(25, angle_destination))
                            servo_val = self.conversion_angle(angle_destination)

                            # Envoi des commandes
                            trame_servo = bytes([servo_val, 0, 0, 0, 0, 0])
                            com.send_command(0x07, trame_servo)
                            if self.etat_voie == "LIGNE_DROITE":
                                com.send_command(0x05, b'\x19\x00\x00\x00\x00\x00') # Vitesse stable
                            elif self.etat_voie == "COURBE_GAUCHE" or self.etat_voie == "COURBE_DROITE":
                                com.send_command(0x05, b'\x1a\x00\x00\x00\x00\x00') # Légèrement plus lent pour les courbes

                            self.last_cmd_t = t_now
                            
                            # Log des données
                            d_p = plus_proche[2] if plus_proche else "N/A"
                            d_l = plus_loin[2] if plus_loin else "N/A"
                            self.log_data(self.etat_voie, angle_destination, servo_val, d_p, d_l)

                        # Signal d'arrêt externe
                        if signal.read_signal() == "STOP": 
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

    def conversion_angle(self, angle):
        """ Mappe -30°/+30° vers 62/109 (Milieu 86) """
        angle_norm = max(-30, min(30, angle))
        servo_angle = 86 + (angle_norm * (27 / 30))
        return max(62, min(109, int(round(servo_angle))))
    
    def calc_angle_centre(self, gauche, droit):
        """ Calcule l'erreur de centrage (Proportionnel) """
        if not gauche or not droit or len(gauche) == 0 or len(droit) == 0:
            return 0
        
        moy_g = sum(gauche)/len(gauche)
        moy_d = sum(droit)/len(droit)
        
        erreur = moy_g - moy_d

        derivee = erreur - self.last_erreur
        self.last_erreur = erreur

        commande = (erreur * self.Kp) + (derivee * self.Kd)
        return -commande

    def stop_detection(self):
        self.lidar.stop_lidar()


