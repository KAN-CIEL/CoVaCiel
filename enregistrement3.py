# -*- coding: utf-8 -*-
import mysql.connector
import time
import os
import json
import struct
import math
from smbus2 import SMBus
# import board
# import busio
# import adafruit_bno055

# ==========================================
# CLASSE 1 : GESTION DE LA BASE DE DONNEES
# ==========================================
class DatabaseManager:
    def __init__(self, host, user, password, database):
        print("Initialisation de la base de donnees...")
        self.db = mysql.connector.connect(
            host=host,
            user=user,
            password=password,
            database=database
        )
        self.cursor = self.db.cursor()
        self.id_session = self.start_session()

    def start_session(self):
        # ATTENTION : J'ai mis 'sessions_course' en minuscule. 
        # Si dans phpMyAdmin c'est 'Sessions_course', modifie la ligne ci-dessous !
        self.cursor.execute("INSERT INTO sessions_course (date_course) VALUES (NOW())")
        self.db.commit()
        session_id = self.cursor.lastrowid
        print(f"--- SESSION DEMARREE : ID {session_id} ---")
        return session_id

    def insert_telemetry(self, data):
        try:
            # 1. Insertion ELECTRIQUE
            sql_elec = "INSERT INTO Mesures_Electriques (batterie_tension, batterie_courant) VALUES (%s, %s)"
            self.cursor.execute(sql_elec, (data['tension'], data['courant']))
            id_elec = self.cursor.lastrowid 

            # 2. Insertion PHYSIQUE
            sql_phys = "INSERT INTO Mesures_Physiques (vitesse_kmh, acceleration, magnetometre, direction, distance_parcourue, lidar) VALUES (%s, %s, %s, %s, %s, %s)"
            self.cursor.execute(sql_phys, (data['vitesse'], data['accel'], data['magneto'], data['direction'], data['distance'], data['lidar']))
            id_phys = self.cursor.lastrowid 

            # 3. Insertion COULEUR (Via IA Camera)
            sql_coul = "INSERT INTO Mesures_couleur (obstacle_vert, obstacle_rouge) VALUES (%s, %s)"
            id_couleur = self.cursor.lastrowid

            # 4. Insertion GLOBALE
            sql_glob = "INSERT INTO Mesures_Globales (id_session, id_physique, id_electrique, id_couleur) VALUES (%s, %s, %s, %s)"
            self.cursor.execute(sql_glob, (self.id_session, id_phys, id_elec, id_couleur))
            
            self.db.commit()
            print(f"[OK] Vitesse: {data['vitesse']} | Couleur: {data['couleur']}")

        except Exception as e:
            print(f"Erreur d'insertion SQL : {e}")
            self.db.rollback()
        
    def close(self):
        print("Fermeture de la connexion BD.")
        self.cursor.close()
        self.db.close()

# ==========================================
# CLASSE 2 : LECTURE I2C DE L'ARDUINO
# ==========================================
class CapteurArduinoI2C:
    def __init__(self, adresse=0x08):
        print(f"Ouverture de la ligne I2C vers l'Arduino (Adresse {adresse})...")
        try:
            self.bus = SMBus(1) # Port I2C par defaut du Raspberry Pi
            self.adresse = adresse
            self.actif = True
        except:
            print("[ATTENTION] Bus I2C introuvable (Simulation active)")
            self.actif = False

    def lire_donnees(self):
        # Si le capteur n'est pas actif dès le départ, on renvoie 0
        if not self.actif:
            return 0.0, 0.0, 0.0
            
        try:
            # Tentative de lecture sur le bus I2C
            donnees_brutes = self.bus.read_i2c_block_data(self.adresse, 0, 12)
            valeurs = struct.unpack('<3f', bytearray(donnees_brutes))
            
            # --- SÉCURITÉ : On vérifie chaque valeur ---
            # Si une valeur est "nan" ou "inf", on la transforme en 0.0
            v1 = valeurs[0] if math.isfinite(valeurs[0]) else 0.0
            v2 = valeurs[1] if math.isfinite(valeurs[1]) else 0.0
            v3 = valeurs[2] if math.isfinite(valeurs[2]) else 0.0
            
            return v1, v2, v3
            
        except Exception as e:
            # Si le câble est débranché ou qu'il y a une erreur I2C :
            # On affiche l'erreur dans la console mais on renvoie 0 pour la BDD
            print(f"[ERREUR I2C] Lecture impossible : {e}")
            return 0.0, 0.0, 0.0

# ==========================================
# CLASSE 3 : LECTURE DE L'IA CAMERA (JSON)
# ==========================================
class VisionIA_Lecteur:
    def __init__(self, filepath="vision_data.json"):
        self.filepath = filepath

    def get_couleur_log(self):
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, "r") as fichier:
                    vision = json.load(fichier)
                    vert = vision.get("vert", "null")
                    rouge = vision.get("rouge", "null")
                    
                    if vert != "null" and rouge != "null":
                        return f"VERT:{vert} / ROUGE:{rouge}"
                    elif vert != "null":
                        return f"VERT a {vert}"
                    elif rouge != "null":
                        return f"ROUGE a {rouge}"
                    else:
                        return "Aucun obstacle"
            except Exception:
                return "Erreur lecture IA"
        return "IA hors ligne"
    
# ==========================================
# CLASSE 4 : LE CONTROLEUR PRINCIPAL (ROBOT)
# ==========================================
class RobotController:
    def __init__(self):
        print("--- DEMARRAGE DU SYSTEME COVACIEL ---")
        self.db_manager = DatabaseManager("localhost", "root", "ciel", "covaciel_db")
        # Changement d'argument pour correspondre e  la classe CapteurArduinoI2C
        self.arduino = CapteurArduinoI2C(adresse=0x00)
        self.vision = VisionIA_Lecteur()

    def get_empty_sensors(self):
        return {
            'tension': 0.0, 'courant': 0.0, 'vitesse': 0.0, 'accel': 0.0,
            'magneto': 0.0, 'distance': 0.0, 'lidar': "{}", 'direction': 0.0, 
            'couleur': "Inconnu"
        }

    def run(self):
        try:
            while True:
                data = self.get_empty_sensors()
                
                # 1. Lecture de l'Arduino
                accel, vitesse, distance = self.arduino.lire_donnees()
                data['accel'] = round(accel, 2)
                data['vitesse'] = round(vitesse, 2)
                data['distance'] = round(distance, 2)

                # 2. Lecture de la Camera (IA)
                data['couleur'] = self.vision.get_couleur_log()

                # 3. Envoi e  la base de donnees
                self.db_manager.insert_telemetry(data)

                # 4. On ecoute e*  la même vitesse que la camera
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nerreur keyborard.")
        finally:
            self.db_manager.close()
            print("--- SYSTEME ETEINT ---")

if __name__ == "__main__":
    robot = RobotController()
    robot.run()
