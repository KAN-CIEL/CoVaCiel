# -*- coding: utf-8 -*-
import mysql.connector
import json
import os
import time

class EnregistreurCovaciel:
    """
    Cette classe gère toute la sauvegarde vers MariaDB.
    Elle est conçue pour être utilisée par le cerveau de la voiture.
    """
    def __init__(self, host="localhost", user="root", password="ciel", database="covaciel_db"):
        try:
            # 1. Connexion à la base
            self.conn = mysql.connector.connect(
                host=host, user=user, password=password, database=database
            )
            self.cursor = self.conn.cursor()
            
            # 2. Création automatique de la session 
            self.cursor.execute("INSERT INTO sessions_course (date_course) VALUES (NOW())")
            self.conn.commit()
            self.id_session = self.cursor.lastrowid
            print(f"[LOG] Session démarrée avec l'ID : {self.id_session}")
        except Exception as e:
            print(f"[ERREUR BDD] Connexion impossible : {e}")
            self.conn = None

    def lire_ia_couleur(self):
        """Lit le fichier produit par ton IA de détection de couleurs"""
        if os.path.exists("vision_data.json"):
            try:
                with open("vision_data.json", "r") as f:
                    data = json.load(f)
                    return data.get("vert", "null"), data.get("rouge", "null")
            except: pass
        return "null", "null"

    def sauvegarder_trame(self, code_cmd, data_bytes, lidar_val=0):
        """
        Sauvegarde une trame UART/SPI dans la base de données.
        code_cmd : 0x05 (Moteur), 0x07 (Servo), 0x8A (Vitesse), etc.
        data_bytes : La liste des octets de données (payload)
        """
        if not self.conn: return

        # Valeurs par défaut
        vitesse = 0.0
        angle = 86  # Valeur neutre par défaut
        
        # --- ANALYSE DES TRAMES  ---
        if code_cmd == 0x05: # SET_MOTEUR : data_bytes[0] est la vitesse
            vitesse = data_bytes[0]
        
        elif code_cmd == 0x07: # SET_SERVO : data_bytes[0] est l'angle
            angle = data_bytes[0]
            
        elif code_cmd == 0x8A: # VITESSE 
            vitesse = data_bytes[0] # Extraction simplifiée pour l'exemple

        # Récupération des données IA
        vert, rouge = self.lire_ia_couleur()

        try:
            # 1. Insertion Physique (Vitesse, Direction, Lidar)
            self.cursor.execute(
                "INSERT INTO mesures_physiques (vitesse_kmh, direction, lidar) VALUES (%s, %s, %s)",
                (vitesse, angle, lidar_val)
            )
            id_p = self.cursor.lastrowid

            # 2. Insertion Couleur (IA)
            self.cursor.execute(
                "INSERT INTO mesures_couleur (obstacle_vert, obstacle_rouge) VALUES (%s, %s)",
                (vert, rouge)
            )
            id_c = self.cursor.lastrowid

            # 3. Insertion Globale (Lien entre tout)
            self.cursor.execute(
                "INSERT INTO mesures_globales (id_session, id_physique, id_couleur) VALUES (%s, %s, %s)",
                (self.id_session, id_p, id_c)
            )
            self.conn.commit()
        except Exception as e:
            print(f"[SQL ERROR] Échec de l'enregistrement : {e}")

    def fermer(self):
        if self.conn:
            self.cursor.close()
            self.conn.close()

if __name__ == "__main__":
    # Test manuel de la classe
    test = EnregistreurCovaciel()
    print("Tentative d'enregistrement de test...")
    # On simule une trame moteur (0x05) avec une vitesse de 15
    test.sauvegarder_trame(0x05, [15, 0, 0, 0, 0])
    print("Test terminé, vérifie phpMyAdmin !")
    test.fermer()
