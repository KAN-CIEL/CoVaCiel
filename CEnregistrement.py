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


    def sauvegarder_trame(self, code_cmd, data_bytes):
        if not self.conn: return
        
        # 1. LE MOUCHARD : Si cette ligne ne s'affiche pas dans ton terminal, 
        # c'est que main.py n'envoie RIEN à la base de données !
        print(f"[DEBUG BDD] Tentative de sauvegarde -> Code: {code_cmd} | Data: {data_bytes}")

        vitesse, angle, tension = 0.0, 50, 0.0
        
        # 2. LA SÉCURITÉ : On vérifie qu'il y a bien des octets avant de lire data_bytes[0]
        if isinstance(data_bytes, (list, bytes, bytearray)) and len(data_bytes) > 0:
            if code_cmd == 0x84 or code_cmd == 0x05: # MOTEUR
                vitesse = data_bytes[0]
            elif code_cmd == 0x86 or code_cmd == 0x07: # SERVO/ANGLE
                angle = data_bytes[0]
            elif code_cmd == 0x89 or code_cmd == 0x09: # BATTERIE
                tension = data_bytes[0] / 10.0 
            elif code_cmd == 0x8A or code_cmd == 0x0A: # VITESSE REELLE
                vitesse = data_bytes[0]

        vert, rouge = self.lire_ia_couleur()

        try:
            self.cursor.execute("INSERT INTO Mesures_Physiques (vitesse_kmh, angle) VALUES (%s, %s)", (vitesse, angle))
            id_p = self.cursor.lastrowid

            self.cursor.execute("INSERT INTO Mesures_Electriques (batterie_tension) VALUES (%s)", (tension,))
            id_e = self.cursor.lastrowid

            self.cursor.execute("INSERT INTO Mesures_couleur (obstacle_vert, obstacle_rouge) VALUES (%s, %s)", (vert, rouge))
            id_c = self.cursor.lastrowid

            self.cursor.execute("INSERT INTO Mesures_Globales (id_session, id_physique, id_electrique, id_couleur) VALUES (%s, %s, %s, %s)", 
                                (self.id_session, id_p, id_e, id_c))
            self.conn.commit()
            print(f"[BDD] Ligne enregistrée avec succès !")
        except Exception as e:
            print(f"[SQL Error] : {e}")

    def fermer(self):
        if self.conn:
            self.cursor.close()
            self.conn.close()


    
