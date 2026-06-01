import mysql.connector
import json
import os

class CEnregistreurCovaciel:
    def __init__(self, host="localhost", user="root", password="ciel", database="covaciel_db"):
        self.json_path = os.path.join(os.path.dirname(__file__), "vision_data.json")
        try:
            self.conn = mysql.connector.connect(
                host=host, user=user, password=password, database=database
            )
            self.cursor = self.conn.cursor()
            self.cursor.execute("INSERT INTO sessions_course (date_course) VALUES (NOW())")
            self.conn.commit()
            self.id_session = self.cursor.lastrowid
            print(f"[LOG DB] Session ID: {self.id_session}")
        except Exception as e:
            print(f"[ERREUR BDD] : {e}")
            self.conn = None

    def lire_ia_couleur(self):
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, "r") as f:
                    data = json.load(f)
                    return data.get("vert", "null"), data.get("rouge", "null")
            except Exception as e:
                pass
        return "null", "null"

    def sauvegarder_donnees(self, vitesse, angle, lidar_val, batterie_val): #nico (Ajout de batterie_val)
        """Re�oit directement les valeurs num�riques calcul�es par CCerveau"""
        if not self.conn: return

        vert, rouge = self.lire_ia_couleur()

        try:
            # Insertion des donn�es physiques (Ajout de la batterie)
            self.cursor.execute(
                "INSERT INTO Mesures_Physiques (vitesse_kmh, angle, lidar) VALUES (%s, %s, %s)",
                (vitesse, angle, lidar_val) #nico (Correction de la variable "batterie" en "batterie_val")
            )
            id_p = self.cursor.lastrowid

            # 2. Insertion des donn�es �lectriques (Batterie) #nico
            self.cursor.execute(
                "INSERT INTO Mesures_Electriques (batterie_tension) VALUES (%s)",
                (batterie_val,)
            )
            id_e = self.cursor.lastrowid

            # Insertion des couleurs
            self.cursor.execute(
                "INSERT INTO Mesures_couleur (obstacle_vert, obstacle_rouge) VALUES (%s, %s)",
                (vert, rouge)
            )
            id_c = self.cursor.lastrowid

            # Liaison globale
            self.cursor.execute(
                "INSERT INTO Mesures_Globales (id_session, id_physique, id_couleur) VALUES (%s, %s, %s)", #nico (plus besoin de id_electrique)
                (self.id_session, id_p, id_c)
            )
            self.conn.commit()
        except Exception as e:
            print(f"[SQL ERROR] : {e}")
            self.conn.rollback() #nico

    def fermer(self):
        if self.conn:
            self.cursor.close()
            self.conn.close()