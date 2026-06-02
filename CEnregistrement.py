import mysql.connector
import json
import os
import threading   #nico1

class CEnregistreurCovaciel:
    def __init__(self, host="localhost", user="root", password="ciel", database="covaciel_db"):
        self.json_path = os.path.join(os.path.dirname(__file__), "vision_data.json")
        self.lock = threading.Lock()   #nico1 : protege la connexion (acces concurrents)
        self.conn = None
        self.cursor = None
        self.id_session = None
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
        """ Lit le dernier resultat de la detection couleur (fichier JSON).
            Robuste : renvoie ('null', 'null') si le fichier manque/est corrompu. """
        if os.path.exists(self.json_path):
            try:
                with open(self.json_path, "r") as f:
                    data = json.load(f)
                    return data.get("vert", "null"), data.get("rouge", "null")
            except Exception:
                pass
        return "null", "null"

    def sauvegarder_donnees(self, vitesse, angle, lidar_statut, batterie_val):   #nico1
        """ Enregistre un point de mesure. Appelee a 5Hz max par le Cerveau.
            - vitesse      : float (km/h) deja parse cote UART
            - angle        : angle physique reel en degres (-30..+30), PAS le PWM servo
            - lidar_statut : chaine 'Oui' (arret d'urgence) ou 'Non' (voie libre)
            - batterie_val : float (tension) deja parse cote UART
            Robuste : valeurs par defaut si None/incoherent, thread-safe, reconnexion auto. """  #nico1
        if not self.conn:
            return

        # --- Garde-fous anti-crash : valeurs par defaut si trame vide/incomplete --- #nico1
        try:
            vitesse = float(vitesse)
        except (TypeError, ValueError):
            vitesse = 0.0
        try:
            batterie_val = float(batterie_val)
        except (TypeError, ValueError):
            batterie_val = 0.0
        try:
            angle = float(angle)
        except (TypeError, ValueError):
            angle = 0.0
        if lidar_statut not in ("Oui", "Non"):
            lidar_statut = "Non"

        vert, rouge = self.lire_ia_couleur()

        # --- Section critique : une seule requete a la fois sur la connexion --- #nico1
        with self.lock:
            try:
                # Reconnexion auto si MariaDB a coupe la connexion (timeout serveur) #nico1
                self.conn.ping(reconnect=True, attempts=3, delay=0.2)

                # 1. Mesures physiques (angle en degres, lidar en 'Oui'/'Non')
                self.cursor.execute(
                    "INSERT INTO Mesures_Physiques (vitesse_kmh, angle, lidar) VALUES (%s, %s, %s)",
                    (vitesse, angle, lidar_statut)
                )
                id_p = self.cursor.lastrowid

                # 2. Mesures electriques (batterie)
                self.cursor.execute(
                    "INSERT INTO Mesures_Electriques (batterie_tension) VALUES (%s)",
                    (batterie_val,)
                )

                # 3. Mesures couleur (IA)
                self.cursor.execute(
                    "INSERT INTO Mesures_couleur (obstacle_vert, obstacle_rouge) VALUES (%s, %s)",
                    (vert, rouge)
                )
                id_c = self.cursor.lastrowid

                # 4. Liaison globale
                self.cursor.execute(
                    "INSERT INTO Mesures_Globales (id_session, id_physique, id_couleur) VALUES (%s, %s, %s)",
                    (self.id_session, id_p, id_c)
                )
                self.conn.commit()
            except Exception as e:
                print(f"[SQL ERROR] : {e}")
                try:
                    self.conn.rollback()
                except Exception:
                    pass

    def fermer(self):
        with self.lock:   #nico1
            if self.conn:
                try:
                    self.cursor.close()
                    self.conn.close()
                except Exception:
                    pass
                self.conn = None
