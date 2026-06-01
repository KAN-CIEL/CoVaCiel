from CDetection_Lidar import CDetection
from CGestion_Lidar import CGestion
from CEnregistrement import CEnregistreurCovaciel #nico

import time
import csv
import datetime
import os

class CCerveau:
    def __init__(self):
        self.lidar = CDetection()
        self.gestion_lidar = CGestion()
        self.enregistreur = CEnregistreurCovaciel() #nico
        self.last_db_save_t = 0 #nico


        self.distance_arret = 200 # mm
        self.distances_urgence = 400
        self.SEUIL_MUR_INT = 450 # mm
        self.SEUIL_MUR_EXT = 300 # mm

        self.last_cmd_t = 0
        self.cmd_interval = 0.05 # 20Hz (50ms)
        self.etat_voie = "LIGNE_DROITE"
        self.log_file = "lidar_logs.csv"

        # Debounce de SORTIE de virage : il faut voir "LIGNE_DROITE" en continu
        # pendant DUREE_CONFIRM_SORTIE avant de quitter un virage (anti-clignotement).
        self.t_ligne_candidate = 0
        self.DUREE_CONFIRM_SORTIE = 0.4

        # Gains PID pour le centrage (Ligne Droite)
        # I et D sont normalises par dt : valeurs rescalees pour rester
        # equivalentes a l'ancien reglage a 20 Hz (Ki/dt et Kd*dt avec dt=0.05).
        self.Kp_centre = 0.028    # moins de gain proportionnel (anti-oscillation a haute vitesse)
        self.Ki_centre = 0.02
        self.Kd_centre = 0.0012   # plus d'amortissement (freine les ondulations)

        # Memoires separees pour eviter les coups de raquette
        self.last_erreur_centre = 0
        self.somme_erreur_centre = 0
        self.recul_val = 0

        self.DISTANCE_CIBLE_VIRAGE = 320   # (utilise par l'ancien suivi de mur, desormais inactif)
        self.Kp_mur = 0.05

        # Braquage proportionnel en virage : angle = -Kp_courbe * (asymetrie des murs).
        # 0.025 -> ~15deg a l'entree (diff=600), sature a 30deg vers diff=1200.
        self.Kp_courbe = 0.025

        #enregistrement
        self.angle = 0

        self._init_log_file()

    def _init_log_file(self):
        """ Cree le fichier de log avec l'en-tete """
        with open(self.log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Etat", "Angle_Dest", "Servo", "Obs_Proche", "Obs_Loin"])

    def log_data(self, etat, angle_dest, servo_val, obs_proche_dist, obs_loin_dist):
        """ Enregistre les donnees dans le CSV """
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open(self.log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, etat, f"{angle_dest:.1f}", servo_val, obs_proche_dist, obs_loin_dist])

    def start_detection(self, signal, com):
        if self.lidar.start_lidar():
            com.start()
            print("Analyse active. Mode Navigation engage. Logging actif.")
            try:
                scans = self.lidar.lidar.iter_scans()

                angle_destination = 0.0
                servo_val = 86
                vitesse_imu = 0 #nico

                for scan in scans:
                    try:
                        # 1. Filtrage et Analyse
                        self.gestion_lidar.filtrer_tout_en_un(scan)

                        plus_proche = self.gestion_lidar.get_obstacle_proche()
                        plus_loin = self.gestion_lidar.get_obstacle_loin()
                        gauche, droit = self.gestion_lidar.get_secteurs()

                        t_now = time.time()

                        # --- LOGIQUE DE SECURITE / MARCHE ARRIERE ---
                        # On ne traite la marche arriere que s'il existe un obstacle proche.
                        if plus_proche:
                            _, _, d_proche_val = plus_proche

                            if d_proche_val < self.distance_arret :
                                if self.etat_voie == "COURBE_GAUCHE":
                                    self.recul_val = 0x3e
                                elif self.etat_voie == "COURBE_DROITE":
                                    self.recul_val = 0x6d
                                elif self.etat_voie == "LIGNE_DROITE":
                                    if plus_proche[1] < 60:
                                        self.recul_val = 0x6d
                                    elif plus_proche[1] > 300:
                                        self.recul_val = 0x3e

                                print(f"!!! STOP : {d_proche_val}mm !!!")
                                trame_recul = bytes([self.recul_val, 0, 0, 0, 0, 0])
                                com.send_command(0x05, b'\xed\x00\x00\x00\x00\x00') #recul
                                com.send_command(0x07, trame_recul)
                                self.temps_fin_recul = t_now + 2.5
                                continue

                        # --- LOGIQUE DE NAVIGATION (20Hz) ---
                        # Desormais executee a CHAQUE scan, qu'il y ait un obstacle proche ou non.
                        if t_now - self.last_cmd_t > self.cmd_interval:

                            # dt reel depuis le dernier calcul (garde-fou : 1ere iteration ou trou de scan)
                            dt = t_now - self.last_cmd_t
                            if dt <= 0 or dt > 0.5:
                                dt = self.cmd_interval

                            # 1. Etat theorique de la route devant nous
                            nouvel_etat = self.calc_virage(gauche, droit)

                            # 2. Machine a etats avec debounce :
                            #    - ENTREE en virage : immediate (on doit reagir vite)
                            #    - SORTIE de virage : seulement si "LIGNE_DROITE" tient DUREE_CONFIRM_SORTIE
                            #    - virage oppose : bascule immediate (rare, demande une forte asymetrie)
                            if self.etat_voie in ("COURBE_DROITE", "COURBE_GAUCHE"):
                                if nouvel_etat == self.etat_voie:
                                    self.t_ligne_candidate = 0  # virage confirme, on annule la sortie
                                elif nouvel_etat == "LIGNE_DROITE":
                                    if self.t_ligne_candidate == 0:
                                        self.t_ligne_candidate = t_now
                                    elif t_now - self.t_ligne_candidate >= self.DUREE_CONFIRM_SORTIE:
                                        self.etat_voie = "LIGNE_DROITE"
                                        self.t_ligne_candidate = 0
                                        self.somme_erreur_centre = 0
                                        self.last_erreur_centre = 0
                                else:  # virage oppose
                                    self.etat_voie = nouvel_etat
                                    self.t_ligne_candidate = 0
                                    self.somme_erreur_centre = 0
                                    self.last_erreur_centre = 0
                            else:  # LIGNE_DROITE : on entre en virage des qu'il est detecte
                                if nouvel_etat != "LIGNE_DROITE":
                                    self.etat_voie = nouvel_etat
                                    self.t_ligne_candidate = 0
                                    self.somme_erreur_centre = 0
                                    self.last_erreur_centre = 0

                            # 3. Calcul de la cible
                            if self.etat_voie in ("COURBE_DROITE", "COURBE_GAUCHE"):
                                # Braquage PROPORTIONNEL a la severite du virage
                                target = self.calc_angle_courbe(gauche, droit)
                            else: # LIGNE_DROITE
                                # PID de centrage
                                target = self.calc_angle_centre(gauche, droit, dt)

                            # 4. Lissage passe-bas (70% ancienne valeur, 30% nouvelle)
                            angle_destination = (angle_destination * 0.7) + (target * 0.3)

                            # Bornage final et Conversion
                            angle_destination = max(-30, min(30, angle_destination))

                            self.angle = angle_destination # Pour l'enregistrement dans la base de donnees

                            servo_val = self.conversion_angle(angle_destination)

                            # Envoi des commandes
                            trame_servo = bytes([servo_val, 0, 0, 0, 0, 0])
                            com.send_command(0x07, trame_servo)

                            if self.etat_voie == "LIGNE_DROITE":
                                com.send_command(0x05, b'\x28\x00\x1e\x00\x00\x00') # Vitesse stable
                            else:
                                #com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')
                                com.send_command(0x05, b'\x1e\x00\x00\x00\x00\x00') # Vitesse virage

                            self.last_cmd_t = t_now

                        # --- nico: Sauvegarde des donnees ---
                        # On ne sauvegarde pas a 20Hz (trop rapide pour MySQL), on le fait a 5Hz (toutes les 0.2s)
                        if t_now - self.last_db_save_t > 0.2:
                            # lidar_val = distance de l'obstacle le plus proche
                            dist_lidar = plus_proche[2] if plus_proche else 0

                            # 2. Lecture de la Batterie (Commande 0x09)
                            batterie_val = 0.0
                            trame_batterie = com.send_command(0x09, b'\x00\x00\x00\x00\x00\x00')

                            # Si la carte repond bien et renvoie assez d'octets
                            if trame_batterie and len(trame_batterie) >= 2:
                                # On colle l'octet 0 (poids fort) et l'octet 1 (poids faible)
                                batterie_brute = (trame_batterie[0] << 8) | trame_batterie[1]
                                # On divise par 100 comme demande
                                batterie_val = batterie_brute / 100.0

                            # 3. Lecture de la vitesse IMU (Commande 0x06)
                            trame_vitesse_imu = com.send_command(0x06, b'\x00\x00\x00\x00\x00\x00') #nico

                            if trame_vitesse_imu and len(trame_vitesse_imu) >= 2: #nico
                                vitesse_imu = (trame_vitesse_imu[0] << 8) | trame_vitesse_imu[1] #nico
                                vitesse_imu = vitesse_imu / 100.0 #nico

                            # On appelle la fonction avec les variables de CCerveau
                            self.enregistreur.sauvegarder_donnees(
                                vitesse=vitesse_imu,
                                angle=servo_val,
                                lidar_val=dist_lidar,
                                batterie_val=batterie_val
                            )
                            self.last_db_save_t = t_now
                            #plus nico

                            # Log des donnees
                            d_p = plus_proche[2] if plus_proche else "N/A"
                            d_l = plus_loin[2] if plus_loin else "N/A"
                            self.log_data(self.etat_voie, angle_destination, servo_val, d_p, d_l)

                        # Signal d'arret externe
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
                self.enregistreur.fermer() #nico

    def conversion_angle(self, angle):
        """ Mappe -30/+30 vers 62/109 (Milieu 86) """
        angle_norm = max(-30, min(30, angle))
        servo_angle = 86 + (angle_norm * (27 / 30))
        return max(62, min(109, int(round(servo_angle))))

    def calc_angle_centre(self, gauche, droit, dt):
        """ Calcule l'erreur de centrage (PID normalise par dt) """
        if not gauche or not droit or len(gauche) == 0 or len(droit) == 0:
            return 0

        moy_g = sum(gauche)/len(gauche)
        moy_d = sum(droit)/len(droit)

        erreur = moy_g - moy_d

        # Integrale en erreur.secondes (clamp ramene a +/-50 pour un effet equivalent a l'ancien +/-1000)
        self.somme_erreur_centre += erreur * dt
        self.somme_erreur_centre = max(-50, min(50, self.somme_erreur_centre))

        # Derivee en erreur/seconde
        derivee = (erreur - self.last_erreur_centre) / dt if dt > 0 else 0
        self.last_erreur_centre = erreur

        commande = (erreur * self.Kp_centre) + (self.somme_erreur_centre * self.Ki_centre) + (derivee * self.Kd_centre)
        return -commande

    def calc_angle_courbe(self, gauche, droit):
        """ Braquage PROPORTIONNEL a la severite du virage (asymetrie des murs).
            Courbe douce -> petit angle ; virage serre -> grand angle. """
        if not gauche or not droit or len(gauche) == 0 or len(droit) == 0:
            return 0
        moy_g = sum(gauche) / len(gauche)
        moy_d = sum(droit) / len(droit)
        diff = moy_g - moy_d   # > 0 : plus d'espace a gauche -> virage a DROITE (angle negatif)
        target = -self.Kp_courbe * diff
        return max(-30, min(30, target))

    def calc_virage(self, gauche, droit):
        # Si un cote manque (mur exterieur hors de portee en virage serre),
        # on NE repart PAS en ligne droite : on garde l'etat courant.
        if not gauche or not droit or len(gauche) == 0 or len(droit) == 0:
            if self.etat_voie in ("COURBE_DROITE", "COURBE_GAUCHE"):
                return self.etat_voie
            return "LIGNE_DROITE"

        moy_g = sum(gauche) / len(gauche)
        moy_d = sum(droit) / len(droit)
        diff = moy_g - moy_d   # > 0 : mur gauche plus loin -> virage a DROITE

        dist_devant = self.gestion_lidar.get_distance_frontale()

        SEUIL_ENTREE = 600     # asymetrie pour ENTRER en virage
        SEUIL_SORTIE = 250     # asymetrie pour SORTIR (hysteresis : plus bas)
        SEUIL_FRONTAL = 2500

        # Hysteresis : une fois en virage, on y RESTE tant que l'asymetrie reste
        # marquee. Empeche le clignotement COURBE/LIGNE au milieu d'un virage.
        if self.etat_voie == "COURBE_DROITE":
            if diff > SEUIL_SORTIE:
                return "COURBE_DROITE"
        elif self.etat_voie == "COURBE_GAUCHE":
            if -diff > SEUIL_SORTIE:
                return "COURBE_GAUCHE"

        # Conditions d'ENTREE (la distance frontale ne sert qu'a anticiper l'entree)
        if diff > SEUIL_ENTREE and dist_devant < SEUIL_FRONTAL:
            return "COURBE_DROITE"
        elif -diff > SEUIL_ENTREE and dist_devant < SEUIL_FRONTAL:
            return "COURBE_GAUCHE"

        return "LIGNE_DROITE"

    def calc_angle_suivi_mur(self, distances, cote="GAUCHE"):
        """
        Calcule un ajustement d'angle pour maintenir une distance cible
        avec le mur interieur du virage.
        """
        if not distances or len(distances) == 0:
            return 0

        moyenne_distances = sum(distances) / len(distances)

        # L'erreur est la difference entre ou on est et ou on veut etre
        # Si erreur > 0 : on est trop pres. Si erreur < 0 : on est trop loin.
        erreur = self.DISTANCE_CIBLE_VIRAGE - moyenne_distances

        # PID simplifie pour le virage
        commande = erreur * self.Kp_mur

        if cote == "GAUCHE":
            # En virage a GAUCHE, le mur interieur est a GAUCHE.
            # Si commande > 0 (trop pres), on soustrait a l'angle (on redresse vers la droite).
            return -commande
        else:
            # En virage a DROITE, le mur interieur est a DROITE.
            # Si commande > 0 (trop pres), on ajoute a l'angle (on redresse vers la gauche).
            return commande

    def stop_detection(self):
        self.lidar.stop_lidar()
