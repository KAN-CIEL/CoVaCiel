from CDetection_Lidar import CDetection
from CGestion_Lidar import CGestion
from CEnregistrement import CEnregistreurCovaciel #nico

import time
import csv
import datetime
import os

class CCerveau:
    def __init__(self, enregistreur=None):   #nico1
        self.lidar = CDetection()
        self.gestion_lidar = CGestion()
        # Instance partagee injectee par main.py ; fallback si appel sans argument #nico1
        self.enregistreur = enregistreur if enregistreur is not None else CEnregistreurCovaciel()  #nico1
        self.last_db_save_t = 0 #nico


        self.distance_arret = 250 # mm  (releve de 200 -> reagit un peu plus tot, moins de contact)
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
        self.Kp_centre = 0.035    # gain proportionnel du centrage (reduit -> moins d'oscillation)
        self.Ki_centre = 0.05
        self.Kd_centre = 0.001    # derivee reduite -> n'amplifie plus le bruit des secteurs

        # Memoires separees pour eviter les coups de raquette
        self.last_erreur_centre = 0
        self.somme_erreur_centre = 0
        self.recul_val = 0
        self.temps_fin_recul = 0   # fin de la fenetre de marche arriere soutenue
        self.DUREE_RECUL = 1.5     # duree minimale de marche arriere (s) -- quoi qu'il arrive

        # --- FUSION navigation : "aller au plus loin" (pilote) + centrage doux + bouclier ---
        # Convention interne : '+' = tourner a GAUCHE, '-' = tourner a DROITE.
        self.K_CENTRE = 0.010      # centrage doux : s'eloigne du mur le plus proche (mur_G ~ mur_D)
        self.K_GAP = 0.4           # "aller au plus loin" : braquage (deg) par degre d'ecart (reduit -> moins de saturation)
        self.SEUIL_BOUCLIER = 900  # mm : sous cette distance, le mur le plus proche repousse (+ tot, + loin)
        self.K_BOUCLIER = 0.02     # force de repulsion (deg par mm sous le seuil) -- reduit : ne sature plus a lui seul
        self.BOUCLIER_MAX = 15     # deg : plafond de la contribution bouclier (anti-saturation)
        self.SEUIL_VIRAGE = 2000   # mm : mur devant plus proche que ca -> etat COURBE (detection plus tot)
        self.SENS_GAP = -1         # inversion globale gauche/droite (convention LIDAR/servo inversee cote materiel)
        self.BIAIS_FORCE = 10      # braquage FORCE ajoute dans le sens du virage (deg) ; le PID regule autour

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

    def _enregistrer_bdd(self, com, arret_urgence, angle_destination, servo_val, plus_proche, plus_loin):  #nico1
        """ Prepare et envoie les 4 valeurs a la BDD (appel bride a 5Hz par la boucle). #nico1
            - angle  : angle physique reel en degres (self.angle), PAS le PWM servo (62-109)
            - lidar  : 'Oui' si arret d'urgence, 'Non' si voie libre
            - vitesse / batterie : floats parses en asynchrone par le thread UART (struct) """  #nico1
        # 1. On demande vitesse (0x06) et batterie (0x09) ; les reponses sont parsees
        #    en asynchrone par CCommunication._listen et stockees (thread-safe).
        com.send_command(0x06, b'\x00\x00\x00\x00\x00\x00')   # GET vitesse IMU
        com.send_command(0x09, b'\x00\x00\x00\x00\x00\x00')   # GET batterie

        # 2. Relecture des dernieres valeurs connues (defaut robuste si com incomplet)
        vitesse_val = com.get_vitesse() if hasattr(com, "get_vitesse") else 0.0
        batterie_val = com.get_batterie() if hasattr(com, "get_batterie") else 0.0

        # 3. Statut lidar (chaine) et angle reel en degres
        lidar_statut = "Oui" if arret_urgence else "Non"

        # 4. Sauvegarde BDD (robuste / thread-safe cote CEnregistrement)
        self.enregistreur.sauvegarder_donnees(
            vitesse=vitesse_val,
            angle=self.angle,
            lidar_statut=lidar_statut,
            batterie_val=batterie_val,
        )

        # 5. Log CSV de debug navigation (separe de la BDD)
        d_p = plus_proche[2] if plus_proche else "N/A"
        d_l = plus_loin[2] if plus_loin else "N/A"
        self.log_data(self.etat_voie, angle_destination, servo_val, d_p, d_l)

    def start_detection(self, signal, com):
        if self.lidar.start_lidar():
            com.start()
            print("Analyse active. Mode Navigation engage. Logging actif.")
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

                        t_now = time.time()

                        # --- ARRET D'URGENCE EXTERNE (XBEE) : PRIORITAIRE sur TOUT, meme pendant
                        #     un recul. Teste en premier a chaque scan -> stop immediat garanti.
                        if signal.read_signal() == "STOP":
                            self.temps_fin_recul = 0   # annule une marche arriere en cours
                            com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')  # vitesse 0
                            com.send_command(0x07, b'\x56\x00\x00\x00\x00\x00')  # roues au centre
                            print("!!! STOP XBEE -> arret")
                            break

                        # Distance de l'obstacle DROIT DEVANT (cone etroit) : un mur lateral de
                        # couloir ne doit PAS declencher la marche arriere.
                        dist_frontale = self.gestion_lidar.distance_frontale_min()

                        # Statut d'arret d'urgence (obstacle proche DEVANT) -> sert au champ "lidar" #nico1
                        arret_urgence = bool(dist_frontale < self.distance_arret)  #nico1

                        # --- ENREGISTREMENT BDD bride a 5Hz (TOUJOURS, meme en arret d'urgence) --- #nico1
                        if t_now - self.last_db_save_t > 0.2:                                        #nico1
                            self._enregistrer_bdd(com, arret_urgence, angle_destination, servo_val,  #nico1
                                                  plus_proche, plus_loin)                            #nico1
                            self.last_db_save_t = t_now                                              #nico1

                        # --- MARCHE ARRIERE SOUTENUE (>= DUREE_RECUL), seulement si obstacle DEVANT ---
                        # Nouveau declenchement : on choisit le sens et on arme une fenetre de recul.
                        if dist_frontale < self.distance_arret and t_now >= self.temps_fin_recul:
                                self.temps_fin_recul = t_now + self.DUREE_RECUL
                                # Convention G/D inversee cote materiel (cf. SENS_GAP) : 0x3e <-> 0x6d
                                if self.etat_voie == "COURBE_GAUCHE":
                                    self.recul_val = 0x6d
                                elif self.etat_voie == "COURBE_DROITE":
                                    self.recul_val = 0x3e
                                elif self.etat_voie == "LIGNE_DROITE":
                                    if plus_proche and plus_proche[1] < 60:
                                        self.recul_val = 0x3e
                                    elif plus_proche and plus_proche[1] > 300:
                                        self.recul_val = 0x6d
                                print(f"!!! STOP : {dist_frontale:.0f}mm -> recul {self.DUREE_RECUL}s !!!")

                        # Tant qu'on est dans la fenetre, on RECULE en continu (quoi qu'il arrive)
                        if t_now < self.temps_fin_recul:
                                trame_recul = bytes([self.recul_val, 0, 0, 0, 0, 0])
                                com.send_command(0x05, b'\xed\x00\x00\x00\x00\x00') #recul
                                com.send_command(0x07, trame_recul)
                                print("recul")
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

                            # 3. CIBLE = "aller au plus loin" (gap) + PID de centrage + braquage FORCE en virage + bouclier
                            # Convention interne : '+' = tourner a GAUCHE, '-' = tourner a DROITE.

                            # a) PID de centrage (P + I + D) : regule la trajectoire en douceur.
                            target = -self.calc_angle_centre(gauche, droit, dt)

                            # b) Aller au plus loin : direction la plus degagee.
                            #    rel_gap > 0 = ouverture a DROITE -> tourner a DROITE (-).
                            rel_gap = self.gestion_lidar.direction_degagee()
                            target += -self.K_GAP * rel_gap

                            # c) En VIRAGE : on FORCE un braquage (valeur fixe ajoutee dans le sens du
                            #    virage) ; le PID + le gap regulent autour -> plus de "braquage fort minute".
                            if self.etat_voie in ("COURBE_DROITE", "COURBE_GAUCHE"):
                                sens = 1 if self.etat_voie == "COURBE_GAUCHE" else -1
                                target += sens * self.BIAIS_FORCE

                            # d) Bouclier anti-mur LATERAL : mur a DROITE (25-90) -> GAUCHE ;
                            #    mur a GAUCHE (270-335) -> DROITE. Force BORNEE (BOUCLIER_MAX) pour ne pas
                            #    saturer a elle seule, et zones laterales seulement : un point pile DEVANT
                            #    (~0deg) ne doit PAS faire braquer a gauche arbitrairement (il est deja
                            #    gere par le gap et la marche arriere).
                            if plus_proche and plus_proche[2] < self.SEUIL_BOUCLIER:
                                ang_p = plus_proche[1]
                                force = min(self.BOUCLIER_MAX,
                                            self.K_BOUCLIER * (self.SEUIL_BOUCLIER - plus_proche[2]))
                                if 25 <= ang_p <= 90:
                                    target += force      # mur a droite -> tourner a gauche
                                elif 270 <= ang_p <= 335:
                                    target -= force      # mur a gauche -> tourner a droite

                            # Inversion globale si tout le braquage part du mauvais cote (cf. SENS_GAP)
                            target *= self.SENS_GAP

                            target = max(-30, min(30, target))

                            # 4. Lissage passe-bas (70% ancienne, 30% nouvelle = braquage progressif, moins sec)
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

                            print("avance")

                            self.last_cmd_t = t_now

                    except ValueError:
                        self.lidar.lidar.clear_input()
                        continue

            except Exception as e:
                print(f"Erreur majeure : {e}")
            finally:
                self.lidar.stop_lidar()
                # NB : on NE ferme PAS l'enregistreur ici : c'est une instance partagee qui #nico1
                # doit persister entre les courses (GO/STOP). Elle se ferme a l'arret du programme. #nico1

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

    def calc_virage(self, gauche, droit):
        """ Etat = LIGNE_DROITE par defaut. On passe en COURBE seulement quand un mur est
            detecte DEVANT (vrai virage), pas a chaque correction. Sert surtout a regler la
            vitesse. Hysteresis sur la distance frontale pour eviter le clignotement. """
        dist_devant = self.gestion_lidar.get_distance_frontale(demi_angle=25)

        en_courbe = self.etat_voie in ("COURBE_DROITE", "COURBE_GAUCHE")
        # On ne SORT du virage que quand la voie redevient bien degagee devant (hysteresis)
        seuil = self.SEUIL_VIRAGE * 1.4 if en_courbe else self.SEUIL_VIRAGE
        if dist_devant >= seuil:
            return "LIGNE_DROITE"

        # Mur devant -> virage : on nomme l'etat selon le cote le plus ouvert
        if not gauche or not droit or len(gauche) == 0 or len(droit) == 0:
            return self.etat_voie if en_courbe else "LIGNE_DROITE"
        moy_g = sum(gauche) / len(gauche)
        moy_d = sum(droit) / len(droit)
        if moy_g >= moy_d:
            return "COURBE_GAUCHE"   # plus d'espace a gauche
        return "COURBE_DROITE"

    def stop_detection(self):
        self.lidar.stop_lidar()
