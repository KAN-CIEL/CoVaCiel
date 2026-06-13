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

        # Detection de BLOCAGE (coince contre un mur LATERAL en virage : le recul frontal
        # ne se declenche pas). Signature : forte butee maintenue + scene figee (on n'avance plus).
        self.obs_stuck_ref = None  # distance de reference de l'obstacle le plus proche
        self.t_progres = 0         # derniere fois que la scene a change (= on a avance)
        self.DUREE_STUCK = 1.5     # s sans progres a forte butee -> considere coince
        self.STUCK_BAND = 120      # mm : variation mini de l'obstacle proche pour dire "ca avance"
        self.STUCK_LOCK = 26       # deg : on ne teste le blocage qu'a forte butee

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

        # --- FOLLOW-THE-GAP (braquage principal) ---
        self.K_STEER = 0.8         # braquage (deg de servo) par degre d'ecart du cap FTG vise
        self.FTG_BULLE_DEG = 20    # demi-largeur (deg) de la bulle de securite autour du mur le + proche
        # Centrage doux : le FTG seul longe le mur interieur (il "coupe") ; cette correction
        # repousse du mur lateral le plus proche pour rester au milieu du couloir EN LIGNE DROITE.
        self.K_CENTRE_FTG = 0.008  # deg de braquage par mm d'ecart (mur_gauche - mur_droit)
        self.CENTRE_MAX = 8        # deg : plafond de la correction de centrage (reste secondaire au FTG)
        # Renfort de virage : le FTG pur se redresse trop tot et SOUS-VIRE en fin de courbe.
        # Quand un mur se rapproche DEVANT en courbe, on accentue le braquage dans le sens du virage.
        self.SEUIL_RENFORT = 1200  # mm : sous cette distance frontale, on renforce le virage
        self.K_RENFORT = 0.02      # deg de braquage par mm sous le seuil
        self.RENFORT_MAX = 18      # deg : plafond du renfort de virage

        # REPULSION PRIORITAIRE DES MURS : on suit la corde (FTG) tant qu'on est loin des
        # murs ; mais des qu'un mur lateral devient proche, un braquage de FUITE ECRASE
        # progressivement le FTG (priorite a ne pas toucher le mur). A DIST_CRITIQUE et en
        # deca, la fuite est totale (la voiture s'eloigne a fond du mur).
        self.SEUIL_REPULSE = 350   # mm : sous cette distance (mur le + proche), la repulsion s'active
        self.DIST_CRITIQUE = 130   # mm : a cette distance, la fuite est PRIORITAIRE a 100%
        self.FUITE_MAX = 30        # deg : braquage de fuite max (butee opposee au mur)

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
                # Le LIDAR est lu par un THREAD dedie qui draine le buffer en continu
                # (cf. CDetection) -> plus de debordement/desync. La nav consomme juste le
                # DERNIER scan disponible, a son rythme, sans bloquer le LIDAR.
                self.lidar.start_reader()

                angle_destination = 0.0
                servo_val = 86
                derniere_seq = -1

                while True:
                    try:
                        # Dernier scan du thread lecteur ; si pas de NOUVEAU scan -> petite attente.
                        seq, scan = self.lidar.get_latest_scan()
                        if scan is None or seq == derniere_seq:
                            time.sleep(0.005)
                            continue
                        derniere_seq = seq

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
                                # Recul a -30 (0xe2) au lieu de -19 (0xed) : meme puissance que la
                                # marche avant, pour vraiment decoincer la voiture (sinon immobile).
                                com.send_command(0x05, b'\xe2\x00\x00\x00\x00\x00') #recul
                                com.send_command(0x07, trame_recul)
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

                            # 3. CIBLE = FOLLOW-THE-GAP : on vise le CENTRE du plus grand espace
                            #    libre devant, apres avoir masque une bulle de securite autour du mur
                            #    le plus proche. UN SEUL terme coherent -> evite les murs par
                            #    construction, fini les 4 termes qui se battaient et qui saturaient.
                            #    cap > 0 = ouverture a DROITE.
                            cap, profondeur_gap = self.gestion_lidar.direction_ftg(
                                rayon_bulle_deg=self.FTG_BULLE_DEG,
                            )

                            # a) Braquage FTG : viser l'ouverture la plus degagee.
                            target = -self.K_STEER * cap

                            # b) Centrage doux (LIGNE DROITE) : repousse du mur lateral le plus
                            #    proche pour rester au milieu du couloir. ATTENUE quand le cap FTG
                            #    est fort (virage) : sinon il pousse vers l'EXTERIEUR et fait
                            #    SOUS-VIRER la voiture. moy_g = mur GAUCHE, moy_d = mur DROIT.
                            if gauche and droit:
                                moy_g = sum(gauche) / len(gauche)
                                moy_d = sum(droit) / len(droit)
                                corr = self.K_CENTRE_FTG * (moy_g - moy_d)
                                corr = max(-self.CENTRE_MAX, min(self.CENTRE_MAX, corr))
                                attenuation = max(0.0, 1.0 - abs(cap) / 25.0)  # 1 a 0deg -> 0 des 25deg
                                target += corr * attenuation

                            # c) Renfort de virage : quand un mur se rapproche DEVANT en courbe,
                            #    on accentue le braquage dans le sens du virage (le FTG pur se
                            #    redresse trop tot -> clippe le mur). D'autant plus fort que le mur
                            #    frontal est proche. pre-SENS : negatif = DROITE (apres *SENS_GAP).
                            if self.etat_voie in ("COURBE_GAUCHE", "COURBE_DROITE") \
                                    and dist_frontale < self.SEUIL_RENFORT:
                                renfort = min(self.RENFORT_MAX,
                                              self.K_RENFORT * (self.SEUIL_RENFORT - dist_frontale))
                                if self.etat_voie == "COURBE_DROITE":
                                    target -= renfort   # tourne PLUS a droite
                                else:
                                    target += renfort   # tourne PLUS a gauche

                            # d) REPULSION PRIORITAIRE DES MURS : tant qu'on est loin, on suit la
                            #    corde (FTG ci-dessus). Des qu'un mur devient proche -- SUR TOUT LE
                            #    CHAMP AVANT (le point le plus proche, quel que soit son angle, pas
                            #    seulement les secteurs lateraux) -- un braquage de FUITE ECRASE
                            #    progressivement le FTG -> la voiture reste ECARTEE des murs.
                            #    pre-SENS : negatif = DROITE, positif = GAUCHE (apres SENS_GAP).
                            if plus_proche and plus_proche[2] < self.SEUIL_REPULSE:
                                # gravite : 0 a SEUIL_REPULSE -> 1 a DIST_CRITIQUE (et en deca)
                                grav = (self.SEUIL_REPULSE - plus_proche[2]) / \
                                       (self.SEUIL_REPULSE - self.DIST_CRITIQUE)
                                grav = max(0.0, min(1.0, grav))
                                # angle du mur le plus proche : rel > 0 = a DROITE, < 0 = a GAUCHE
                                ang_p = plus_proche[1]
                                rel = ang_p if ang_p <= 180 else ang_p - 360
                                # mur a DROITE -> fuir a GAUCHE (pre-SENS positif) ; a gauche -> a droite
                                fuite = self.FUITE_MAX if rel > 0 else -self.FUITE_MAX
                                # melange a PRIORITE : plus le mur est proche, plus la fuite domine
                                target = (1.0 - grav) * target + grav * fuite

                            # Inversion materielle (cf. SENS_GAP) appliquee a l'ensemble.
                            target *= self.SENS_GAP

                            target = max(-30, min(30, target))

                            # 4. Lissage passe-bas (70% ancienne, 30% nouvelle = braquage progressif, moins sec)
                            angle_destination = (angle_destination * 0.7) + (target * 0.3)

                            # Bornage final et Conversion
                            angle_destination = max(-30, min(30, angle_destination))

                            self.angle = angle_destination # Pour l'enregistrement dans la base de donnees

                            # --- DETECTION DE BLOCAGE (coince contre un mur en virage) ---
                            # On suit l'obstacle le plus proche : tant qu'il VARIE, on progresse.
                            # S'il reste fige (scene immobile) ALORS qu'on est a forte butee, c'est
                            # qu'on est coince -> on arme un recul de desengagement (mur lateral, donc
                            # le recul frontal ci-dessus ne s'est pas declenche).
                            d_proche = plus_proche[2] if plus_proche else None
                            if d_proche is not None:
                                if self.obs_stuck_ref is None or abs(d_proche - self.obs_stuck_ref) > self.STUCK_BAND:
                                    self.obs_stuck_ref = d_proche
                                    self.t_progres = t_now
                            if (abs(angle_destination) >= self.STUCK_LOCK
                                    and t_now - self.t_progres > self.DUREE_STUCK
                                    and t_now >= self.temps_fin_recul):
                                self.temps_fin_recul = t_now + self.DUREE_RECUL
                                # On RECULE roues vers le cote le PLUS OUVERT -> la voiture recule
                                # VERS l'espace libre (sur un virage gauche : roues a gauche).
                                # 0x3e = servo 62 = roues GAUCHE ; 0x6d = servo 109 = roues DROITE.
                                moy_g = sum(gauche) / len(gauche) if gauche else 0
                                moy_d = sum(droit) / len(droit) if droit else 0
                                self.recul_val = 0x3e if moy_g >= moy_d else 0x6d
                                self.t_progres = t_now
                                print(f"!!! COINCE en virage ({d_proche:.0f}mm fige) -> recul desengagement")

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

                    except ValueError:
                        # Erreur de traitement d'un scan : on passe au suivant. On ne touche
                        # PAS au LIDAR ici (c'est le thread lecteur qui gere le flux serie).
                        continue

            except Exception as e:
                print(f"Erreur majeure : {e}")
            finally:
                self.lidar.stop_reader()   # arrete le thread lecteur AVANT de couper le LIDAR
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
