class CGestion:
    def __init__(self):
        self.scan = []
    """
    def trier_par_angle(self):
        self.scan.sort(key=lambda x: x[1])  # Trie par angle
        return self.scan

    def filtrer_distance(self, dist_min=100, dist_max=20000):
        self.scan = [p for p in self.scan if dist_min < p[2] < dist_max]
        return self.scan
    
    def filtrer_qualite(self, qualite_min=10):
        self.scan = [p for p in self.scan if p[0] >= qualite_min]
        return self.scan
    
    def filtrer_angle(self, angle_min=30, angle_max=330):
        self.scan = [p for p in self.scan if (0 <= p[1] <= angle_min) or (angle_max <= p[1] <= 360)]
        return self.scan
    """

    def filtrer_tout_en_un(self, scan_brut, dist_min=100, dist_max=4000, qualite_min=10,
                           angle_min=110, angle_max=250):
        """ Filtre tout en un seul passage pour �conomiser le CPU """
        self.scan = [
            p for p in scan_brut 
            if p[0] >= qualite_min              # Qualit�
            and dist_min < p[2] < dist_max      # Distance
            and (p[1] <= angle_min or p[1] >= angle_max) # Angle (devant)
        ]
        return self.scan
    
    def get_secteurs(self):
        gauche = [p[2] for p in self.scan if 210 <= p[1] <= 329]
        droit = [p[2] for p in self.scan if 31 <= p[1] <= 130]
        return gauche, droit

    def get_distance_frontale(self, demi_angle=15):
        """ Distance moyenne dans un cone etroit droit devant (0 +/- demi_angle) """
        pts = [p[2] for p in self.scan if p[1] <= demi_angle or p[1] >= 360 - demi_angle]
        if not pts:
            return 4000
        return sum(pts) / len(pts)

    def distance_frontale_min(self, demi_angle=30):
        """ Distance du point le plus proche DROIT DEVANT (cone +/- demi_angle).
            Sert a l'arret d'urgence : un mur LATERAL (couloir etroit) ne doit PAS
            declencher la marche arriere, seulement un obstacle vraiment devant. """
        dists = [p[2] for p in self.scan if p[1] <= demi_angle or p[1] >= 360 - demi_angle]
        if not dists:
            return 9999
        return min(dists)

    def direction_degagee(self, demi_champ=80, taille_bin=10):
        """ Angle relatif (deg ; + = droite, - = gauche) de la direction la PLUS
            DEGAGEE devant la voiture : on decoupe le champ avant en secteurs et on
            renvoie le centre du secteur dont la distance moyenne est la plus grande. """
        if not self.scan:
            return 0.0
        sommes = {}
        comptes = {}
        for p in self.scan:
            rel = p[1] if p[1] <= 180 else p[1] - 360   # -180..180 (0 = devant, + = droite)
            if -demi_champ <= rel <= demi_champ:
                b = int(round(rel / taille_bin))
                sommes[b] = sommes.get(b, 0.0) + p[2]
                comptes[b] = comptes.get(b, 0) + 1
        if not comptes:
            return 0.0
        best_b = max(comptes, key=lambda b: sommes[b] / comptes[b])
        return best_b * taille_bin

    def direction_ftg(self, demi_champ=110, taille_bin=6, rayon_bulle_deg=20):
        """ FOLLOW-THE-GAP (base sur les echos REELS). Renvoie (cap_relatif_deg, profondeur_mm) :
            - cap_relatif_deg : angle vers lequel piloter (+ = droite, 0 = devant) ;
            - profondeur_mm   : distance moyenne mesuree dans la direction visee.

            Principe : on vise la direction OBSERVEE la plus degagee (distance moyenne par
            secteur la plus grande), apres avoir masque une bulle de securite autour du mur
            le plus proche. On n'invente AUCUN bin libre : un secteur sans echo n'est pas
            considere (sinon il ferait croire a un grand 'trou' fantome droit devant -> la
            voiture resterait collee a 0 / indecise). """
        if not self.scan:
            return 0.0, 0.0

        n_bins = int(2 * demi_champ / taille_bin) + 1
        sommes = [0.0] * n_bins
        comptes = [0] * n_bins
        mind = [None] * n_bins   # distance min observee par bin (pour la bulle de securite)
        for p in self.scan:
            rel = p[1] if p[1] <= 180 else p[1] - 360
            if -demi_champ <= rel <= demi_champ:
                i = int(round((rel + demi_champ) / taille_bin))
                if 0 <= i < n_bins:
                    sommes[i] += p[2]
                    comptes[i] += 1
                    mind[i] = p[2] if mind[i] is None else min(mind[i], p[2])

        observes = [i for i in range(n_bins) if comptes[i] > 0]
        if not observes:
            return 0.0, 0.0

        # Distance moyenne par bin observe ; -1 = bin a ignorer (vide ou masque par la bulle)
        moy = [(sommes[i] / comptes[i]) if comptes[i] > 0 else -1.0 for i in range(n_bins)]

        # Bulle de securite : on EXCLUT les bins autour du mur le plus proche
        i_proche = min(observes, key=lambda i: mind[i])
        demi_bulle = int(rayon_bulle_deg / taille_bin)
        for j in range(max(0, i_proche - demi_bulle), min(n_bins, i_proche + demi_bulle + 1)):
            moy[j] = -1.0

        candidats = [i for i in range(n_bins) if moy[i] >= 0]
        if not candidats:
            # tout est masque par la bulle (mur tout autour) -> on garde le cap devant
            return 0.0, 0.0

        # Cible = bin observe le PLUS DEGAGE ; egalite -> le plus proche du devant (stable)
        centre = n_bins // 2
        i_cible = max(candidats, key=lambda i: (moy[i], -abs(i - centre)))

        cap = -demi_champ + i_cible * taille_bin
        return float(cap), float(moy[i_cible])

    def pente_moyenne(self, distances):
        if len(distances) < 2:
            return 0
        return (distances[-1] - distances[0]) / len(distances)


    def get_obstacle_proche(self):
        if not self.scan : return None
        #valides = [p for p in self.scan if p[2] > 0]
        return min(self.scan, key=lambda x: x[2]) #if valides else None
    
    def get_obstacle_loin(self):
        if not self.scan : return None
        return max(self.scan, key=lambda x: x[2])
    
    