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
                           angle_min=90, angle_max=270):
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
    
    