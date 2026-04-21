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
    
    