class CGestion:
    def __init__(self):
        self.scan = []

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

    def get_obstacle_proche(self):
        if not self.scan : return None
        valides = [p for p in self.scan if p[2] > 0]
        return min(valides, key=lambda x: x[2]) if valides else None
    
    