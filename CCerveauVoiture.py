

from CDetection_Lidar import CDetection
from CGestion_Lidar import CGestion


class CCerveau:
    def __init__(self):
        
        self.lidar = CDetection()
        self.gestion_lidar = CGestion()
        self.q, a, d = 0, 0, 0
        self.historique_obstacles = {}
        self.angle_destination = 0
    
    def start_detection(self, etat_signal=None, cerveau=None, signal=None, com=None):
        if self.lidar.start_lidar():
            self.lidar.lidar.clear_input()
            print("Analyse en cours ... (Ctrl+C pour arreter)")
            try:
                for scan in self.lidar.lidar.iter_scans():
                    obstacle_proche = self.lidar.gerer(scan)


                    if obstacle_proche:
                        self.q, self.a, self.d = obstacle_proche
                        self.historique_obstacles[self.d] = self.a

                        if self.d < 200:
                            self.lidar.stop_lidar() #stop moteur
                            print("Obstacle trop proche ! Arrêt d'urgence.")
                            break

                        #print(f"OBSTACLE : Angle: {self.a:.1f}°, Dist: {self.d:.0f}mm (Qualite: {self.q})")

                        if len(self.historique_obstacles) >= 10:
                            self.calcul_destination()
                    else:
                        #print("Chemin libre. . .")
                        pass
                    
                    if com.gestion_start_and_stop(etat_signal, cerveau, signal, com) == False:
                        print("Arrêt de la détection en cours...")
                        self.lidar.stop_lidar()
                        break

                    
                   
            except KeyboardInterrupt:
                pass
            finally:
                self.lidar.stop_lidar()
        else:
            print("Erreur lors du démarrage du LIDAR")
    
    def stop_detection(self):
        self.lidar.stop_lidar()
    
    def calcul_destination(self):
        dist_max = max(self.historique_obstacles.keys())
        
        self.angle_destination = self.historique_obstacles[dist_max]
        
        print("-" * 30)
        print(f"ANALYSE : Sur 10 points, la voie la plus libre est à :")
        print(f"Angle: {self.angle_destination:.1f}° | Distance: {dist_max:.0f}mm")
        print("-" * 30)
        
        self.historique_obstacles.clear()
        
        return self.angle_destination, dist_max