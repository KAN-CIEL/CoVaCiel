

from CDetection_Lidar import CDetection
from CGestion_Lidar import CGestion


class CCerveau:
    def __init__(self):
        
        self.lidar = CDetection()
        self.gestion_lidar = CGestion()
        self.q, a, d = 0, 0, 0
        self.historique_obstacles = {}
        self.angle_destination = 0
    
    def start_detection(self, signal, com):
        if self.lidar.start_lidar():
            print("Analyse active. Mode Turbo engag�.")
            try:
                # On r�cup�re l'it�rateur
                scans = self.lidar.lidar.iter_scans()
                
                while True:
                    try:
                        scan = next(scans)
                        
                        # OPTIMISATION : On utilise la nouvelle fonction 1 seul passage
                        obstacle_proche = self.gestion_lidar.filtrer_tout_en_un(scan)
                        
                        # On cherche le plus proche uniquement si on a des points
                        plus_proche = self.gestion_lidar.get_obstacle_proche()

                        if plus_proche:
                            q, a, d = plus_proche
                            self.historique_obstacles[d] = a
                            
                            if d < 250:
                                print("!!! STOP : OBSTACLE !!!")
                                com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')# STOP MOTEUR
                                #break
                            elif d >= 250:
                                com.send_command(0x05, b'\x1E\x00\x00\x00\x00\x00')# Avancer

                            if len(self.historique_obstacles) >= 10:
                                self.calcul_destination()
                        
                        # V�rification signal XBEE
                        msg = signal.read_signal()
                        if msg == "STOP": 
                            com.send_command(0x05, b'\x00\x00\x00\x00\x00\x00')# STOP MOTEUR
                            break

                    except ValueError: # Souvent l'erreur derri�re "mismatch"
                        print("Sync lost... purger le buffer")
                        self.lidar.lidar.clear_input()
                        continue
                        
            except Exception as e:
                print(f"Erreur majeure : {e}")
            finally:
                self.lidar.stop_lidar()
    
    def stop_detection(self):
        self.lidar.stop_lidar()
    
    def calcul_destination(self):
        if not self.historique_obstacles: 
            return 0, 0
            
        dist_max = max(self.historique_obstacles.keys())
        self.angle_destination = self.historique_obstacles[dist_max]
        
        print(f"--- DESTINATION : Angle {self.angle_destination:.1f}� | Voie libre � {dist_max:.0f}mm ---")
        
        # LOGIQUE DE DIRECTION :
        # Si angle entre 0 et 30 -> Tourner un peu � droite
        # Si angle entre 330 et 360 -> Tourner un peu � gauche
        # (A adapter selon le montage de ton LIDAR)
        
        self.historique_obstacles.clear()
        return self.angle_destination, dist_max