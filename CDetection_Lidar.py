from rplidar import RPLidar
from CGestion_Lidar import CGestion

class CDetection:
    def __init__(self):

        self.lidar = None
        self.PORT = '/dev/ttyUSB0'
        self.BAUDERATE = 256000
        self.timeout = 3
    
    def start_lidar(self):
        try:
            self.lidar = RPLidar(self.PORT, self.BAUDERATE, self.timeout)
            print("LIDAR demarre")
            return True
        except Exception as e:
            print(f"Erreur de connexion au LIDAR: {e}")
            return False
    
    def stop_lidar(self):
        if self.lidar:
            print("Arret du LIDAR...")
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("LIDAR arrete proprement")
        else:
            print("LIDAR n'est pas demarre.")
    
    def gerer(self, scan):
        gestion = CGestion()
        gestion.scan = scan

        gestion.trier_par_angle()
        gestion.filtrer_distance()
        gestion.filtrer_qualite()
        gestion.filtrer_angle()

        obstacle_proche = gestion.get_obstacle_proche()
        return obstacle_proche

