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
        # On v?rifie si l'objet lidar existe
        if self.lidar:
            try:
                # On arr?te les scans et le moteur
                self.lidar.stop()
                self.lidar.stop_motor()
                # On ferme la connexion s?rie proprement
                self.lidar.disconnect()
                print("LIDAR arr?t? proprement.")
            except Exception as e:
                print(f"Note: Erreur lors de la fermeture : {e}")
            finally:
                self.lidar = None # On remet ? None pour ?viter de boucler
        else:
            print("LIDAR d?j? arr?t? ou non d?marr?.")
    
    def gerer(self, scan):
        gestion = CGestion()
        gestion.scan = scan

        gestion.trier_par_angle()
        gestion.filtrer_distance()
        gestion.filtrer_qualite()
        gestion.filtrer_angle()

        obstacle_proche = gestion.get_obstacle_proche()
        return obstacle_proche