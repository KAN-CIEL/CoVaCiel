from rplidar import RPLidar
from CGestion_Lidar import CGestion
import time

class CDetection:
    def __init__(self):

        self.lidar = None
        self.PORT = '/dev/lidar'   # symlink fixe (udev) -> CP2102 du RPLidar
        self.BAUDERATE = 256000
        self.timeout = 3

    def start_lidar(self):
        try:
            self.lidar = RPLidar(self.PORT, self.BAUDERATE, self.timeout)
            # Remise a l'etat repos pour eviter "Descriptor length mismatch" :
            # un run precedent a pu laisser le LIDAR en mode scan -> on arrete le
            # scan, le moteur, et on vide le buffer serie residuel avant de relancer.
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.clear_input()
            time.sleep(0.5)
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